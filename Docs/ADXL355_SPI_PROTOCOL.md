# ADXL355 SPI Protocol — FT232H MPSSE

## Status

Working. Sensor init, register R/W, FIFO streaming, and HDF5 collection all functional. Validated with 7-point data integrity check (`validate-data` binary).

- 1 / 2 / 4 kHz ODR: lossless (measured 2026-10-15, see Performance)

## Current Configuration

- SPI clock: 1 MHz
- MPSSE latency timer: 1 ms
- SPI mode: 0 (CPOL=0, CPHA=0)
- CS: DBUS3, active low
- Sensor range: ±2g (scale factor 256,000 LSB/g)
- FIFO: 96 entries = 32 complete XYZ samples (3 entries per sample, 3 bytes per entry)
- FIFO overflow: `STATUS.FIFO_OVR` (0x04 bit 2) latches when a sample is dropped and clears on read. Each poll reads STATUS + FIFO_ENTRIES (0x04–0x05) in one 2-byte transaction.
- Temperature: read separately via register, not available in FIFO

### Performance

Each FIFO poll is 2 split Write+Read transactions (`FIFO_ENTRIES`, then the data). A register read takes ~0.4 ms and a full poll < 1 ms, far inside the 8 ms it takes the FIFO to fill at 4 kHz.

| ODR | Effective Rate (sensor clock runs ~0.5% fast) | Sample Loss |
|-----|---------------|-------------|
| 1 kHz | ~1005 Hz | 0% |
| 2 kHz | ~2010 Hz | 0% |
| 4 kHz | ~4020 Hz | 0% |

The earlier ~30% loss at 4 kHz (~2830 Hz) was not USB latency; it was the RX offset described under Transport Quirks.

## SPI Command Format

The ADXL355 uses `(register_address << 1) | RNW` as the command byte:
- Read: `(addr << 1) | 0x01`
- Write: `(addr << 1) | 0x00`

This was not obvious. Many SPI sensors use `addr | 0x80` for reads (bit 7 = R/W). The ADXL355 puts the address in bits [7:1] and R/W in bit 0. Early code used the wrong format, which produced no errors but returned garbage data.

## FT232H MPSSE Transport Quirks

### Never call SPI_ReadWrite

This libMPSSE build answers `SPI_ReadWrite` with the MPSSE bad-command reply (`0xFA 0x00`). Those 2 bytes stay in the FTDI RX queue, so every later `SPI_Read` returns the previous transaction's last 2 bytes first.

Earlier code primed the bus with `SPI_ReadWrite` after init and then compensated by reading `count + 2` bytes and dropping the first 2 (documented as a "2-byte pipeline delay"). That workaround clocked 2 extra bytes out of `FIFO_DATA` on every read, eating into the next FIFO entry and losing ~1 sample per poll.

Verified with `FT_GetQueueStatus`: right after `SPI_InitChannel` the RX queue is empty and exact-length split reads return `DEVID_AD/DEVID_MST/PARTID = AD 1D ED` on the first transaction; each `SPI_ReadWrite` leaves 2 bytes behind. libMPSSE already resets and purges the device in `SPI_InitChannel`, so no priming is needed.

All reads use the split approach with exact lengths: `SPI_Write` (cmd byte, CS stays asserted) followed by `SPI_Read` (`count` bytes, CS deasserts).

## Write Path

`SPI_Write` with both `CS_ENABLE` and `CS_DISABLE` flags works normally. The command byte + data byte are sent in a single call.
