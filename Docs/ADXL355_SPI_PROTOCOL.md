# ADXL355 SPI Protocol — FT232H MPSSE

## Status

Working. Sensor init, register R/W, FIFO streaming, and HDF5 collection all functional. Validated with 7-point data integrity check (`validate-data` binary).

- 1kHz ODR: ~993 Hz effective (lossless)
- 4kHz ODR: ~2830 Hz effective (FIFO overflow, USB bandwidth limited)

## Current Configuration

- SPI clock: 1 MHz
- MPSSE latency timer: 1 ms
- SPI mode: 0 (CPOL=0, CPHA=0)
- CS: DBUS3, active low
- Sensor range: ±2g (scale factor 256,000 LSB/g)
- FIFO: 96 entries = 32 complete XYZ samples (3 entries per sample, 3 bytes per entry)
- Temperature: read separately via register, not available in FIFO

### Performance

Each FIFO poll requires 4 USB round-trips (2 split Write+Read transactions): one to read `FIFO_ENTRIES`, one to read the data. USB transaction latency is the bottleneck, not byte throughput.

| ODR | Effective Rate | Sample Loss | Notes |
|-----|---------------|-------------|-------|
| 1 kHz | ~993 Hz | ~0% | USB keeps up |
| 4 kHz | ~2830 Hz | ~30% | FIFO overflows between polls |

## SPI Command Format

The ADXL355 uses `(register_address << 1) | RNW` as the command byte:
- Read: `(addr << 1) | 0x01`
- Write: `(addr << 1) | 0x00`

This was not obvious. Many SPI sensors use `addr | 0x80` for reads (bit 7 = R/W). The ADXL355 puts the address in bits [7:1] and R/W in bit 0. Early code used the wrong format, which produced no errors but returned garbage data.

## FT232H MPSSE Transport Quirks

Three non-obvious behaviors were discovered empirically:

### 1. Bus priming required after channel init

The first 2-3 SPI transactions after `SPI_InitChannel` return invalid data (0xFF or noise). A priming sequence is required: one `SPI_ReadWrite` followed by a split `SPI_Write` + `SPI_Read`.

### 2. SPI_ReadWrite (full-duplex) is unreliable for reads

`SPI_ReadWrite` only returns valid data once after priming, then reverts to garbage. All production reads use the split approach: `SPI_Write` (cmd byte, CS stays asserted) followed by `SPI_Read` (data bytes, CS deasserts).

### 3. 2-byte pipeline delay on SPI_Read

The first 2 bytes returned by `SPI_Read` are stale (from a previous transaction's auto-increment state). Fresh register data begins at byte index 2. All reads request `count + 2` bytes and discard the first 2.

## Write Path

`SPI_Write` with both `CS_ENABLE` and `CS_DISABLE` flags works normally — no pipeline issues. The command byte + data byte are sent in a single call.
