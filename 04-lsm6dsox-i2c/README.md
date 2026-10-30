# LSM6DSOX over FT232H I2C

Bring-up tool for the ST LSM6DSOX 6-axis IMU (Adafruit breakout) on an Adafruit FT232H (USB-C) through the STEMMA QT cable, using libMPSSE I2C. Datasheet and app notes: `../References/LSM6DSOX/`.

## Wiring

- STEMMA QT cable from the FT232H to the sensor board (3.3 V, GND, SCL = D0, SDA = D1/D2).
- **FT232H "I2C Mode" switch on** (joins D1 and D2). With it off every address NACKs. SPI sensors (ADXL355) need it off, so one FT232H runs either I2C or SPI.
- Sensor address 0x6A (SA0 low, Adafruit default) or 0x6B.

## Run

```powershell
cargo build --release
.\target\x86_64-pc-windows-msvc\release\lsm6dsox.exe <command> [--clock 100|400|1000] [--channel N]
```

| Command | Checks |
|---|---|
| `scan` | FT232H channels, I2C addresses that answer |
| `pins` | D1-D2 joined (I2C switch), pull-ups on SCL/SDA |
| `info` | WHO_AM_I, INTERNAL_FREQ_FINE and trimmed ODRs, control registers |
| `selftest` | Built-in self-test against datasheet limits (keep still) |
| `read [--odr 104]` | Polled output registers, mean and noise per axis |
| `fifo [--odr 833] [--no-gyro] [--secs 3]` | FIFO streaming: lost samples (TAG_CNT slots, overrun latch), rate fit, bus throughput |

## Results (2026-10-30, one board, sensor still on a desk)

| Mode | 400 kHz | 1 MHz |
|---|---|---|
| Accel + gyro 833 / 1667 Hz | no loss | no loss |
| Accel + gyro 3333 Hz | overruns (39 kB/s max) | no loss (47 kB/s) |
| Accel + gyro 6667 Hz | - | overruns (82 kB/s max) |
| Accel only 3333 Hz | no loss | - |
| Accel only 6667 Hz | overruns | no loss (48 kB/s) |

- Self-test passes (accel 222~430 mg, gyro 373~426 dps).
- **Sample rate depends on the gyro**: gyro on, +0.08 % from the INTERNAL_FREQ_FINE rate (834.07 Hz at 833); gyro off, +2.35 % (852.93 Hz). Measure the rate, do not assume it.
- The first ~100 ms after power-up carry gyro values up to 60 dps; drop them.
- FIFO holds 512 words (3 KB, no compression): about 77 ms of accel + gyro at 3333 Hz before overrun.
- Polled register reads manage ~100~400 Hz depending on host load; use the FIFO for anything faster.
- 1 MHz (Fm+) is characterization-only in the datasheet; it ran clean here with the Adafruit pull-ups and a short cable.
