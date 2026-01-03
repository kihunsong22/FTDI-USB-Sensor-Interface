# MPU6050 Interface via FT232H

Rust driver for MPU6050 6-axis sensor (accelerometer + gyroscope) using FTDI FT232H I2C bridge.

## What's Included

- **Library**: MPU6050 driver with streaming API (1-1000 Hz)
- **mpu6050-reader**: Real-time CLI display with bar graphs
- **collector**: Acquire data to HDF5 (polling ~100Hz or FIFO ~850Hz)
- **sensor-gui**: Interactive GUI with time-series plots and FFT (requires `gui` feature)
- **analyzer**: FFT, statistics, vibration analysis (requires `analysis` feature)

## Quick Start

```bash
# Build (see ../INSTALLATION.md for prerequisites)
cargo build --release

# Live sensor display (default binary)
cargo run --release

# Collect data to HDF5 (FIFO mode, 60 seconds)
cargo run --release --bin collector -- --output data.h5 --mode fifo --duration 60

# Analyze recorded data
cargo run --release --features analysis --bin analyzer -- --input data.h5 --all

# GUI visualizer (live or playback)
cargo run --release --features gui --bin sensor-gui
```

## Programs

| Program | Command | Purpose |
|---------|---------|---------|
| **mpu6050-reader** | `cargo run --release` | Real-time CLI display with bar graphs |
| **collector** | `cargo run --release --bin collector -- [OPTIONS]` | Record sensor data to HDF5 |
| **sensor-gui** | `cargo run --release --features gui --bin sensor-gui` | GUI with plots and FFT |
| **analyzer** | `cargo run --release --features analysis --bin analyzer -- [OPTIONS]` | Post-processing analysis |

### Collector Options

```
--output <FILE>     Output HDF5 file (default: sensor_data.h5)
--mode <MODE>       Collection mode: polling or fifo (default: polling)
--rate <HZ>         Target sample rate (polling: 1-100, fifo: 4-1000)
--duration <SECS>   Duration in seconds (optional, Ctrl+C to stop)
```

### Analyzer Options

```
--input <FILE>      Input HDF5 file (required)
--start <SECS>      Start time for analysis window
--end <SECS>        End time for analysis window
--statistics        Compute statistical metrics
--fft               Perform FFT frequency analysis
--vibration         Compute vibration metrics (RMS, velocity, displacement)
--all               Run all analyses
--output <FILE>     Output file (default: stdout)
```

## Library Usage

```rust
use ft232_sensor_interface::{Mpu6050, StreamControl};

let mut sensor = Mpu6050::new(0)?;

// Single read
let data = sensor.read_all()?;
println!("Accel X: {:.2}g", data.accel_x_g());

// Stream at 100 Hz with callback
sensor.stream(100, |data| {
    let (ax, ay, az) = data.accel_to_g();
    if (ax*ax + ay*ay + az*az).sqrt() > 3.0 {
        StreamControl::Break  // Stop on high vibration
    } else {
        StreamControl::Continue
    }
})?;

// FIFO mode for higher sample rates (~850 Hz)
sensor.enable_fifo(1000)?;
sensor.stream_fifo(20, |batch| {
    println!("Got {} samples", batch.len());
    StreamControl::Continue
})?;
sensor.disable_fifo()?;
```

## Technical Specs

- **I2C**: 400 kHz Fast Mode
- **Accelerometer**: ±2g (16384 LSB/g)
- **Gyroscope**: ±250°/s (131 LSB/°/s)
- **Sample Rates**: Polling ~100Hz max, FIFO ~850Hz max
- **Platform**: Windows x64 (requires FTDI D2XX drivers)
