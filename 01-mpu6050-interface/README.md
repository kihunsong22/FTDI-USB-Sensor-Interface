# FT232H Sensor Interface - MPU6050 Driver

A Rust library and executable for interfacing with the MPU6050 6-axis motion sensor (accelerometer + gyroscope) via the FTDI FT232H USB-to-I2C bridge.

## Overview

This project provides:
- **Library**: Reusable MPU6050 driver for integration with other applications
- **Executable**: Standalone sensor reader for continuous data acquisition and display

The implementation uses FTDI's libMPSSE library for I2C communication through FFI (Foreign Function Interface).

## Hardware Requirements

- **FT232H Module**: FTDI USB-to-I2C/SPI/GPIO converter
- **MPU6050 Sensor**: 6-axis motion sensor (I2C interface)
- **Connections**:
  - FT232H D0 (SCL) → MPU6050 SCL
  - FT232H D1/D2 (SDA) → MPU6050 SDA
  - 3.3V power and ground connections
  - 4.7kΩ pull-up resistors on SDA and SCL lines (often built into modules)

## Software Requirements

- **Rust**: Edition 2021 or later
- **FTDI Drivers**: D2XX drivers must be installed
- **libMPSSE**: Included in project directory
- **Windows**: x64 platform (configured for MSVC toolchain)

## Project Structure

```
FT232-Sensor-Interface/
├── src/
│   ├── lib.rs          # Library entry point
│   ├── main.rs         # Executable for sensor reading
│   ├── ffi.rs          # FFI bindings to libMPSSE
│   ├── error.rs        # Error types
│   └── mpu6050.rs      # MPU6050 driver implementation
├── FTDI MPSSE/         # FTDI libMPSSE library and headers
├── FTDI-D2XX-Drivers-Win-2.12.36.20U/  # D2XX drivers
├── Cargo.toml          # Project manifest
├── build.rs            # Build script for DLL linking
└── README.md           # This file
```

## Building

Build the project using Cargo:

```bash
cargo build --release
```

This will:
1. Link against the libMPSSE and FTD2XX DLLs
2. Compile both the library and executable
3. Create the binary at `target/release/mpu6050-reader.exe`

## Running the Executable

Connect your hardware and run:

```bash
cargo run --release
```

Expected output:
```
MPU6050 Sensor Reader
====================
Initializing FT232H I2C interface...
Sensor initialized successfully!
Press Ctrl+C to exit

Timestamp            | Accelerometer (g)              | Gyroscope (°/s)
-------------------------------------------------------------------------------------
    0.00s (     0) | X:   0.12 Y:  -0.05 Z:   1.01 | X:   0.23 Y:  -0.15 Z:   0.08
```

## Using as a Library

This library is designed for easy integration with other packages for vibration analysis, visualization, logging, and real-time data processing.

### Adding as Dependency

Add to your project's `Cargo.toml`:

```toml
[dependencies]
ft232-sensor-interface = { path = "../01-mpu6050-interface" }
```

### Simple Reading

```rust
use ft232_sensor_interface::Mpu6050;

let mut sensor = Mpu6050::new(0)?;
let data = sensor.read_all()?;

println!("Accel X: {:.2}g", data.accel_x_g());
println!("Gyro Z: {:.2}°/s", data.gyro_z_dps());
```

### Streaming API (1-1000 Hz)

For continuous data acquisition with configurable sample rates:

```rust
use ft232_sensor_interface::{Mpu6050, StreamControl};

let mut sensor = Mpu6050::new(0)?;

// Stream at 500 Hz for vibration analysis
sensor.stream(500, |data| {
    let (ax, ay, az) = data.accel_to_g();
    let magnitude = (ax*ax + ay*ay + az*az).sqrt();

    // Process data in real-time
    if magnitude > 3.0 {
        println!("High vibration: {:.2}g", magnitude);
        StreamControl::Break  // Stop streaming
    } else {
        StreamControl::Continue
    }
})?;
```

### Batch Collection

For offline analysis (FFT, statistics, etc.):

```rust
// Collect 2048 samples at 1000 Hz
let samples = sensor.collect_samples(1000, 2048)?;

// Perform frequency analysis, vibration detection, etc.
for sample in &samples {
    // Process collected data...
}
```

### Time-Based Acquisition

```rust
use std::time::Duration;

// Monitor for 10 seconds at 200 Hz
sensor.stream_for(200, Duration::from_secs(10), |data| {
    // Log or process each sample
})?;
```

### Complete Examples

See `examples/` directory for full working examples:
- `vibration_analysis.rs` - Real-time monitoring and batch analysis
- `data_logging.rs` - CSV logging with timestamps

Run with:
```bash
cargo run --example vibration_analysis
cargo run --example data_logging
```

For detailed library usage guide, see [Docs/LIBRARY_USAGE.md](Docs/LIBRARY_USAGE.md).

## API Reference

### Main Types

- **`Mpu6050`**: Main sensor interface
  - `new(channel_index: u32) -> Result<Self>`: Initialize sensor
  - `read_all() -> Result<SensorData>`: Single sensor reading
  - `stream(rate_hz, callback) -> Result<u64>`: Continuous streaming with callback
  - `stream_for(rate_hz, duration, callback) -> Result<u64>`: Time-limited streaming
  - `collect_samples(rate_hz, count) -> Result<Vec<SensorData>>`: Batch collection
  - `read_accel() -> Result<(i16, i16, i16)>`: Read accelerometer (raw)
  - `read_gyro() -> Result<(i16, i16, i16)>`: Read gyroscope (raw)

- **`SensorData`**: Sensor reading structure
  - Raw fields: `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z` (i16)
  - `accel_to_g() -> (f32, f32, f32)`: All accelerometer axes in g
  - `gyro_to_dps() -> (f32, f32, f32)`: All gyroscope axes in °/s
  - `accel_x_g()`, `accel_y_g()`, `accel_z_g()`: Individual accel axes in g
  - `gyro_x_dps()`, `gyro_y_dps()`, `gyro_z_dps()`: Individual gyro axes in °/s

- **`StreamControl`**: Control streaming operations
  - `Continue`: Keep streaming
  - `Break`: Stop streaming

- **`Mpu6050Error`**: Error type for all operations

### Sample Rates

Supported range: **1-1000 Hz** (configurable)
- **Actual maximum: ~100 Hz** (limited by I2C/USB overhead with FT232H + libMPSSE)
- **Recommended for motion/vibration: 50-100 Hz**
- **Recommended for general monitoring: 10-50 Hz**

**Note:** Library is optimized with single 14-byte I2C transaction for all sensor data (accel + gyro).
Requesting rates >100 Hz will maintain timing but won't increase actual sample throughput.

## Troubleshooting

### "No I2C channels found"
- Ensure FT232H is connected via USB
- Verify FTDI drivers are installed
- Check that no other application is using the device

### "Invalid WHO_AM_I response"
- Check physical I2C connections (SDA, SCL)
- Verify power supply to MPU6050 (3.3V)
- Ensure pull-up resistors are present on I2C lines
- Check I2C address (default is 0x68)

### Build errors
- Ensure DLL files exist in the specified paths
- Verify Rust toolchain is x86_64-pc-windows-msvc
- Check that libmpsse.dll and ftd2xx.dll are in the correct directories

## Technical Details

- **I2C Clock Rate**: 400 kHz (Fast Mode)
- **Accelerometer Range**: ±2g (16384 LSB/g)
- **Gyroscope Range**: ±250°/s (131 LSB/°/s)
- **Sample Rates**: 1-1000 Hz configurable, ~100 Hz actual max
  - Limited by I2C/USB overhead (FT232H via libMPSSE)
  - Single optimized 14-byte I2C read per sample (accel + temp + gyro)
- **Data Format**: 16-bit big-endian values
- **Platform**: Windows 32-bit (i686-pc-windows-msvc)

## References

- [MPU6050 Register Map](./MPU-6000%20and%20MPU-6050%20Register%20Map%20and%20Descriptions%20Revision%204.2.pdf)
- [MPU6050 Product Specification](./MPU-6000%20and%20MPU-6050%20Product%20Specification%20Revision%203.4.pdf)
- [libMPSSE I2C User Guide](./FTDI%20MPSSE/User%20Guide%20For%20libMPSSE%20–%20I2C.pdf)
- [D2XX Programmer's Guide](./D2XX%20Programmer's%20Guide.pdf)

## License

This project interfaces with proprietary FTDI libraries. Refer to FTDI's licensing terms for libMPSSE and D2XX drivers.
