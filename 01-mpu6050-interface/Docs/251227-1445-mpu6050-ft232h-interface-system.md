# MPU6050-FT232H Interface System

**Date**: December 27, 2025
**Project**: FT232H Sensor Interface
**Purpose**: USB-to-I2C bridge for MPU6050 motion sensor data acquisition

---

## System Overview

This system provides a software interface between a host computer and an MPU6050 6-axis motion sensor through FTDI's FT232H USB-to-I2C bridge. The implementation is written in Rust and uses Foreign Function Interface (FFI) to leverage FTDI's proprietary libMPSSE library for I2C communication.

### Key Components

1. **Hardware Layer**
   - FT232H: USB-to-MPSSE converter (Multi-Protocol Synchronous Serial Engine)
   - MPU6050: 3-axis accelerometer + 3-axis gyroscope sensor
   - I2C physical connection at 400 kHz (Fast Mode)

2. **Software Architecture**
   - FFI bindings to Windows DLLs (libmpsse.dll, FTD2XX.dll)
   - MPU6050 driver abstraction
   - Dual-target build: library + standalone executable

3. **Data Flow**
   ```
   MPU6050 (I2C device 0x68)
     ↓ I2C @ 400kHz
   FT232H (USB bridge)
     ↓ USB 2.0
   Host PC (Windows)
     ↓ libMPSSE DLL
   Rust FFI bindings
     ↓
   MPU6050 driver
     ↓
   Application (library consumer or standalone executable)
   ```

---

## Architecture

### Module Structure

**FFI Layer** (`src/ffi.rs`)
- Defines C-compatible types matching Windows and FTDI APIs
- Declares external functions from libmpsse.dll
- Provides type safety around raw pointers and status codes
- Reference: `FTDI MPSSE/include/libmpsse_i2c.h`

**Error Handling** (`src/error.rs`)
- Custom error type (`Mpu6050Error`) wrapping FTDI status codes
- Converts FT_STATUS to Rust Result types
- Provides descriptive error messages for debugging

**MPU6050 Driver** (`src/mpu6050.rs`)
- Encapsulates I2C communication with MPU6050
- Implements sensor initialization sequence:
  1. Wake sensor from sleep mode (PWR_MGMT_1 = 0x00)
  2. Verify device identity (WHO_AM_I register = 0x68)
  3. Configure accelerometer range (±2g default)
  4. Configure gyroscope range (±250°/s default)
- Provides high-level read functions for sensor data
- Reference: `MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2.pdf`

**Library Interface** (`src/lib.rs`)
- Public API exports: `Mpu6050`, `SensorData`, `Mpu6050Error`
- Designed for integration with external applications

**Standalone Executable** (`src/main.rs`)
- Continuous sensor reading at ~100 Hz
- Real-time console display of accelerometer and gyroscope data
- Proper error handling and device cleanup

### Build System

**Cargo Configuration** (`Cargo.toml`)
- Dual compilation targets: library (`[lib]`) and binary (`[[bin]]`)
- Dependency: `thiserror` for error type derivation

**Build Script** (`build.rs`)
- Locates and links libmpsse.dll at compile time
- Sets library search paths for Windows MSVC linker
- Note: FTD2XX.dll dependency is resolved at runtime by libmpsse.dll

**Platform Configuration** (`.cargo/config.toml`)
- Target: x86_64-pc-windows-msvc
- Windows-specific build settings

---

## Technical Decisions

### Why MPSSE Instead of D2XX?

**MPSSE (Multi-Protocol Synchronous Serial Engine)**
- High-level library with built-in I2C protocol implementation
- Functions: `I2C_DeviceRead()`, `I2C_DeviceWrite()`, `I2C_InitChannel()`
- Handles timing, start/stop conditions, ACK/NACK automatically
- Appropriate for standard I2C devices like MPU6050

**D2XX (Direct Driver)**
- Low-level access requiring manual MPSSE command construction
- Only necessary for custom protocols or non-standard timing
- Not required for this use case

**Decision**: Use MPSSE for simplicity and reliability with standard I2C.

### Why Rust FFI Instead of Pure Rust?

The FT232H requires FTDI's proprietary drivers (no open-source alternatives support MPSSE mode). FFI allows us to:
- Leverage battle-tested FTDI libraries
- Maintain type safety through Rust wrappers
- Provide a safe, idiomatic Rust API to consumers

### I2C Communication Pattern

Based on FTDI sample code and MPU6050 requirements:

**Register Read Sequence**:
1. Write register address with START condition
2. Read data with START (repeated start), STOP, and NACK_LAST_BYTE
3. Transfer options: `START_BIT | STOP_BIT | NACK_LAST_BYTE | FAST_TRANSFER_BYTES`

**Register Write Sequence**:
1. Write [register_address, data] with START and STOP conditions
2. Transfer options: `START_BIT | STOP_BIT | FAST_TRANSFER_BYTES`

Reference: `FTDI MPSSE/samples/BME280-I2C/bme280.c` and `FTDI MPSSE/samples/I2C_EEPROM/main.c`

### Data Conversion

**Raw to Physical Units**:
- Accelerometer: 16-bit signed → divide by 16384 → g (±2g range)
- Gyroscope: 16-bit signed → divide by 131 → °/s (±250°/s range)
- Byte order: Big-endian (MSB first)

---

## Integration Points

### As a Library

External projects can depend on this crate and use:

```rust
use ft232_sensor_interface::{Mpu6050, SensorData};
```

**API Surface**:
- `Mpu6050::new(channel_index)` → Initialize sensor
- `read_accel()` → (i16, i16, i16) raw values
- `read_gyro()` → (i16, i16, i16) raw values
- `read_all()` → SensorData struct
- `SensorData::accel_to_g()` → (f32, f32, f32) in g
- `SensorData::gyro_to_dps()` → (f32, f32, f32) in °/s

### As a Standalone Tool

Executable: `mpu6050-reader.exe`

Provides immediate sensor data visualization for:
- Hardware validation
- Sensor calibration
- Quick testing and debugging

---

## File Reference Map

| Purpose | Files |
|---------|-------|
| FFI Bindings | `src/ffi.rs`, `FTDI MPSSE/include/libmpsse_i2c.h` |
| Sensor Driver | `src/mpu6050.rs`, MPU6050 register map PDF |
| Error Handling | `src/error.rs` |
| Public API | `src/lib.rs` |
| Executable | `src/main.rs` |
| Build Configuration | `Cargo.toml`, `build.rs`, `.cargo/config.toml` |
| Libraries | `FTDI MPSSE/build/x64/DLL/libmpsse.dll` |
| Runtime Dependencies | `FTDI-D2XX-Drivers-Win-2.12.36.20U/x86/FTD2XX.dll` |
| Documentation | `README.md`, `Docs/` |

---

## System Requirements

**Hardware**:
- FT232H module (USB 2.0)
- MPU6050 sensor module
- I2C pull-up resistors (typically 4.7kΩ, often integrated in modules)

**Software**:
- Windows x64
- Rust toolchain (MSVC)
- FTDI D2XX drivers installed
- libMPSSE and FTD2XX DLLs accessible at runtime

**Connections**:
- FT232H D0 (SCL) ↔ MPU6050 SCL
- FT232H D1/D2 (SDA) ↔ MPU6050 SDA
- Common ground
- 3.3V power to MPU6050

---

## Future Development

This library is designed to support future integration with a visualization package. The clean API separation allows:

1. **Data Streaming**: Continuous sensor data can be consumed by visualization tools
2. **Multi-Sensor Support**: Architecture supports adding more I2C sensors
3. **Configurability**: Sensor ranges and sample rates can be made adjustable
4. **Data Logging**: Raw or processed data can be logged for post-processing

The library provides the foundation for motion tracking, orientation estimation, gesture recognition, or other IMU-based applications.

---

## References

- **MPU6050 Register Map**: `MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2.pdf`
- **MPU6050 Specification**: `MPU-6000 and MPU-6050 Product Specification Revision 3.4.pdf`
- **libMPSSE I2C Guide**: `FTDI MPSSE/User Guide For libMPSSE – I2C.pdf`
- **D2XX Programming**: `D2XX Programmer's Guide.pdf`
- **Sample Code**: `FTDI MPSSE/samples/I2C_EEPROM/`, `FTDI MPSSE/samples/BME280-I2C/`
