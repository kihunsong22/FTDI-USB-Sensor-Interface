# Library Usage Guide

This document explains how to use `ft232-sensor-interface` as a library in other Rust projects for vibration analysis, visualization, logging, and other sensor data processing tasks.

## Adding as a Dependency

### Option 1: Path Dependency (Development)

In your project's `Cargo.toml`:

```toml
[dependencies]
ft232-sensor-interface = { path = "../01-mpu6050-interface" }
```

### Option 2: Git Dependency (If published to a repository)

```toml
[dependencies]
ft232-sensor-interface = { git = "https://github.com/yourorg/ft232-sensor-interface" }
```

## Quick Start

### Simple Data Reading

```rust
use ft232_sensor_interface::Mpu6050;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut sensor = Mpu6050::new(0)?;
    let data = sensor.read_all()?;

    println!("Accel: ({:.2}, {:.2}, {:.2})g",
        data.accel_x_g(), data.accel_y_g(), data.accel_z_g());

    Ok(())
}
```

## API Overview

### Core Methods

| Method | Description | Use Case |
|--------|-------------|----------|
| `new(channel)` | Initialize sensor | Setup |
| `read_all()` | Single reading | Manual polling |
| `stream(rate_hz, callback)` | Continuous streaming | Real-time processing |
| `stream_for(rate_hz, duration, callback)` | Time-limited streaming | Timed acquisition |
| `collect_samples(rate_hz, count)` | Batch collection | Offline analysis |

### Data Access

`SensorData` provides multiple access patterns:

```rust
let data = sensor.read_all()?;

// Tuple access (all axes at once)
let (ax, ay, az) = data.accel_to_g();
let (gx, gy, gz) = data.gyro_to_dps();

// Individual axis access
let x_accel = data.accel_x_g();
let z_gyro = data.gyro_z_dps();

// Raw values (i16)
let raw_x = data.accel_x;
```

## Sample Rate Configuration

The library supports sample rates from **1 Hz to 1000 Hz**:

```rust
// Low rate for monitoring
sensor.stream(10, |data| { /* ... */ })?;

// Medium rate for motion tracking
sensor.stream(100, |data| { /* ... */ })?;
```

**Practical Limits (FT232H via libMPSSE I2C):**
- **~100 Hz**: Maximum achievable rate with current I2C implementation
- **50-100 Hz**: Recommended for motion tracking and vibration monitoring
- **10-50 Hz**: Good for general monitoring
- **1-10 Hz**: Sufficient for slow movement tracking

**Note:** Actual achievable rate is limited by I2C communication overhead (MPSSE library + USB latency).
The library reads all sensor data (14 bytes: accel + temp + gyro) in a single optimized I2C transaction.
Requesting rates >100 Hz will not increase actual sample rate but will maintain timing for other operations.

## Usage Patterns

### 1. Vibration Analysis

Collect high-frequency samples for FFT analysis:

```rust
use ft232_sensor_interface::Mpu6050;

let mut sensor = Mpu6050::new(0)?;

// Collect 4096 samples at 1000 Hz
let samples = sensor.collect_samples(1000, 4096)?;

// Extract acceleration data for FFT
let accel_x: Vec<f32> = samples.iter()
    .map(|s| s.accel_x_g())
    .collect();

// Perform FFT analysis (using external FFT library)
// let spectrum = fft::process(&accel_x);
```

### 2. Real-Time Visualization

Stream data to a GUI or terminal display:

```rust
use ft232_sensor_interface::{Mpu6050, StreamControl};
use std::sync::mpsc;

let (tx, rx) = mpsc::channel();
let mut sensor = Mpu6050::new(0)?;

// Spawn visualization thread
std::thread::spawn(move || {
    for data in rx {
        // Update visualization with data
    }
});

// Stream data to visualizer
sensor.stream(100, |data| {
    tx.send(data).ok();
    StreamControl::Continue
})?;
```

### 3. Data Logging

Log sensor data with timestamps:

```rust
use ft232_sensor_interface::Mpu6050;
use std::time::Instant;
use std::fs::File;
use std::io::Write;

let mut sensor = Mpu6050::new(0)?;
let mut log = File::create("data.csv")?;
let start = Instant::now();

sensor.stream_for(200, Duration::from_secs(60), |data| {
    let t = start.elapsed().as_secs_f64();
    writeln!(log, "{:.3},{:.4},{:.4},{:.4}",
        t, data.accel_x_g(), data.accel_y_g(), data.accel_z_g()).ok();
})?;
```

### 4. Event Detection

Monitor for specific conditions:

```rust
use ft232_sensor_interface::{Mpu6050, StreamControl};

let mut sensor = Mpu6050::new(0)?;

// Detect sudden acceleration (impact/shock)
sensor.stream(500, |data| {
    let (ax, ay, az) = data.accel_to_g();
    let magnitude = (ax*ax + ay*ay + az*az).sqrt();

    if magnitude > 3.0 {
        println!("Impact detected: {:.2}g at {:?}", magnitude, Instant::now());
        // Trigger action: save data, send alert, etc.
    }

    StreamControl::Continue
})?;
```

### 5. Statistical Analysis

Compute running statistics:

```rust
use ft232_sensor_interface::Mpu6050;

let mut sensor = Mpu6050::new(0)?;
let mut min_x = f32::MAX;
let mut max_x = f32::MIN;
let mut sum_x = 0.0f32;
let mut count = 0u64;

sensor.stream_for(100, Duration::from_secs(10), |data| {
    let x = data.accel_x_g();
    min_x = min_x.min(x);
    max_x = max_x.max(x);
    sum_x += x;
    count += 1;
})?;

println!("X-axis statistics over 10 seconds:");
println!("  Min: {:.3}g", min_x);
println!("  Max: {:.3}g", max_x);
println!("  Avg: {:.3}g", sum_x / count as f32);
```

## Multi-Package Project Structure

For projects with multiple packages (visualization, analysis, logging):

```
FT232-Sensor-Interface/
├── 01-mpu6050-interface/        # This library
├── 02-vibration-analyzer/       # Analysis package
│   └── Cargo.toml
│       [dependencies]
│       ft232-sensor-interface = { path = "../01-mpu6050-interface" }
├── 03-sensor-visualizer/        # Visualization package
│   └── Cargo.toml
│       [dependencies]
│       ft232-sensor-interface = { path = "../01-mpu6050-interface" }
└── 04-data-logger/             # Logging package
    └── Cargo.toml
        [dependencies]
        ft232-sensor-interface = { path = "../01-mpu6050-interface" }
```

## Thread Safety

The `Mpu6050` struct is **not thread-safe** (uses `*mut` FFI handle). For multi-threaded applications:

```rust
use std::sync::{Arc, Mutex};
use std::thread;

let sensor = Arc::new(Mutex::new(Mpu6050::new(0)?));

// Spawn reader thread
let sensor_clone = sensor.clone();
let reader = thread::spawn(move || {
    let mut sensor = sensor_clone.lock().unwrap();
    sensor.stream(100, |data| {
        // Process data
        StreamControl::Continue
    })
});

// Main thread can do other work
reader.join().unwrap()?;
```

## Performance Tips

1. **Use batch collection** (`collect_samples`) for offline analysis - more efficient than individual reads
2. **Realistic sample rates**: Don't request higher rates than needed (saves CPU and reduces timing jitter)
3. **Minimize callback overhead**: Keep streaming callbacks fast; offload heavy processing to separate threads
4. **Pre-allocate buffers**: When collecting data, pre-allocate vectors with `Vec::with_capacity()`

## Error Handling

All methods return `Result<T, Mpu6050Error>`:

```rust
use ft232_sensor_interface::Mpu6050Error;

match sensor.read_all() {
    Ok(data) => { /* process data */ },
    Err(Mpu6050Error::NoChannelsFound) => {
        eprintln!("FT232H device not connected");
    },
    Err(Mpu6050Error::InvalidDeviceId(id)) => {
        eprintln!("Wrong sensor detected: 0x{:02X}", id);
    },
    Err(e) => {
        eprintln!("Communication error: {}", e);
    }
}
```

## Examples

See the `examples/` directory for complete working examples:

- `vibration_analysis.rs` - Vibration monitoring and analysis
- `data_logging.rs` - CSV data logging with timestamps

Run examples with:
```bash
cargo run --example vibration_analysis
cargo run --example data_logging
```

## API Documentation

Generate full API documentation:

```bash
cd 01-mpu6050-interface
cargo doc --open
```

## Troubleshooting

### "No I2C channels found"
- Check FT232H is connected via USB
- Verify FTDI drivers are installed
- Ensure no other application is using the device

### Lower than expected sample rate
- I2C communication overhead limits maximum rate
- Try reducing target rate or optimize callback processing
- Check system load (other processes consuming CPU)

### Missing DLLs at runtime
- The build script auto-copies DLLs to target directory
- If issues persist, manually copy `libmpsse.dll` and `FTD2XX.dll` to executable directory
