# API Quick Reference

Quick reference for using ft232-sensor-interface in your packages.

## Installation

```toml
[dependencies]
ft232-sensor-interface = { path = "../01-mpu6050-interface" }
```

## Basic Usage

```rust
use ft232_sensor_interface::Mpu6050;

// Initialize
let mut sensor = Mpu6050::new(0)?;

// Single read
let data = sensor.read_all()?;
println!("X: {:.2}g", data.accel_x_g());
```

## Streaming API

### Method Comparison

| Method | Use Case | Returns |
|--------|----------|---------|
| `read_all()` | Manual polling | Single `SensorData` |
| `stream(rate, callback)` | Continuous with control | Sample count |
| `stream_for(rate, duration, callback)` | Fixed duration | Sample count |
| `collect_samples(rate, count)` | Batch for analysis | `Vec<SensorData>` |

### stream() - Continuous with Control

```rust
use ft232_sensor_interface::{Mpu6050, StreamControl};

sensor.stream(500, |data| {
    // Process each sample
    if some_condition {
        StreamControl::Break    // Stop streaming
    } else {
        StreamControl::Continue // Keep going
    }
})?;
```

**Use for:**
- Event detection
- Real-time monitoring
- Conditional stopping

### stream_for() - Time-Limited

```rust
use std::time::Duration;

sensor.stream_for(200, Duration::from_secs(10), |data| {
    // Runs for exactly 10 seconds
})?;
```

**Use for:**
- Fixed-duration captures
- Periodic monitoring
- Time-based logging

### collect_samples() - Batch Collection

```rust
let samples = sensor.collect_samples(1000, 2048)?;

// Now process all samples
for sample in &samples {
    // FFT, statistics, etc.
}
```

**Use for:**
- FFT analysis
- Statistical processing
- Offline analysis

## Data Access

### Tuple Access (All Axes)

```rust
let (ax, ay, az) = data.accel_to_g();
let (gx, gy, gz) = data.gyro_to_dps();
```

### Individual Access

```rust
let x = data.accel_x_g();     // Single axis
let z = data.gyro_z_dps();
```

### Raw Values

```rust
let raw_x: i16 = data.accel_x;  // Raw ADC value
```

## Sample Rate Guidelines

| Rate | Use Case | Notes |
|------|----------|-------|
| 1-10 Hz | Slow monitoring | Very low CPU |
| 10-50 Hz | General monitoring | Good for most tasks |
| 50-100 Hz | Motion/vibration | Maximum useful rate |
| >100 Hz | Not achievable | Limited by I2C overhead |

**Practical limit:** ~100 Hz due to I2C/USB overhead (FT232H + libMPSSE)

**Optimization:** Library uses single 14-byte I2C read for all sensor data (accel + gyro) instead of separate transactions.

## Common Patterns

### Vibration Monitoring

```rust
sensor.stream(500, |data| {
    let (ax, ay, az) = data.accel_to_g();
    let magnitude = (ax*ax + ay*ay + az*az).sqrt();

    if magnitude > 3.0 {
        log_event("High vibration", magnitude);
    }
    StreamControl::Continue
})?;
```

### Data Logging

```rust
let mut file = File::create("log.csv")?;
sensor.stream_for(100, Duration::from_secs(60), |data| {
    writeln!(file, "{},{},{}",
        data.accel_x_g(),
        data.accel_y_g(),
        data.accel_z_g()).ok();
})?;
```

### FFT Analysis

```rust
let samples = sensor.collect_samples(1000, 2048)?;
let accel_x: Vec<f32> = samples.iter()
    .map(|s| s.accel_x_g())
    .collect();

// Perform FFT on accel_x
```

### Peak Detection

```rust
let mut peak = 0.0f32;
sensor.stream_for(500, Duration::from_secs(5), |data| {
    let val = data.accel_x_g().abs();
    peak = peak.max(val);
})?;
println!("Peak: {:.2}g", peak);
```

## Error Handling

```rust
match sensor.read_all() {
    Ok(data) => { /* use data */ },
    Err(Mpu6050Error::NoChannelsFound) => {
        eprintln!("Device not connected");
    },
    Err(Mpu6050Error::InvalidParameter(msg)) => {
        eprintln!("Bad parameter: {}", msg);
    },
    Err(e) => {
        eprintln!("Error: {}", e);
    }
}
```

## Multi-Threading

Sensor is not thread-safe. Use mutex for shared access:

```rust
use std::sync::{Arc, Mutex};

let sensor = Arc::new(Mutex::new(Mpu6050::new(0)?));

// Clone for thread
let sensor_clone = sensor.clone();
std::thread::spawn(move || {
    let mut s = sensor_clone.lock().unwrap();
    s.stream(100, |data| { /* ... */ })
});
```

## Performance Tips

1. **Batch collection** is fastest for large datasets
2. **Keep callbacks fast** - offload heavy processing
3. **Pre-allocate** when possible: `Vec::with_capacity()`
4. **Match rate to need** - don't oversample

## Examples

Run complete examples:

```bash
cargo run --example vibration_analysis
cargo run --example data_logging
```

See `examples/` directory for full code.
