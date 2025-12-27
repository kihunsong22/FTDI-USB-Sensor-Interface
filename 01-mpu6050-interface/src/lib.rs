//! FT232H-based sensor interface library for MPU6050
//!
//! This library provides a high-level interface to the MPU6050 6-axis motion sensor
//! using the FTDI FT232H USB-to-I2C bridge via the libMPSSE library.
//!
//! # Quick Start
//!
//! ## Simple Data Reading
//! ```no_run
//! use ft232_sensor_interface::Mpu6050;
//!
//! let mut sensor = Mpu6050::new(0)?;
//! let data = sensor.read_all()?;
//!
//! // Access converted values directly
//! println!("Accel X: {:.2}g", data.accel_x_g());
//! println!("Gyro Z: {:.2}°/s", data.gyro_z_dps());
//! # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
//! ```
//!
//! ## Streaming for Real-Time Processing
//! ```no_run
//! use ft232_sensor_interface::{Mpu6050, StreamControl};
//!
//! let mut sensor = Mpu6050::new(0)?;
//!
//! // Stream at 500 Hz for vibration monitoring
//! sensor.stream(500, |data| {
//!     let (ax, ay, az) = data.accel_to_g();
//!     let magnitude = (ax*ax + ay*ay + az*az).sqrt();
//!
//!     if magnitude > 3.0 {
//!         println!("High vibration detected: {:.2}g", magnitude);
//!         StreamControl::Break  // Stop on threshold
//!     } else {
//!         StreamControl::Continue
//!     }
//! })?;
//! # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
//! ```
//!
//! ## Data Collection for Analysis
//! ```no_run
//! use ft232_sensor_interface::Mpu6050;
//!
//! let mut sensor = Mpu6050::new(0)?;
//!
//! // Collect 2048 samples at 1000 Hz for FFT analysis
//! let samples = sensor.collect_samples(1000, 2048)?;
//!
//! // Perform frequency analysis, logging, etc.
//! for sample in &samples {
//!     // Process collected data...
//! }
//! # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
//! ```
//!
//! ## Time-Based Acquisition
//! ```no_run
//! use ft232_sensor_interface::Mpu6050;
//! use std::time::Duration;
//!
//! let mut sensor = Mpu6050::new(0)?;
//! let mut peak_gyro = 0.0f32;
//!
//! // Monitor for 10 seconds at 200 Hz
//! sensor.stream_for(200, Duration::from_secs(10), |data| {
//!     let (gx, gy, gz) = data.gyro_to_dps();
//!     peak_gyro = peak_gyro.max(gx.abs()).max(gy.abs()).max(gz.abs());
//! })?;
//!
//! println!("Peak rotation rate: {:.2}°/s", peak_gyro);
//! # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
//! ```
//!
//! ## High-Speed FIFO Mode (~850 Hz)
//!
//! For applications requiring higher sample rates (vibration analysis, frequency
//! analysis), the MPU6050's FIFO buffer allows high-speed internal sampling with
//! periodic batch reads. Actual sample rate is ~850 Hz due to hardware clock tolerance.
//!
//! ```no_run
//! use ft232_sensor_interface::Mpu6050;
//!
//! let mut sensor = Mpu6050::new(0)?;
//!
//! // Enable FIFO mode for high-speed sampling (~850 Hz actual)
//! sensor.enable_fifo(1000)?;
//!
//! // Collect 2048 samples for FFT analysis
//! let samples = sensor.collect_samples_fifo(1000, 2048)?;
//!
//! // Or stream FIFO batches with callback (20ms intervals, ~17 samples/batch)
//! sensor.stream_fifo(20, |batch| {
//!     println!("Received {} samples", batch.len());
//!     // Process batch...
//!     ft232_sensor_interface::StreamControl::Continue
//! })?;
//! # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
//! ```
//!
//! **Note**: FIFO mode provides buffered high-speed sampling (~2.5x faster than
//! direct polling). Samples are read in batches with 20-50ms latency. Use direct
//! `stream()` for real-time applications requiring immediate response.

pub mod error;
mod ffi;
pub mod mpu6050;

// Re-export public API
pub use error::{Mpu6050Error, Result};
pub use mpu6050::{Mpu6050, SensorData, StreamControl};
