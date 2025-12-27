//! Example: Data logging with timestamp
//!
//! Demonstrates continuous data logging to a file at configurable rates.
//!
//! Run with: cargo run --example data_logging

use ft232_sensor_interface::Mpu6050;
use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Data Logging Example");
    println!("====================\n");

    // Initialize sensor
    let mut sensor = Mpu6050::new(0)?;

    // Create log file
    let mut log_file = File::create("sensor_log.csv")?;
    writeln!(log_file, "timestamp_ms,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps")?;

    println!("Logging data at 100 Hz for 10 seconds...");
    println!("Output file: sensor_log.csv\n");

    let start_time = Instant::now();
    let log_duration = Duration::from_secs(10);

    let samples = sensor.stream_for(100, log_duration, |data| {
        let timestamp_ms = start_time.elapsed().as_millis();

        // Write to CSV
        let _ = writeln!(
            log_file,
            "{},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2}",
            timestamp_ms,
            data.accel_x_g(),
            data.accel_y_g(),
            data.accel_z_g(),
            data.gyro_x_dps(),
            data.gyro_y_dps(),
            data.gyro_z_dps()
        );
    })?;

    println!("Logged {} samples", samples);
    println!("Average rate: {:.1} Hz", samples as f64 / log_duration.as_secs_f64());
    println!("\nLog file saved successfully!");

    Ok(())
}
