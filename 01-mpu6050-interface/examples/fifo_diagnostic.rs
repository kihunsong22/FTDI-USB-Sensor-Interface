//! FIFO Diagnostic - Check actual fill rate
//!
//! This measures the actual FIFO fill rate to verify 1kHz configuration

use ft232_sensor_interface::Mpu6050;
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 FIFO Diagnostic");
    println!("========================\n");

    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Enable FIFO at 1kHz
    println!("Enabling FIFO at 1kHz...");
    sensor.enable_fifo(1000)?;
    println!("FIFO enabled!\n");

    // Wait for FIFO to accumulate data
    println!("Waiting 100ms for FIFO to fill...");
    thread::sleep(Duration::from_millis(100));

    // Read FIFO and check fill rate
    let start = Instant::now();
    let batch = sensor.read_fifo_batch()?;
    let read_time = start.elapsed();

    let fill_time_ms = 100.0;
    let samples_per_sec = (batch.len() as f64 / fill_time_ms) * 1000.0;

    println!("Results:");
    println!("  Samples after 100ms: {}", batch.len());
    println!("  Read operation took: {:.2}ms", read_time.as_secs_f64() * 1000.0);
    println!("  Calculated fill rate: {:.1} Hz", samples_per_sec);
    println!();

    if samples_per_sec >= 900.0 {
        println!("✓ FIFO is filling at ~1kHz");
    } else if samples_per_sec >= 300.0 {
        println!("⚠ FIFO fill rate is lower than expected");
        println!("  Expected: ~1000 Hz");
        println!("  Actual: {:.1} Hz", samples_per_sec);
    } else {
        println!("✗ FIFO fill rate is much lower than expected");
    }

    sensor.disable_fifo()?;
    Ok(())
}
