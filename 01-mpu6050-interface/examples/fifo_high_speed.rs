//! Example: High-speed data collection using FIFO mode
//!
//! Demonstrates high-speed sampling (~850 Hz) for vibration/frequency analysis
//! using the MPU6050's internal FIFO buffer.
//!
//! Run with: cargo run --release --example fifo_high_speed

use ft232_sensor_interface::{Mpu6050, StreamControl};
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 FIFO Mode - High-Speed Sampling Example");
    println!("================================================\n");

    // Initialize sensor
    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Example 1: Enable FIFO and start streaming immediately
    println!("1. Enabling FIFO for high-speed sampling...");
    sensor.enable_fifo(1000)?;
    println!("   FIFO enabled!\n");

    // Example 1: Stream FIFO data
    println!("1. Streaming FIFO data (5 seconds)...");
    let mut total_samples = 0;
    let mut batches = 0;
    let mut empty_batches = 0;

    let start = Instant::now();
    sensor.stream_fifo(20, |batch| {
        batches += 1;
        total_samples += batch.len();

        if batch.is_empty() {
            empty_batches += 1;
        }

        if batches % 50 == 0 {
            println!("   Batch {}: {} samples (total: {}, empty: {})",
                     batches, batch.len(), total_samples, empty_batches);
        }

        if start.elapsed() >= std::time::Duration::from_secs(5) {
            StreamControl::Break
        } else {
            StreamControl::Continue
        }
    })?;

    let elapsed = start.elapsed().as_secs_f64();
    let actual_rate = total_samples as f64 / elapsed;

    println!("   Total samples: {} in {:.2}s", total_samples, elapsed);
    println!("   Actual sample rate: {:.1} Hz", actual_rate);
    println!("   Empty batches (overflows): {} / {}\n", empty_batches, batches);

    // Example 2: Collect specific number for FFT
    println!("2. Collecting 2048 samples for FFT analysis...");
    let fft_samples = sensor.collect_samples_fifo(1000, 2048)?;

    println!("   Collected {} samples", fft_samples.len());

    // Calculate some statistics
    let mut sum_x = 0.0f32;
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;

    for sample in &fft_samples {
        let x = sample.accel_x_g();
        sum_x += x;
        min_x = min_x.min(x);
        max_x = max_x.max(x);
    }

    let avg_x = sum_x / fft_samples.len() as f32;

    println!("   Accel X statistics:");
    println!("     Average: {:.3}g", avg_x);
    println!("     Min: {:.3}g", min_x);
    println!("     Max: {:.3}g", max_x);
    println!("     Range: {:.3}g\n", max_x - min_x);

    // Disable FIFO
    println!("3. Disabling FIFO...");
    sensor.disable_fifo()?;
    println!("   FIFO disabled\n");

    println!("Example complete!");
    Ok(())
}
