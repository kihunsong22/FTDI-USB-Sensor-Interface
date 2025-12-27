//! Example: High-speed data collection using FIFO mode
//!
//! Demonstrates 1kHz sampling for vibration/frequency analysis
//!
//! Run with: cargo run --example fifo_1khz_analysis

use ft232_sensor_interface::{Mpu6050, StreamControl};
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 FIFO Mode - 1kHz Sampling Example");
    println!("==========================================\n");

    // Initialize sensor
    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Example 1: Enable FIFO and collect batch
    println!("1. Enabling FIFO at 1kHz...");
    sensor.enable_fifo(1000)?;
    println!("   FIFO enabled!\n");

    // Wait for some samples to accumulate
    std::thread::sleep(std::time::Duration::from_millis(100));

    // Read FIFO batch
    let samples = sensor.read_fifo_batch()?;
    println!("   Read {} samples from FIFO", samples.len());
    println!("   Expected ~100 samples at 1kHz for 100ms\n");

    // Example 2: Stream FIFO data
    println!("2. Streaming FIFO data (5 seconds at 1kHz)...");
    let mut total_samples = 0;
    let mut batches = 0;

    let start = Instant::now();
    sensor.stream_fifo(50, |batch| {
        batches += 1;
        total_samples += batch.len();

        if batches % 20 == 0 {
            println!("   Batch {}: {} samples (total: {})",
                     batches, batch.len(), total_samples);
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
    println!("   Actual sample rate: {:.1} Hz\n", actual_rate);

    // Example 3: Collect specific number for FFT
    println!("3. Collecting 2048 samples for FFT analysis...");
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
    println!("4. Disabling FIFO...");
    sensor.disable_fifo()?;
    println!("   FIFO disabled\n");

    println!("Example complete!");
    Ok(())
}
