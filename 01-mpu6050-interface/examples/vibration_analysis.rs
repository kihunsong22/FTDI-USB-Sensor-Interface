//! Example: Vibration analysis using the streaming API
//!
//! This demonstrates how to use the ft232-sensor-interface library
//! for vibration analysis with configurable sample rates.
//!
//! Run with: cargo run --example vibration_analysis

use ft232_sensor_interface::{Mpu6050, StreamControl};
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Vibration Analysis Example");
    println!("==========================\n");

    // Initialize sensor
    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized successfully!\n");

    // Example 1: Real-time monitoring at 500 Hz
    println!("1. Real-time monitoring (500 Hz, 5 seconds)");
    let mut peak_magnitude = 0.0f32;
    let mut sample_count = 0u64;

    let start = Instant::now();
    sensor.stream_for(500, std::time::Duration::from_secs(5), |data| {
        let (ax, ay, az) = data.accel_to_g();
        let magnitude = (ax * ax + ay * ay + az * az).sqrt();

        peak_magnitude = peak_magnitude.max(magnitude);
        sample_count += 1;
    })?;

    let elapsed = start.elapsed();
    let actual_rate = sample_count as f64 / elapsed.as_secs_f64();

    println!("   Samples collected: {}", sample_count);
    println!("   Actual sample rate: {:.1} Hz", actual_rate);
    println!("   Peak acceleration: {:.3}g\n", peak_magnitude);

    // Example 2: Batch collection for FFT analysis
    println!("2. Batch collection for FFT (1000 Hz, 2048 samples)");

    let start = Instant::now();
    let samples = sensor.collect_samples(1000, 2048)?;
    let elapsed = start.elapsed();

    println!("   Collected {} samples in {:.2}s", samples.len(), elapsed.as_secs_f64());

    // Calculate statistics on collected data
    let mut sum_x = 0.0f32;
    let mut sum_y = 0.0f32;
    let mut sum_z = 0.0f32;

    for sample in &samples {
        sum_x += sample.accel_x_g();
        sum_y += sample.accel_y_g();
        sum_z += sample.accel_z_g();
    }

    let avg_x = sum_x / samples.len() as f32;
    let avg_y = sum_y / samples.len() as f32;
    let avg_z = sum_z / samples.len() as f32;

    println!("   Average acceleration:");
    println!("     X: {:.3}g", avg_x);
    println!("     Y: {:.3}g", avg_y);
    println!("     Z: {:.3}g\n", avg_z);

    // Example 3: Conditional streaming with threshold detection
    println!("3. Threshold monitoring (200 Hz, stop on high vibration)");

    let threshold_g = 2.5;
    let mut triggered = false;
    let mut trigger_data = None;

    sensor.stream(200, |data| {
        let (ax, ay, az) = data.accel_to_g();
        let magnitude = (ax * ax + ay * ay + az * az).sqrt();

        if magnitude > threshold_g {
            triggered = true;
            trigger_data = Some(data);
            StreamControl::Break
        } else {
            StreamControl::Continue
        }
    })?;

    if triggered {
        if let Some(data) = trigger_data {
            println!("   Threshold exceeded!");
            println!("   Accel: ({:.3}, {:.3}, {:.3})g",
                data.accel_x_g(), data.accel_y_g(), data.accel_z_g());
        }
    } else {
        println!("   No threshold violations detected");
    }

    println!("\nExample complete!");
    Ok(())
}
