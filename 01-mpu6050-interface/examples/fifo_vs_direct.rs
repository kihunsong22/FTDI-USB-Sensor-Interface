//! Compare FIFO vs Direct reads
//!
//! Verifies sensors are producing data even though FIFO isn't filling

use ft232_sensor_interface::Mpu6050;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 FIFO vs Direct Read Comparison");
    println!("========================================\n");

    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Test direct reads BEFORE enabling FIFO
    println!("1. Direct reads (before FIFO):");
    for i in 0..5 {
        let data = sensor.read_all()?;
        let (ax, ay, az) = data.accel_to_g();
        println!("   Sample {}: ax={:.3}g, ay={:.3}g, az={:.3}g",
                 i+1, ax, ay, az);
        thread::sleep(Duration::from_millis(20));
    }

    // Enable FIFO
    println!("\n2. Enabling FIFO at 1kHz...");
    sensor.enable_fifo(1000)?;
    println!("   FIFO enabled!\n");

    // Check if direct reads still work AFTER enabling FIFO
    println!("3. Direct reads (after FIFO enabled):");
    for i in 0..5 {
        let data = sensor.read_all()?;
        let (ax, ay, az) = data.accel_to_g();
        println!("   Sample {}: ax={:.3}g, ay={:.3}g, az={:.3}g",
                 i+1, ax, ay, az);
        thread::sleep(Duration::from_millis(20));
    }

    // Check FIFO after those direct reads
    println!("\n4. FIFO check after 100ms:");
    thread::sleep(Duration::from_millis(100));
    let batch = sensor.read_fifo_batch()?;
    println!("   FIFO samples: {}", batch.len());

    if batch.is_empty() {
        println!("\n⚠ Sensors are working (direct reads OK) but FIFO not filling!");
        println!("   This suggests FIFO hardware/configuration issue.");
    } else {
        println!("\n✓ FIFO is working!");
    }

    sensor.disable_fifo()?;
    Ok(())
}
