//! FIFO Configuration Check - Verify register settings
//!
//! This reads back MPU6050 configuration registers to verify FIFO setup

use ft232_sensor_interface::Mpu6050;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 FIFO Configuration Check");
    println!("==================================\n");

    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Read configuration BEFORE enabling FIFO
    println!("Before enable_fifo():");
    print_config(&mut sensor)?;

    // Enable FIFO at 1kHz
    println!("\nEnabling FIFO at 1kHz...");
    sensor.enable_fifo(1000)?;
    println!("Done!\n");

    // Read configuration AFTER enabling FIFO
    println!("After enable_fifo():");
    print_config(&mut sensor)?;

    // Wait and check if FIFO is filling
    println!("\nWaiting 100ms to check FIFO fill rate...");
    thread::sleep(Duration::from_millis(100));

    let batch = sensor.read_fifo_batch()?;
    println!("FIFO samples after 100ms: {}", batch.len());

    if batch.len() >= 90 {
        println!("✓ FIFO is filling at ~1kHz");
    } else if batch.len() >= 30 {
        println!("⚠ FIFO filling but slower than expected ({} samples)", batch.len());
    } else {
        println!("✗ FIFO not filling properly ({} samples)", batch.len());
    }

    sensor.disable_fifo()?;
    Ok(())
}

fn print_config(sensor: &mut Mpu6050) -> Result<(), Box<dyn std::error::Error>> {
    let (pwr_mgmt_1, pwr_mgmt_2, config, smplrt_div, fifo_en, user_ctrl) = sensor.read_fifo_config()?;

    println!("  PWR_MGMT_1:  0x{:02X} (sleep={}, clock={})",
             pwr_mgmt_1,
             (pwr_mgmt_1 >> 6) & 1,
             pwr_mgmt_1 & 0x07);
    println!("  PWR_MGMT_2:  0x{:02X} (accel_stby={:03b}, gyro_stby={:03b})",
             pwr_mgmt_2,
             (pwr_mgmt_2 >> 3) & 0x07,
             pwr_mgmt_2 & 0x07);
    println!("  CONFIG:      0x{:02X} (DLPF_CFG={})",
             config,
             config & 0x07);
    println!("  SMPLRT_DIV:  0x{:02X} (divider={}, rate={} Hz)",
             smplrt_div,
             smplrt_div,
             1000 / (1 + smplrt_div as u16));
    println!("  FIFO_EN:     0x{:02X} (accel+gyro={})",
             fifo_en,
             if fifo_en == 0x78 { "yes" } else { "no" });
    println!("  USER_CTRL:   0x{:02X} (FIFO_EN={}, FIFO_RESET={})",
             user_ctrl,
             (user_ctrl >> 6) & 1,
             (user_ctrl >> 2) & 1);

    Ok(())
}
