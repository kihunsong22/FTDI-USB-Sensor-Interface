//! Raw FIFO Count Check - Monitor FIFO count register directly
//!
//! This monitors the FIFO count registers directly to see if they're changing

use ft232_sensor_interface::Mpu6050;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 Raw FIFO Count Monitor");
    println!("================================\n");

    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    println!("Enabling FIFO at 1kHz...");
    sensor.enable_fifo(1000)?;
    println!("FIFO enabled!\n");

    println!("Monitoring FIFO count register for 500ms:");
    println!("(Watching for any changes in the count)");
    println!();

    let start = std::time::Instant::now();
    let mut last_count = 0u16;
    let mut change_count = 0;

    while start.elapsed() < Duration::from_millis(500) {
        let count = sensor.get_fifo_count()?;

        if count != last_count {
            println!("  t={:3}ms: FIFO count = {} bytes ({} samples)",
                    start.elapsed().as_millis(),
                    count,
                    count / 12);
            last_count = count;
            change_count += 1;
        }

        thread::sleep(Duration::from_millis(10));
    }

    println!();
    if change_count == 0 {
        println!("✗ FIFO count NEVER changed - FIFO not accumulating data!");
        println!("  Final count: {} bytes", last_count);
    } else {
        println!("✓ FIFO count changed {} times", change_count);
        println!("  This indicates FIFO is accumulating data");
    }

    sensor.disable_fifo()?;
    Ok(())
}
