//! MPU6050 sensor reader - Continuous data acquisition and display
//!
//! This executable continuously reads accelerometer and gyroscope data from the
//! MPU6050 sensor via FT232H and displays it to the console.

use ft232_sensor_interface::{Mpu6050, Mpu6050Error};
use std::io::{self, Write};
use std::thread;
use std::time::Duration;

/// Create a horizontal bar graph for a value
/// Range: -2.0 to +2.0 for accelerometer (g), -250 to +250 for gyroscope (°/s)
fn create_bar(value: f32, max_value: f32, width: usize) -> String {
    let normalized = (value / max_value).clamp(-1.0, 1.0);
    let center = width / 2;
    let bar_length = ((normalized.abs() * center as f32) as usize).min(center);

    let mut bar = String::new();

    if normalized < 0.0 {
        // Negative value: bar extends left from center
        bar.push_str(&" ".repeat(center - bar_length));
        bar.push_str(&"█".repeat(bar_length));
        bar.push('|');
        bar.push_str(&" ".repeat(center));
    } else {
        // Positive value: bar extends right from center
        bar.push_str(&" ".repeat(center));
        bar.push('|');
        bar.push_str(&"█".repeat(bar_length));
        bar.push_str(&" ".repeat(center - bar_length));
    }

    bar
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MPU6050 Sensor Reader");
    println!("====================");
    println!("Initializing FT232H I2C interface...");

    // Initialize the sensor on channel 0
    let mut sensor = match Mpu6050::new(0) {
        Ok(s) => {
            println!("Sensor initialized successfully!");
            println!("Press Ctrl+C to exit\n");
            s
        }
        Err(Mpu6050Error::NoChannelsFound) => {
            eprintln!("Error: No FT232H devices found.");
            eprintln!("Please check:");
            eprintln!("  1. FT232H is connected via USB");
            eprintln!("  2. FTDI drivers are installed");
            eprintln!("  3. No other application is using the device");
            return Err(Box::new(Mpu6050Error::NoChannelsFound));
        }
        Err(Mpu6050Error::InvalidDeviceId(id)) => {
            eprintln!("Error: Invalid MPU6050 device ID: 0x{:02X}", id);
            eprintln!("Please check:");
            eprintln!("  1. MPU6050 is properly connected to FT232H I2C pins");
            eprintln!("  2. Power supply to MPU6050 is correct (3.3V)");
            eprintln!("  3. Pull-up resistors are present on SDA/SCL lines");
            return Err(Box::new(Mpu6050Error::InvalidDeviceId(id)));
        }
        Err(e) => {
            eprintln!("Error initializing sensor: {}", e);
            return Err(Box::new(e));
        }
    };

    let start_time = std::time::Instant::now();
    let mut sample_count = 0u64;

    // Clear screen once at start
    print!("\x1B[2J\x1B[H");
    io::stdout().flush()?;

    loop {
        match sensor.read_all() {
            Ok(data) => {
                // Convert to physical units
                let (ax, ay, az) = data.accel_to_g();
                let (gx, gy, gz) = data.gyro_to_dps();

                // Calculate elapsed time and rate
                let elapsed = start_time.elapsed().as_secs_f64();
                let sample_rate = if elapsed > 0.0 {
                    sample_count as f64 / elapsed
                } else {
                    0.0
                };

                // Move cursor to top without clearing (reduces flicker)
                print!("\x1B[H");

                // Header
                println!("MPU6050 Sensor Reader - Live Data                              ");
                println!("==================================                              ");
                println!("Time: {:.2}s | Samples: {} | Rate: {:.1} Hz                    ",
                    elapsed, sample_count, sample_rate);
                println!();

                // Accelerometer display
                println!("ACCELEROMETER (g)                    -2g ◄─────────┼─────────► +2g");
                println!("  X: {:7.3}g  [{}]", ax, create_bar(ax, 2.0, 40));
                println!("  Y: {:7.3}g  [{}]", ay, create_bar(ay, 2.0, 40));
                println!("  Z: {:7.3}g  [{}]", az, create_bar(az, 2.0, 40));

                println!();

                // Gyroscope display
                println!("GYROSCOPE (°/s)                   -250°/s ◄───────┼───────► +250°/s");
                println!("  X: {:7.2}°/s [{}]", gx, create_bar(gx, 250.0, 40));
                println!("  Y: {:7.2}°/s [{}]", gy, create_bar(gy, 250.0, 40));
                println!("  Z: {:7.2}°/s [{}]", gz, create_bar(gz, 250.0, 40));

                println!();
                println!("Press Ctrl+C to exit                                           ");

                // Flush to ensure immediate display
                io::stdout().flush()?;

                sample_count += 1;
            }
            Err(e) => {
                eprintln!("\nError reading sensor: {}", e);
                eprintln!("Retrying...");
                thread::sleep(Duration::from_millis(500));
            }
        }

        // No sleep - run as fast as I2C allows (~100 Hz with optimized single read)
    }
}
