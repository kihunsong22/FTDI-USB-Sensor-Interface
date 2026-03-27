//! ADXL355 sensor reader - Continuous data acquisition and display

use ft232_adxl355_interface::{Adxl355, Adxl355Error, create_bar};
use std::io::{self, Write};
use std::thread;
use std::time::Duration;

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("ADXL355 Sensor Reader");
    println!("=====================");
    println!("Initializing FT232H I2C interface...");

    let mut sensor = match Adxl355::new(0) {
        Ok(s) => {
            println!("Sensor initialized successfully!");
            println!("Press Ctrl+C to exit\n");
            s
        }
        Err(Adxl355Error::NoChannelsFound) => {
            eprintln!("Error: No FT232H devices found.");
            eprintln!("Please check:");
            eprintln!("  1. FT232H is connected via USB");
            eprintln!("  2. FTDI drivers are installed");
            eprintln!("  3. No other application is using the device");
            return Err(Box::new(Adxl355Error::NoChannelsFound));
        }
        Err(Adxl355Error::InvalidDeviceId(id)) => {
            eprintln!("Error: Invalid ADXL355 device ID: 0x{:02X} (expected 0xAD)", id);
            eprintln!("Please check:");
            eprintln!("  1. ADXL355 is properly connected to FT232H I2C pins");
            eprintln!("  2. SCLK/VSSIO pin is connected to ground (I2C mode)");
            eprintln!("  3. Power supply is correct (2.25-3.6V)");
            return Err(Box::new(Adxl355Error::InvalidDeviceId(id)));
        }
        Err(e) => {
            eprintln!("Error initializing sensor: {}", e);
            return Err(Box::new(e));
        }
    };

    let start_time = std::time::Instant::now();
    let mut sample_count = 0u64;
    let range = sensor.get_range();

    // Clear screen once at start
    print!("\x1B[2J\x1B[H");
    io::stdout().flush()?;

    loop {
        match sensor.read_all() {
            Ok(data) => {
                let (ax, ay, az) = data.accel_to_g(range);
                let temp_c = data.temperature_c();

                let elapsed = start_time.elapsed().as_secs_f64();
                let sample_rate = if elapsed > 0.0 {
                    sample_count as f64 / elapsed
                } else {
                    0.0
                };

                // Move cursor to top
                print!("\x1B[H");

                println!("ADXL355 Sensor Reader - Live Data                              ");
                println!("==================================                              ");
                println!("Time: {:.2}s | Samples: {} | Rate: {:.1} Hz                    ",
                    elapsed, sample_count, sample_rate);
                println!("Temperature: {:.1}°C                                           ", temp_c);
                println!();

                println!("ACCELEROMETER (g)  [-2g ◄─────────────────┼─────────────────► +2g]");
                println!("  X: {:8.5}g  [{}]", ax, create_bar(ax, 2.0, 40));
                println!("  Y: {:8.5}g  [{}]", ay, create_bar(ay, 2.0, 40));
                println!("  Z: {:8.5}g  [{}]", az, create_bar(az, 2.0, 40));

                println!();
                println!("Press Ctrl+C to exit                                           ");

                io::stdout().flush()?;

                sample_count += 1;
            }
            Err(e) => {
                eprintln!("\nError reading sensor: {}", e);
                eprintln!("Retrying...");
                thread::sleep(Duration::from_millis(500));
            }
        }
    }
}
