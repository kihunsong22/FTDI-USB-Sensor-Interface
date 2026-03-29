//! ADXL355 sensor reader - Continuous data acquisition and display
//!
//! Uses FIFO mode for high-speed sampling over SPI (target: 4kHz).
//! Display updates at ~120 Hz, decoupled from read rate.

use ft232_adxl355_spi::{Adxl355, Adxl355Error, OutputDataRate, create_bar};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("ADXL355 Sensor Reader");
    println!("=====================");
    println!("Initializing FT232H SPI interface...");

    let mut sensor = match Adxl355::new(0) {
        Ok(s) => {
            println!("Sensor initialized successfully!");
            s
        }
        Err(Adxl355Error::NoChannelsFound) => {
            eprintln!("Error: No FT232H devices found.");
            return Err(Box::new(Adxl355Error::NoChannelsFound));
        }
        Err(e) => {
            eprintln!("Error initializing sensor: {}", e);
            return Err(Box::new(e));
        }
    };

    let range = sensor.get_range();
    let odr = OutputDataRate::Odr4000;

    // Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    println!("Enabling FIFO mode at {} Hz...", odr.as_hz());
    sensor.enable_fifo(odr)?;
    println!("FIFO enabled! Press Ctrl+C to exit\n");

    let start_time = Instant::now();
    let mut total_samples = 0u64;
    let mut last_ax = 0.0f32;
    let mut last_ay = 0.0f32;
    let mut last_az = 0.0f32;
    let mut last_batch_size = 0usize;

    // Clear screen
    print!("\x1B[2J\x1B[H");
    io::stdout().flush()?;

    let display_interval = Duration::from_millis(8); // ~120 Hz display
    let mut next_display = Instant::now() + display_interval;

    while running.load(Ordering::SeqCst) {
        // Read FIFO — fast, no display overhead
        match sensor.read_fifo_batch() {
            Ok(batch) if !batch.is_empty() => {
                total_samples += batch.len() as u64;
                last_batch_size = batch.len();
                let data = batch.last().unwrap();
                let (ax, ay, az) = data.accel_to_g(range);
                last_ax = ax;
                last_ay = ay;
                last_az = az;
            }
            Err(e) => {
                eprintln!("FIFO read error: {}", e);
            }
            _ => {} // Empty batch, skip
        }

        // Only update display at ~120 Hz
        let now = Instant::now();
        if now >= next_display {
            next_display = now + display_interval;

            let elapsed = start_time.elapsed().as_secs_f64();
            let sample_rate = if elapsed > 0.0 {
                total_samples as f64 / elapsed
            } else {
                0.0
            };

            // Read temperature once per display update (not in FIFO)
            let temp_str = match sensor.read_temperature() {
                Ok(raw) => {
                    let temp_c = ((raw as f32 - 1885.0) / -9.05) + 25.0;
                    format!("{:.1}°C", temp_c)
                }
                Err(_) => "N/A".to_string(),
            };

            print!("\x1B[H");
            println!("ADXL355 Sensor Reader - FIFO Mode ({} Hz ODR)                   ", odr.as_hz());
            println!("==================================                              ");
            println!("Time: {:.2}s | Samples: {} | Rate: {:.0} Hz | Batch: {}         ",
                elapsed, total_samples, sample_rate, last_batch_size);
            println!("Temperature: {}                                                  ", temp_str);
            println!();
            println!("ACCELEROMETER (g)  [-2g ◄─────────────────┼─────────────────► +2g]");
            println!("  X: {:8.5}g  [{}]", last_ax, create_bar(last_ax, 2.0, 40));
            println!("  Y: {:8.5}g  [{}]", last_ay, create_bar(last_ay, 2.0, 40));
            println!("  Z: {:8.5}g  [{}]", last_az, create_bar(last_az, 2.0, 40));
            println!();
            println!("Press Ctrl+C to exit                                           ");
            let _ = io::stdout().flush();
        }
    }

    println!("\nShutting down...");
    Ok(())
}
