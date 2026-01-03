//! MPU6050 Data Collector
//!
//! Collects sensor data in polling or FIFO mode and writes to HDF5 file.
//!
//! Usage:
//!   collector --output data.h5 --mode fifo --rate 1000 --duration 60

use clap::Parser;
use ft232_sensor_interface::{Hdf5Writer, Mpu6050, StreamControl, TimeKeeper, TimestampedSample};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Parser, Debug)]
#[command(name = "collector")]
#[command(about = "Collect MPU6050 sensor data to HDF5 file", long_about = None)]
struct Args {
    /// Output HDF5 file path
    #[arg(short, long, default_value = "sensor_data.h5")]
    output: PathBuf,

    /// Collection mode: "polling" or "fifo"
    #[arg(short, long, default_value = "polling")]
    mode: String,

    /// Target sample rate in Hz (polling: 1-100, fifo: 4-1000)
    #[arg(short, long, default_value = "100")]
    rate: u32,

    /// Duration in seconds (optional, runs until Ctrl+C if omitted)
    #[arg(short, long)]
    duration: Option<u64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Validate arguments
    if args.mode != "polling" && args.mode != "fifo" {
        eprintln!("Error: mode must be 'polling' or 'fifo'");
        std::process::exit(1);
    }

    if args.mode == "polling" && args.rate > 100 {
        eprintln!("Warning: Polling mode limited to ~100 Hz, reducing from {} Hz", args.rate);
    }

    if args.mode == "fifo" && (args.rate < 4 || args.rate > 1000) {
        eprintln!("Error: FIFO mode rate must be 4-1000 Hz");
        std::process::exit(1);
    }

    println!("MPU6050 Data Collector");
    println!("======================");
    println!("Mode: {}", args.mode);
    println!("Target rate: {} Hz", args.rate);
    println!("Output file: {}", args.output.display());
    if let Some(duration) = args.duration {
        println!("Duration: {} seconds", duration);
    } else {
        println!("Duration: continuous (Ctrl+C to stop)");
    }
    println!();

    // Initialize sensor
    println!("Initializing sensor...");
    let mut sensor = Mpu6050::new(0)?;
    println!("Sensor initialized!\n");

    // Create HDF5 writer
    println!("Creating HDF5 file...");
    let mut writer = Hdf5Writer::create(
        &args.output,
        &args.mode,
        args.rate as f64,
    )?;
    println!("HDF5 file created!\n");

    // Setup Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, stopping collection...");
        r.store(false, Ordering::SeqCst);
    })?;

    // Start time tracking
    let collection_start = std::time::Instant::now();
    let end_time = args.duration.map(|d| collection_start + std::time::Duration::from_secs(d));

    println!("Starting data collection...");
    println!("Press Ctrl+C to stop\n");

    // Run collection based on mode
    let result = if args.mode == "fifo" {
        collect_fifo(&mut sensor, &mut writer, running.clone(), end_time)
    } else {
        collect_polling(&mut sensor, &mut writer, args.rate, running.clone(), end_time)
    };

    // Handle result
    match result {
        Ok(()) => {
            let elapsed = collection_start.elapsed().as_secs_f64();
            let samples = writer.sample_count();
            let actual_rate = samples as f64 / elapsed;

            println!("\nCollection complete!");
            println!("Total samples: {}", samples);
            println!("Elapsed time: {:.2} seconds", elapsed);
            println!("Actual sample rate: {:.1} Hz", actual_rate);
            println!("File: {}", args.output.display());
        }
        Err(e) => {
            eprintln!("\nError during collection: {}", e);
            eprintln!("Attempting to flush data...");
            if let Err(flush_err) = writer.flush() {
                eprintln!("Failed to flush: {}", flush_err);
            }
            return Err(e);
        }
    }

    Ok(())
}

/// Collect data in polling mode
fn collect_polling(
    sensor: &mut Mpu6050,
    writer: &mut Hdf5Writer,
    rate: u32,
    running: Arc<AtomicBool>,
    end_time: Option<std::time::Instant>,
) -> Result<(), Box<dyn std::error::Error>> {
    let timer = TimeKeeper::new();
    let mut sample_buffer = Vec::with_capacity(100);
    let mut last_flush = std::time::Instant::now();

    sensor.stream(rate, |data| {
        // Check if we should stop
        if !running.load(Ordering::SeqCst) {
            return StreamControl::Break;
        }

        if let Some(end) = end_time {
            if std::time::Instant::now() >= end {
                return StreamControl::Break;
            }
        }

        // Create timestamped sample
        let sample = TimestampedSample {
            timestamp: timer.elapsed_secs(),
            data,
        };

        sample_buffer.push(sample);

        // Write batch every 100 samples
        if sample_buffer.len() >= 100 {
            if let Err(e) = writer.append_batch(&sample_buffer) {
                eprintln!("Write error: {}", e);
                return StreamControl::Break;
            }
            sample_buffer.clear();

            // Periodic flush (every 10 seconds)
            if last_flush.elapsed() >= std::time::Duration::from_secs(10) {
                if let Err(e) = writer.flush() {
                    eprintln!("Flush error: {}", e);
                }
                last_flush = std::time::Instant::now();
            }
        }

        StreamControl::Continue
    })?;

    // Write remaining samples
    if !sample_buffer.is_empty() {
        writer.append_batch(&sample_buffer)?;
    }

    // Final flush
    writer.flush()?;

    Ok(())
}

/// Collect data in FIFO mode
fn collect_fifo(
    sensor: &mut Mpu6050,
    writer: &mut Hdf5Writer,
    running: Arc<AtomicBool>,
    end_time: Option<std::time::Instant>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Enable FIFO mode
    sensor.enable_fifo(1000)?;
    println!("FIFO mode enabled");

    let timer = TimeKeeper::new();
    let mut last_flush = std::time::Instant::now();
    let sample_rate = 850.0; // Actual FIFO rate

    sensor.stream_fifo(20, |batch| {
        // Check if we should stop
        if !running.load(Ordering::SeqCst) {
            return StreamControl::Break;
        }

        if let Some(end) = end_time {
            if std::time::Instant::now() >= end {
                return StreamControl::Break;
            }
        }

        if batch.is_empty() {
            return StreamControl::Continue;
        }

        // Get current timestamp (end of batch)
        let batch_end_time = timer.elapsed_secs();
        let batch_size = batch.len();

        // Interpolate timestamps for samples in batch
        // Assume evenly spaced samples
        let dt = 1.0 / sample_rate;
        let timestamped_samples: Vec<TimestampedSample> = batch.iter()
            .enumerate()
            .map(|(i, data)| {
                let timestamp = batch_end_time - (batch_size - 1 - i) as f64 * dt;
                TimestampedSample {
                    timestamp,
                    data: *data,
                }
            })
            .collect();

        // Write batch
        if let Err(e) = writer.append_batch(&timestamped_samples) {
            eprintln!("Write error: {}", e);
            return StreamControl::Break;
        }

        // Periodic flush
        if last_flush.elapsed() >= std::time::Duration::from_secs(10) {
            if let Err(e) = writer.flush() {
                eprintln!("Flush error: {}", e);
            }
            last_flush = std::time::Instant::now();
        }

        StreamControl::Continue
    })?;

    // Disable FIFO
    sensor.disable_fifo()?;

    // Final flush
    writer.flush()?;

    Ok(())
}
