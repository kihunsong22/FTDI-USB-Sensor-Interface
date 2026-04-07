//! ADXL355 Data Collector
//!
//! Collects sensor data in polling or FIFO mode and writes to HDF5 file.

use clap::Parser;
use ft232_adxl355_spi::{
    Adxl355, Hdf5Writer, OutputDataRate, StreamControl, TimeKeeper, TimestampedSample,
};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Parser, Debug)]
#[command(name = "collector")]
#[command(about = "Collect ADXL355 sensor data to HDF5 file")]
struct Args {
    /// Output HDF5 file path
    #[arg(short, long, default_value = "sensor_data.h5")]
    output: PathBuf,

    /// Collection mode: "polling" or "fifo"
    #[arg(short, long, default_value = "polling")]
    mode: String,

    /// Target sample rate in Hz (polling: 1-500, fifo: uses ODR presets)
    #[arg(short, long, default_value = "100")]
    rate: u32,

    /// Duration in seconds (optional, runs until Ctrl+C if omitted)
    #[arg(short, long)]
    duration: Option<u64>,
}

/// Map a rate to the nearest ODR preset
fn rate_to_odr(rate: u32) -> OutputDataRate {
    match rate {
        0..=5 => OutputDataRate::Odr3_906,
        6..=11 => OutputDataRate::Odr7_813,
        12..=23 => OutputDataRate::Odr15_625,
        24..=46 => OutputDataRate::Odr31_25,
        47..=93 => OutputDataRate::Odr62_5,
        94..=187 => OutputDataRate::Odr125,
        188..=375 => OutputDataRate::Odr250,
        376..=750 => OutputDataRate::Odr500,
        751..=1500 => OutputDataRate::Odr1000,
        1501..=3000 => OutputDataRate::Odr2000,
        _ => OutputDataRate::Odr4000,
    }
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    if args.mode != "polling" && args.mode != "fifo" {
        eprintln!("Error: mode must be 'polling' or 'fifo'");
        std::process::exit(1);
    }

    let odr = rate_to_odr(args.rate);
    let actual_rate = odr.as_hz();

    println!("ADXL355 Data Collector");
    println!("======================");
    println!("Mode: {}", args.mode);
    println!("Target rate: {} Hz (actual ODR: {} Hz)", args.rate, actual_rate);
    println!("Output file: {}", args.output.display());
    if let Some(duration) = args.duration {
        println!("Duration: {} seconds", duration);
    } else {
        println!("Duration: continuous (Ctrl+C to stop)");
    }
    println!();

    println!("Initializing sensor...");
    let mut sensor = Adxl355::new(0)?;
    sensor.set_odr(odr)?;
    println!("Sensor initialized!\n");

    println!("Creating HDF5 file...");
    let range_str = match sensor.get_range() {
        ft232_adxl355_spi::Range::G2 => "2g",
        ft232_adxl355_spi::Range::G4 => "4g",
        ft232_adxl355_spi::Range::G8 => "8g",
    };
    let mut writer = Hdf5Writer::create(
        &args.output,
        &args.mode,
        actual_rate,
        range_str,
    )?;
    println!("HDF5 file created!\n");

    // Setup Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, stopping collection...");
        r.store(false, Ordering::SeqCst);
    })?;

    let collection_start = std::time::Instant::now();
    let end_time = args.duration.map(|d| collection_start + std::time::Duration::from_secs(d));

    println!("Starting data collection...");
    println!("Press Ctrl+C to stop\n");

    let result = if args.mode == "fifo" {
        collect_fifo(&mut sensor, &mut writer, odr, running.clone(), end_time)
    } else {
        collect_polling(&mut sensor, &mut writer, args.rate, running.clone(), end_time)
    };

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

fn collect_polling(
    sensor: &mut Adxl355,
    writer: &mut Hdf5Writer,
    rate: u32,
    running: Arc<AtomicBool>,
    end_time: Option<std::time::Instant>,
) -> std::result::Result<(), Box<dyn std::error::Error>> {
    let timer = TimeKeeper::new();
    let mut sample_buffer = Vec::with_capacity(100);
    let mut last_flush = std::time::Instant::now();
    let mut last_progress = std::time::Instant::now();
    let mut progress_count: u64 = 0;

    sensor.stream(rate, |data| {
        if !running.load(Ordering::SeqCst) {
            return StreamControl::Break;
        }

        if let Some(end) = end_time {
            if std::time::Instant::now() >= end {
                return StreamControl::Break;
            }
        }

        let sample = TimestampedSample {
            timestamp: timer.elapsed_secs(),
            data,
        };

        sample_buffer.push(sample);
        progress_count += 1;

        if sample_buffer.len() >= 100 {
            if let Err(e) = writer.append_batch(&sample_buffer) {
                eprintln!("Write error: {}", e);
                return StreamControl::Break;
            }
            sample_buffer.clear();

            if last_flush.elapsed() >= std::time::Duration::from_secs(10) {
                if let Err(e) = writer.flush() {
                    eprintln!("Flush error: {}", e);
                }
                last_flush = std::time::Instant::now();
            }
        }

        if last_progress.elapsed() >= std::time::Duration::from_secs(2) {
            let elapsed = timer.elapsed_secs();
            let rate = progress_count as f64 / elapsed;
            eprint!("\r  Collected {} samples in {:.1}s ({:.0} Hz)    ",
                progress_count, elapsed, rate);
            last_progress = std::time::Instant::now();
        }

        StreamControl::Continue
    })?;

    eprintln!();

    if !sample_buffer.is_empty() {
        writer.append_batch(&sample_buffer)?;
    }

    writer.flush()?;

    Ok(())
}

fn collect_fifo(
    sensor: &mut Adxl355,
    writer: &mut Hdf5Writer,
    odr: OutputDataRate,
    running: Arc<AtomicBool>,
    end_time: Option<std::time::Instant>,
) -> std::result::Result<(), Box<dyn std::error::Error>> {
    sensor.enable_fifo(odr)?;
    println!("FIFO mode enabled (ODR: {} Hz)", odr.as_hz());

    let timer = TimeKeeper::new();
    let mut last_flush = std::time::Instant::now();
    let mut last_progress = std::time::Instant::now();
    let sample_rate = odr.as_hz();
    let dt = 1.0 / sample_rate;
    let mut total_samples: u64 = 0;
    let mut overflow_count: u64 = 0;
    let mut write_buffer: Vec<TimestampedSample> = Vec::with_capacity(256);

    // Wait long enough to accumulate ~10 samples, but stay well under FIFO capacity (32 samples).
    // Minimum 1ms to avoid busy-spinning.
    let poll_sleep = std::time::Duration::from_micros(
        ((10.0 / sample_rate) * 1_000_000.0) as u64
    ).max(std::time::Duration::from_millis(1));

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }
        if let Some(end) = end_time {
            if std::time::Instant::now() >= end {
                break;
            }
        }

        let result = sensor.read_fifo_batch_checked()?;
        if result.overflow_detected {
            overflow_count += 1;
        }
        let batch = result.samples;

        if batch.is_empty() {
            std::thread::sleep(poll_sleep);
            continue;
        }

        let batch_end_time = timer.elapsed_secs();
        let batch_size = batch.len();

        for (i, data) in batch.iter().enumerate() {
            let timestamp = (batch_end_time - (batch_size - 1 - i) as f64 * dt).max(0.0);
            write_buffer.push(TimestampedSample { timestamp, data: *data });
        }

        total_samples += batch_size as u64;

        // Write to HDF5 in larger batches to avoid per-call resize overhead
        if write_buffer.len() >= 200 {
            if let Err(e) = writer.append_batch(&write_buffer) {
                eprintln!("Write error: {}", e);
                break;
            }
            write_buffer.clear();

            if last_flush.elapsed() >= std::time::Duration::from_secs(10) {
                if let Err(e) = writer.flush() {
                    eprintln!("Flush error: {}", e);
                }
                last_flush = std::time::Instant::now();
            }
        }

        if last_progress.elapsed() >= std::time::Duration::from_secs(2) {
            let elapsed = timer.elapsed_secs();
            let rate = total_samples as f64 / elapsed;
            eprint!("\r  Collected {} samples in {:.1}s ({:.0} Hz)    ",
                total_samples, elapsed, rate);
            last_progress = std::time::Instant::now();
        }
    }

    // Flush remaining buffered samples
    if !write_buffer.is_empty() {
        writer.append_batch(&write_buffer)?;
    }

    eprintln!();
    if overflow_count > 0 {
        eprintln!("Warning: {} FIFO overflow(s) detected — some samples were lost", overflow_count);
    }

    sensor.disable_fifo()?;
    writer.flush()?;

    Ok(())
}
