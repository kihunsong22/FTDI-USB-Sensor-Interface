//! ADXL355 Data Collector
//!
//! Collects sensor data in polling or FIFO mode and writes to HDF5 file.

use clap::Parser;
use ft232_adxl355_spi::{
    Adxl355, Hdf5Writer, OutputDataRate, Range, SensorData, StreamControl, TimeKeeper, TimestampedSample,
};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

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

    /// Duration in seconds (optional, runs until Enter or Ctrl+C if omitted)
    #[arg(short, long)]
    duration: Option<u64>,

    /// Measurement range in g: 2, 4 or 8
    #[arg(long, default_value_t = 2)]
    range: u32,
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

/// Polling reads the data registers, so the ODR must be faster than the poll
/// rate or the same sample is read twice. Presets double, so the result is at
/// most 2x the rate, which keeps the sensor LPF (ODR/4) below the poll Nyquist.
fn poll_rate_to_odr(rate: u32) -> OutputDataRate {
    use OutputDataRate::*;
    [Odr3_906, Odr7_813, Odr15_625, Odr31_25, Odr62_5, Odr125, Odr250, Odr500, Odr1000, Odr2000, Odr4000]
        .into_iter()
        .find(|odr| odr.as_hz() > rate as f64)
        .unwrap_or(Odr4000)
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    if args.mode != "polling" && args.mode != "fifo" {
        eprintln!("Error: mode must be 'polling' or 'fifo'");
        std::process::exit(1);
    }

    let range = match Range::from_g(args.range) {
        Some(range) => range,
        None => {
            eprintln!("Error: range must be 2, 4 or 8");
            std::process::exit(1);
        }
    };

    let odr =if args.mode == "fifo" { rate_to_odr(args.rate) } else { poll_rate_to_odr(args.rate) };
    let actual_rate = odr.as_hz();

    println!("ADXL355 Data Collector");
    println!("======================");
    println!("Mode: {}", args.mode);
    println!("Target rate: {} Hz (actual ODR: {} Hz)", args.rate, actual_rate);
    println!("Range: +/-{} g", args.range);
    println!("Output file: {}", args.output.display());
    if let Some(duration) = args.duration {
        println!("Duration: {} seconds", duration);
    } else {
        println!("Duration: continuous (Enter or Ctrl+C to stop)");
    }
    println!();

    println!("Initializing sensor...");
    let mut sensor = Adxl355::new(0)?;
    sensor.set_odr(odr)?;
    sensor.set_range(range)?;
    println!("Sensor initialized!\n");

    println!("Creating HDF5 file...");
    let range_str = match sensor.get_range() {
        Range::G2 => "2g",
        Range::G4 => "4g",
        Range::G8 => "8g",
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

    // Enter also stops collection; EOF (no console attached) is ignored
    let r = running.clone();
    std::thread::spawn(move || {
        let mut line = String::new();
        if matches!(std::io::stdin().read_line(&mut line), Ok(n) if n > 0) {
            println!("\nReceived Enter, stopping collection...");
            r.store(false, Ordering::SeqCst);
        }
    });

    let collection_start = std::time::Instant::now();
    let end_time = args.duration.map(|d| collection_start + std::time::Duration::from_secs(d));

    println!("Starting data collection...");
    println!("Press Enter or Ctrl+C to stop\n");

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
    writer.set_time_origin(&chrono::Local::now())?;
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
    // This thread polls the FIFO; the writer thread spawned below keeps normal priority
    ft232_adxl355_spi::prioritize_polling();

    let timer = TimeKeeper::new();
    writer.set_time_origin(&chrono::Local::now())?;
    let sample_rate = odr.as_hz();
    let dt = 1.0 / sample_rate;
    let mut overflow_count: u64 = 0;
    let mut write_buffer: Vec<TimestampedSample> = Vec::with_capacity(256);

    // Wait long enough to accumulate ~10 samples, but stay well under FIFO capacity (32 samples).
    // Minimum 1ms to avoid busy-spinning.
    let poll_sleep = std::time::Duration::from_micros(
        ((10.0 / sample_rate) * 1_000_000.0) as u64
    ).max(std::time::Duration::from_millis(1));

    // HDF5 writes, flushes and console output run on their own thread: at 4 kHz the
    // 32-sample FIFO overflows after 8 ms, so the polling loop must never wait on them
    let (tx, rx) = mpsc::channel::<Vec<TimestampedSample>>();
    let writer_ref = &mut *writer;
    let polled = std::thread::scope(|scope| {
        let writer_thread = scope.spawn(move || -> ft232_adxl355_spi::Result<()> {
            let mut last_flush = std::time::Instant::now();
            let mut last_progress = std::time::Instant::now();
            let mut total_samples: u64 = 0;
            for batch in rx {
                writer_ref.append_batch(&batch)?;
                total_samples += batch.len() as u64;

                if last_flush.elapsed() >= std::time::Duration::from_secs(10) {
                    writer_ref.flush()?;
                    last_flush = std::time::Instant::now();
                }

                if last_progress.elapsed() >= std::time::Duration::from_millis(100) {
                    if let Some(last) = batch.last() {
                        eprint!("\r  Collected {} samples in {:.1}s ({:.0} Hz) | Sensor temp: {:.1} C    ",
                            total_samples, last.timestamp, total_samples as f64 / last.timestamp.max(1e-9),
                            last.data.temperature_c());
                    }
                    last_progress = std::time::Instant::now();
                }
            }
            Ok(())
        });

        let polled = (|| -> ft232_adxl355_spi::Result<()> {
            // The temperature is not in the FIFO: read it once per second and
            // store the latest reading with every sample
            let mut temperature: u16 = 0;
            let mut last_temperature: Option<std::time::Instant> = None;
            loop {
                if !running.load(Ordering::SeqCst) {
                    break;
                }
                if let Some(end) = end_time {
                    if std::time::Instant::now() >= end {
                        break;
                    }
                }

                if last_temperature.map_or(true, |t| t.elapsed() >= std::time::Duration::from_secs(1)) {
                    temperature = sensor.read_temperature()?;
                    last_temperature = Some(std::time::Instant::now());
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
                    write_buffer.push(TimestampedSample { timestamp, data: SensorData { temperature, ..*data } });
                }

                // Hand over in larger batches to avoid per-call resize overhead.
                // A send error means the writer thread stopped; its error is reported below.
                if write_buffer.len() >= 200 && tx.send(std::mem::take(&mut write_buffer)).is_err() {
                    break;
                }
            }
            Ok(())
        })();

        // Hand over remaining buffered samples, then let the writer finish
        if !write_buffer.is_empty() {
            let _ = tx.send(std::mem::take(&mut write_buffer));
        }
        drop(tx);
        let written = writer_thread.join().expect("HDF5 writer thread panicked");
        polled.and(written)
    });

    eprintln!();
    if overflow_count > 0 {
        eprintln!("Warning: {} FIFO overflow(s) detected — some samples were lost", overflow_count);
    }

    sensor.standby()?;
    polled?;
    writer.flush()?;

    Ok(())
}
