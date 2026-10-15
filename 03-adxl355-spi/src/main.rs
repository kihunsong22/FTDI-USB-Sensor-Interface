//! ADXL355 sensor reader - Continuous data acquisition and display
//!
//! Uses FIFO mode for high-speed sampling over SPI (default 4 kHz ODR).
//! FIFO polling runs on its own thread; display updates at ~120 Hz.

use clap::Parser;
use ft232_adxl355_spi::{Adxl355, Adxl355Error, FifoBatchResult, OutputDataRate, Range, create_bar};
#[cfg(feature = "analysis")]
use ft232_adxl355_spi::analysis::{
    compute_rms, find_frequency_peaks, FrequencyPeak, OverallVibration,
    VibrationMeter, STANDARD_GRAVITY,
};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::{Duration, Instant};
#[cfg(feature = "analysis")]
use std::collections::VecDeque;

#[cfg(feature = "analysis")]
const FFT_WINDOW: usize = 2048;
/// Rolling window for the meter-style values
#[cfg(feature = "analysis")]
const VIB_WINDOW_SECONDS: f64 = 1.0;

/// Messages from the acquisition thread
enum Acquired {
    Batch(FifoBatchResult),
    Temperature(u16),
    Error(String),
}

#[derive(Parser, Debug)]
#[command(name = "adxl355-spi-reader")]
#[command(about = "Live ADXL355 display")]
struct Args {
    /// Output data rate in Hz: 4000, 2000, 1000, 500, 250 or 125
    #[arg(long, default_value_t = 4000)]
    odr: u32,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let odr = match args.odr {
        4000 => OutputDataRate::Odr4000,
        2000 => OutputDataRate::Odr2000,
        1000 => OutputDataRate::Odr1000,
        500 => OutputDataRate::Odr500,
        250 => OutputDataRate::Odr250,
        125 => OutputDataRate::Odr125,
        other => {
            eprintln!("Error: unsupported ODR {} (use 4000, 2000, 1000, 500, 250 or 125)", other);
            std::process::exit(1);
        }
    };
    let odr_hz = odr.as_hz();

    println!("ADXL355 Sensor Reader");
    println!("=====================");
    println!("Initializing FT232H SPI interface...");

    // Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // FIFO polling runs on its own thread so display and analysis work can't
    // stall it: at 4 kHz the 32-sample FIFO overflows after 8 ms
    let (init_tx, init_rx) = mpsc::channel();
    let (tx, rx) = mpsc::channel();
    let acquisition = {
        let running = running.clone();
        thread::spawn(move || acquire(odr, running, init_tx, tx))
    };

    let range = match init_rx.recv() {
        Ok(Ok(range)) => {
            println!("Sensor initialized, FIFO enabled at {} Hz. Press Ctrl+C to exit\n", odr_hz);
            range
        }
        Ok(Err(Adxl355Error::NoChannelsFound)) => {
            eprintln!("Error: No FT232H devices found.");
            return Err(Box::new(Adxl355Error::NoChannelsFound));
        }
        Ok(Err(e)) => {
            eprintln!("Error initializing sensor: {}", e);
            return Err(Box::new(e));
        }
        Err(_) => return Err("acquisition thread exited during init".into()),
    };

    let start_time = Instant::now();
    let mut total_samples = 0u64;
    let mut last_ax = 0.0f32;
    let mut last_ay = 0.0f32;
    let mut last_az = 0.0f32;
    let mut last_batch_size = 0usize;
    let mut fifo_overflows = 0u64;
    let mut temperature: Option<u16> = None;

    // Analysis buffers: rolling window of g-converted samples per axis
    #[cfg(feature = "analysis")]
    let mut buf_x: VecDeque<f32> = VecDeque::with_capacity(FFT_WINDOW);
    #[cfg(feature = "analysis")]
    let mut buf_y: VecDeque<f32> = VecDeque::with_capacity(FFT_WINDOW);
    #[cfg(feature = "analysis")]
    let mut buf_z: VecDeque<f32> = VecDeque::with_capacity(FFT_WINDOW);
    #[cfg(feature = "analysis")]
    let mut rms_x = 0.0f32;
    #[cfg(feature = "analysis")]
    let mut rms_y = 0.0f32;
    #[cfg(feature = "analysis")]
    let mut rms_z = 0.0f32;
    #[cfg(feature = "analysis")]
    let mut peaks_x: Vec<FrequencyPeak> = Vec::new();
    #[cfg(feature = "analysis")]
    let mut peaks_y: Vec<FrequencyPeak> = Vec::new();
    #[cfg(feature = "analysis")]
    let mut peaks_z: Vec<FrequencyPeak> = Vec::new();
    #[cfg(feature = "analysis")]
    let mut last_fft_time = Instant::now();
    #[cfg(feature = "analysis")]
    let fft_interval = Duration::from_millis(500);

    // Meter-style vibration: continuous filtering, restarted on FIFO overflow
    #[cfg(feature = "analysis")]
    let mut vib_meter = VibrationMeter::new(odr_hz, odr_hz / 4.0, VIB_WINDOW_SECONDS);
    #[cfg(feature = "analysis")]
    let mut vib: Option<[OverallVibration; 3]> = None;
    #[cfg(feature = "analysis")]
    let mut last_vib_time = Instant::now();

    // Clear screen
    print!("\x1B[2J\x1B[H");
    io::stdout().flush()?;

    let display_interval = Duration::from_millis(8); // ~120 Hz display
    let mut next_display = Instant::now() + display_interval;

    while running.load(Ordering::SeqCst) {
        // Handle everything the acquisition thread delivered, waiting at most
        // until the next display refresh
        let first = match rx.recv_timeout(next_display.saturating_duration_since(Instant::now())) {
            Ok(message) => Some(message),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        for message in first.into_iter().chain(rx.try_iter()) {
            let result = match message {
                Acquired::Batch(result) => result,
                Acquired::Temperature(raw) => {
                    temperature = Some(raw);
                    continue;
                }
                Acquired::Error(e) => {
                    eprintln!("FIFO read error: {}", e);
                    continue;
                }
            };

            if result.overflow_detected {
                fifo_overflows += 1;
                // Integration needs gap-free data: restart the meter
                #[cfg(feature = "analysis")]
                vib_meter.reset();
            }
            let batch = result.samples;
            total_samples += batch.len() as u64;
            last_batch_size = batch.len();
            let data = batch.last().unwrap();
            let (ax, ay, az) = data.accel_to_g(range);
            last_ax = ax;
            last_ay = ay;
            last_az = az;

            // Feed all samples into analysis buffers
            #[cfg(feature = "analysis")]
            for sample in &batch {
                let (gx, gy, gz) = sample.accel_to_g(range);
                if buf_x.len() >= FFT_WINDOW { buf_x.pop_front(); }
                buf_x.push_back(gx);
                if buf_y.len() >= FFT_WINDOW { buf_y.pop_front(); }
                buf_y.push_back(gy);
                if buf_z.len() >= FFT_WINDOW { buf_z.pop_front(); }
                buf_z.push_back(gz);
                vib_meter.push([gx, gy, gz].map(|g| g as f64 * STANDARD_GRAVITY));
            }
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

            let temp_str = match temperature {
                Some(raw) => format!("{:.1}°C", ((raw as f32 - 1885.0) / -9.05) + 25.0),
                None => "N/A".to_string(),
            };

            // Compute analysis metrics
            #[cfg(feature = "analysis")]
            {
                // RMS: every display update (cheap)
                if !buf_x.is_empty() {
                    rms_x = compute_rms(buf_x.make_contiguous());
                    rms_y = compute_rms(buf_y.make_contiguous());
                    rms_z = compute_rms(buf_z.make_contiguous());
                }

                // FFT: every 500ms, only when buffer is full.
                // Samples are spaced by the ODR, not the measured (lossy) read rate.
                if now.duration_since(last_fft_time) >= fft_interval
                    && buf_x.len() >= FFT_WINDOW
                {
                    peaks_x = find_frequency_peaks(buf_x.make_contiguous(), odr_hz, FFT_WINDOW);
                    peaks_y = find_frequency_peaks(buf_y.make_contiguous(), odr_hz, FFT_WINDOW);
                    peaks_z = find_frequency_peaks(buf_z.make_contiguous(), odr_hz, FFT_WINDOW);
                    last_fft_time = now;
                }

                // Meter-style values over the rolling window: every 100ms
                if now.duration_since(last_vib_time) >= Duration::from_millis(100) {
                    vib = vib_meter.values();
                    last_vib_time = now;
                }
            }

            // Pad every line to a fixed width to overwrite stale content on refresh
            const W: usize = 68;
            let pad = |s: String| {
                if s.len() >= W { s } else { format!("{:<width$}", s, width = W) }
            };

            print!("\x1B[H");
            println!("{}", pad(format!("ADXL355 Sensor Reader - FIFO Mode ({} Hz ODR)", odr_hz)));
            println!("{}", pad("================================================".into()));
            println!("{}", pad(format!("Time: {:.2}s | Samples: {} | Rate: {:.0} Hz | Batch: {} | Overflows: {}",
                elapsed, total_samples, sample_rate, last_batch_size, fifo_overflows)));
            println!("{}", pad(format!("Temperature: {}", temp_str)));
            println!();
            println!("ACCELEROMETER (g)  [-2g ◄─────────────────┼─────────────────► +2g]");
            println!("  X: {:8.5}g  [{}]", last_ax, create_bar(last_ax, 2.0, 40));
            println!("  Y: {:8.5}g  [{}]", last_ay, create_bar(last_ay, 2.0, 40));
            println!("  Z: {:8.5}g  [{}]", last_az, create_bar(last_az, 2.0, 40));

            #[cfg(feature = "analysis")]
            {
                println!();
                println!("{}", pad("RMS (g, incl. gravity)".into()));
                if buf_x.is_empty() {
                    println!("{}", pad("  Collecting...".into()));
                } else {
                    let rms_total = (rms_x * rms_x + rms_y * rms_y + rms_z * rms_z).sqrt();
                    println!("{}", pad(format!(
                        "  X: {:.5}   Y: {:.5}   Z: {:.5}   Mag: {:.5}",
                        rms_x, rms_y, rms_z, rms_total)));
                }
                println!();
                println!("{}", pad("FREQUENCY PEAKS (Hz)".into()));
                if buf_x.len() < FFT_WINDOW {
                    println!("{}", pad(format!("  Buffering... ({}/{})", buf_x.len(), FFT_WINDOW)));
                    for _ in 0..3 { println!("{}", pad(String::new())); }
                } else {
                    for (label, peaks) in [("X", &peaks_x), ("Y", &peaks_y), ("Z", &peaks_z)] {
                        let peak_strs: Vec<String> = peaks.iter().take(3)
                            .map(|p| format!("{:>7.1}", p.frequency_hz))
                            .collect();
                        let display = if peak_strs.is_empty() {
                            "  (no peaks)".to_string()
                        } else {
                            peak_strs.join("  | ")
                        };
                        println!("{}", pad(format!("  {}:  {}", label, display)));
                    }
                    println!("{}", pad(format!(
                        "  [{}pt FFT @ {:.0}Hz, updated every 0.5s]",
                        FFT_WINDOW, odr_hz)));
                }

                println!();
                let band = vib_meter.band();
                println!("{}", pad(format!("VIBRATION (meter-style, {:.0}-{:.0} Hz, rolling {:.0}s)",
                    band.low_hz, band.velocity_high_hz, VIB_WINDOW_SECONDS)));
                match &vib {
                    Some(v) => {
                        let row = |label: &str, prec: usize, value: fn(&OverallVibration) -> f64| pad(format!(
                            "  {:<20} X: {:>8.*}  Y: {:>8.*}  Z: {:>8.*}",
                            label, prec, value(&v[0]), prec, value(&v[1]), prec, value(&v[2])));
                        println!("{}", row("Velocity RMS (mm/s)", 3, |a| a.velocity_rms));
                        println!("{}", row("Accel peak (m/s^2)", 4, |a| a.accel_peak));
                        println!("{}", row("Displ. P-P (um)", 2, |a| a.displacement_pp));
                        // A meter measures one direction at a time; show the highest axis
                        let (axis, worst) = ["X", "Y", "Z"].iter().zip(v.iter())
                            .max_by(|a, b| a.1.velocity_rms.total_cmp(&b.1.velocity_rms))
                            .unwrap();
                        println!("{}", pad(format!("  Overall velocity (highest axis): {:.3} mm/s RMS ({})",
                            worst.velocity_rms, axis)));
                    }
                    None => {
                        println!("{}", pad(format!("  Settling... ({:.0}%)", vib_meter.progress() * 100.0)));
                        for _ in 0..3 { println!("{}", pad(String::new())); }
                    }
                }
            }

            println!();
            println!("{}", pad("Press Ctrl+C to exit".into()));
            let _ = io::stdout().flush();
        }
    }

    println!("\nShutting down...");
    running.store(false, Ordering::SeqCst);
    drop(rx);
    let _ = acquisition.join();
    Ok(())
}

/// Acquisition thread: owns the sensor, polls the FIFO back to back and reads
/// the temperature (not in the FIFO) once per second
fn acquire(
    odr: OutputDataRate,
    running: Arc<AtomicBool>,
    init: mpsc::Sender<Result<Range, Adxl355Error>>,
    out: mpsc::Sender<Acquired>,
) {
    let mut sensor = match Adxl355::new(0) {
        Ok(sensor) => sensor,
        Err(e) => {
            let _ = init.send(Err(e));
            return;
        }
    };
    if let Err(e) = sensor.enable_fifo(odr) {
        let _ = init.send(Err(e));
        return;
    }
    let _ = init.send(Ok(sensor.get_range()));

    let mut last_temperature: Option<Instant> = None;
    while running.load(Ordering::SeqCst) {
        let message = match sensor.read_fifo_batch_checked() {
            // An overflow always comes with a full FIFO (read in the same transaction)
            Ok(result) if result.samples.is_empty() => None,
            Ok(result) => Some(Acquired::Batch(result)),
            Err(e) => Some(Acquired::Error(e.to_string())),
        };
        if let Some(message) = message {
            if out.send(message).is_err() {
                break;
            }
        }

        if last_temperature.map_or(true, |t| t.elapsed() >= Duration::from_secs(1)) {
            if let Ok(raw) = sensor.read_temperature() {
                let _ = out.send(Acquired::Temperature(raw));
            }
            last_temperature = Some(Instant::now());
        }
    }
}
