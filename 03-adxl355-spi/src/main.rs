//! ADXL355 sensor reader - Continuous data acquisition and display
//!
//! Uses FIFO mode for high-speed sampling over SPI (target: 4kHz).
//! Display updates at ~120 Hz, decoupled from read rate.

use ft232_adxl355_spi::{Adxl355, Adxl355Error, OutputDataRate, create_bar};
#[cfg(feature = "analysis")]
use ft232_adxl355_spi::analysis::{compute_rms, find_frequency_peaks, FrequencyPeak};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
#[cfg(feature = "analysis")]
use std::collections::VecDeque;

#[cfg(feature = "analysis")]
const FFT_WINDOW: usize = 2048;

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
                }
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

            // Compute analysis metrics
            #[cfg(feature = "analysis")]
            {
                // RMS: every display update (cheap)
                if !buf_x.is_empty() {
                    rms_x = compute_rms(buf_x.make_contiguous());
                    rms_y = compute_rms(buf_y.make_contiguous());
                    rms_z = compute_rms(buf_z.make_contiguous());
                }

                // FFT: every 500ms, only when buffer is full
                if now.duration_since(last_fft_time) >= fft_interval
                    && buf_x.len() >= FFT_WINDOW
                {
                    peaks_x = find_frequency_peaks(buf_x.make_contiguous(), sample_rate, FFT_WINDOW);
                    peaks_y = find_frequency_peaks(buf_y.make_contiguous(), sample_rate, FFT_WINDOW);
                    peaks_z = find_frequency_peaks(buf_z.make_contiguous(), sample_rate, FFT_WINDOW);
                    last_fft_time = now;
                }
            }

            // Pad every line to a fixed width to overwrite stale content on refresh
            const W: usize = 68;
            let pad = |s: String| {
                if s.len() >= W { s } else { format!("{:<width$}", s, width = W) }
            };

            print!("\x1B[H");
            println!("{}", pad(format!("ADXL355 Sensor Reader - FIFO Mode ({} Hz ODR)", odr.as_hz())));
            println!("{}", pad("================================================".into()));
            println!("{}", pad(format!("Time: {:.2}s | Samples: {} | Rate: {:.0} Hz | Batch: {}",
                elapsed, total_samples, sample_rate, last_batch_size)));
            println!("{}", pad(format!("Temperature: {}", temp_str)));
            println!();
            println!("ACCELEROMETER (g)  [-2g ◄─────────────────┼─────────────────► +2g]");
            println!("  X: {:8.5}g  [{}]", last_ax, create_bar(last_ax, 2.0, 40));
            println!("  Y: {:8.5}g  [{}]", last_ay, create_bar(last_ay, 2.0, 40));
            println!("  Z: {:8.5}g  [{}]", last_az, create_bar(last_az, 2.0, 40));

            #[cfg(feature = "analysis")]
            {
                println!();
                println!("{}", pad("RMS (g)".into()));
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
                        FFT_WINDOW, sample_rate)));
                }
            }

            println!();
            println!("{}", pad("Press Ctrl+C to exit".into()));
            let _ = io::stdout().flush();
        }
    }

    println!("\nShutting down...");
    Ok(())
}
