//! ADXL355 Data Analyzer
//!
//! Post-processing analysis tool for sensor data from HDF5 files.

use clap::Parser;
use ft232_adxl355_spi::{Hdf5Reader, Range, TimestampedSample};
use ft232_adxl355_spi::analysis::{compute_rms, find_frequency_peaks};
use std::f64::consts::PI;
use std::fs::File;
use std::io::{self, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "analyzer")]
#[command(about = "Analyze ADXL355 sensor data from HDF5 file")]
struct Args {
    /// Input HDF5 file path
    #[arg(short, long)]
    input: PathBuf,

    /// Start time in seconds
    #[arg(long)]
    start: Option<f64>,

    /// End time in seconds
    #[arg(long)]
    end: Option<f64>,

    /// Perform FFT frequency analysis
    #[arg(long)]
    fft: bool,

    /// Compute statistical metrics
    #[arg(long)]
    statistics: bool,

    /// Compute vibration metrics (RMS, velocity, displacement)
    #[arg(long)]
    vibration: bool,

    /// Run all analyses
    #[arg(long)]
    all: bool,

    /// Output file (default: stdout)
    #[arg(short, long)]
    output: Option<PathBuf>,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let reader = Hdf5Reader::open(&args.input)?;
    let metadata = reader.metadata();

    let range = match metadata.range.as_str() {
        "4g" => Range::G4,
        "8g" => Range::G8,
        _ => Range::G2,
    };

    // Default to all analyses if none specified
    let run_all = args.all || (!args.statistics && !args.fft && !args.vibration);
    let run_statistics = run_all || args.statistics;
    let run_fft = run_all || args.fft;
    let run_vibration = run_all || args.vibration;

    println!("Loading data from {}...", args.input.display());
    let total_samples = reader.get_total_samples()?;

    if total_samples == 0 {
        eprintln!("Error: No samples in file");
        return Ok(());
    }

    let all_samples = reader.read_range(0, total_samples)?;
    let file_start = all_samples.first().unwrap().timestamp;
    let file_end = all_samples.last().unwrap().timestamp;

    let start_time = args.start.unwrap_or(file_start);
    let end_time = args.end.unwrap_or(file_end);

    if start_time >= end_time {
        eprintln!("Error: Start time must be before end time");
        std::process::exit(1);
    }

    let samples: Vec<TimestampedSample> = all_samples.into_iter()
        .filter(|s| s.timestamp >= start_time && s.timestamp <= end_time)
        .collect();

    if samples.is_empty() {
        eprintln!("Error: No samples in specified time range");
        return Ok(());
    }

    // Compute actual sample rate from timestamps (more accurate than metadata ODR)
    let sample_rate = if samples.len() >= 2 {
        let first_ts = samples.first().unwrap().timestamp;
        let last_ts = samples.last().unwrap().timestamp;
        let duration = last_ts - first_ts;
        if duration > 0.0 {
            (samples.len() - 1) as f64 / duration
        } else {
            metadata.sample_rate_hz
        }
    } else {
        metadata.sample_rate_hz
    };

    println!("Loaded {} samples ({:.2}s to {:.2}s)",
        samples.len(), samples.first().unwrap().timestamp, samples.last().unwrap().timestamp);
    println!("Measured sample rate: {:.1} Hz (configured ODR: {:.1} Hz)",
        sample_rate, metadata.sample_rate_hz);

    let mut output: Box<dyn Write> = if let Some(path) = args.output {
        Box::new(File::create(path)?)
    } else {
        Box::new(io::stdout())
    };

    // Header
    writeln!(output, "{}", "=".repeat(80))?;
    writeln!(output, "ADXL355 DATA ANALYSIS REPORT")?;
    writeln!(output, "{}", "=".repeat(80))?;
    writeln!(output)?;
    writeln!(output, "File Information:")?;
    writeln!(output, "  Sensor: {}", metadata.sensor_type)?;
    writeln!(output, "  Range: {}", metadata.range)?;
    writeln!(output, "  Acquisition mode: {}", metadata.acquisition_mode)?;
    writeln!(output, "  Configured ODR: {:.1} Hz", metadata.sample_rate_hz)?;
    writeln!(output, "  Measured sample rate: {:.1} Hz", sample_rate)?;
    writeln!(output, "  Start time: {}", metadata.start_time)?;
    writeln!(output)?;
    writeln!(output, "Analysis Range:")?;
    writeln!(output, "  Start: {:.2}s", start_time)?;
    writeln!(output, "  End: {:.2}s", end_time)?;
    writeln!(output, "  Duration: {:.2}s", end_time - start_time)?;
    writeln!(output, "  Samples: {}", samples.len())?;

    if run_statistics {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "STATISTICAL ANALYSIS")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_statistics_analysis(&mut output, &samples, range)?;
    }

    if run_fft {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "FREQUENCY ANALYSIS (FFT)")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_fft_analysis(&mut output, &samples, sample_rate, range)?;
    }

    if run_vibration {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "VIBRATION ANALYSIS")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_vibration_analysis(&mut output, &samples, sample_rate, range)?;
    }

    writeln!(output, "\n{}", "=".repeat(80))?;
    writeln!(output, "Analysis complete!")?;

    Ok(())
}

// ============================================================================
// STATISTICS
// ============================================================================

#[derive(Debug)]
struct Stats {
    mean: f32,
    rms: f32,
    std_dev: f32,
    min: f32,
    max: f32,
    peak_to_peak: f32,
}

fn compute_stats(data: &[f32]) -> Stats {
    let n = data.len() as f32;
    let mean = data.iter().sum::<f32>() / n;
    let rms = (data.iter().map(|&x| x * x).sum::<f32>() / n).sqrt();
    let variance = data.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n;
    let std_dev = variance.sqrt();
    let min = data.iter().copied().fold(f32::INFINITY, f32::min);
    let max = data.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let peak_to_peak = max - min;
    Stats { mean, rms, std_dev, min, max, peak_to_peak }
}

fn run_statistics_analysis(output: &mut dyn Write, samples: &[TimestampedSample], range: Range) -> io::Result<()> {
    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).0).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).1).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).2).collect();

    let stats_ax = compute_stats(&accel_x);
    let stats_ay = compute_stats(&accel_y);
    let stats_az = compute_stats(&accel_z);

    writeln!(output)?;
    writeln!(output, "Accelerometer Statistics (g):")?;
    writeln!(output, "{:-<80}", "")?;
    writeln!(output, "{:<10} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}",
        "Axis", "Mean", "RMS", "Std Dev", "Min", "Max", "Peak-Peak")?;
    writeln!(output, "{:-<80}", "")?;

    for (axis, stats) in [("X", &stats_ax), ("Y", &stats_ay), ("Z", &stats_az)] {
        writeln!(output, "{:<10} {:>12.6} {:>12.6} {:>12.6} {:>12.6} {:>12.6} {:>12.6}",
            axis, stats.mean, stats.rms, stats.std_dev, stats.min, stats.max, stats.peak_to_peak)?;
    }

    Ok(())
}

// ============================================================================
// FFT
// ============================================================================

fn run_fft_analysis(output: &mut dyn Write, samples: &[TimestampedSample], sample_rate: f64, range: Range) -> io::Result<()> {
    const WINDOW_SIZE: usize = 2048;

    writeln!(output)?;
    writeln!(output, "FFT Parameters:")?;
    writeln!(output, "  Window size: {} samples", WINDOW_SIZE)?;
    writeln!(output, "  Window type: Hann")?;
    writeln!(output, "  Frequency resolution: {:.2} Hz", sample_rate / WINDOW_SIZE as f64)?;
    writeln!(output, "  Max frequency: {:.1} Hz", sample_rate / 2.0)?;
    writeln!(output)?;

    if samples.len() < WINDOW_SIZE {
        writeln!(output, "Warning: Insufficient samples for FFT (need {}, have {})",
            WINDOW_SIZE, samples.len())?;
        return Ok(());
    }

    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).0).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).1).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).2).collect();

    writeln!(output, "Accelerometer Frequency Analysis:")?;
    writeln!(output, "{:-<80}", "")?;

    for (label, data) in [("Accel X", &accel_x), ("Accel Y", &accel_y), ("Accel Z", &accel_z)] {
        let peaks = find_frequency_peaks(data, sample_rate, WINDOW_SIZE);
        writeln!(output, "\n{} - Top 5 Frequency Peaks:", label)?;
        if peaks.is_empty() {
            writeln!(output, "  No significant peaks detected")?;
        } else {
            for (i, peak) in peaks.iter().take(5).enumerate() {
                writeln!(output, "  {}. {:.2} Hz (magnitude: {:.6})",
                    i + 1, peak.frequency_hz, peak.magnitude)?;
            }
        }
    }

    Ok(())
}

// ============================================================================
// VIBRATION
// ============================================================================

fn high_pass_filter(data: &[f32], cutoff_hz: f64, sample_rate: f64) -> Vec<f32> {
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let dt = 1.0 / sample_rate;
    let alpha = rc / (rc + dt);

    let mut filtered = Vec::with_capacity(data.len());
    let mut prev_input = data[0] as f64;
    let mut prev_output = 0.0_f64;

    for &input in data {
        let input_f64 = input as f64;
        let output = alpha * (prev_output + input_f64 - prev_input);
        filtered.push(output as f32);
        prev_input = input_f64;
        prev_output = output;
    }

    filtered
}

fn integrate_trapezoidal(data: &[f32], dt: f64) -> Vec<f32> {
    let mut integrated = Vec::with_capacity(data.len());
    let mut accumulator = 0.0;

    integrated.push(0.0);

    for i in 1..data.len() {
        accumulator += ((data[i] + data[i - 1]) as f64 / 2.0) * dt;
        integrated.push(accumulator as f32);
    }

    integrated
}

fn run_vibration_analysis(output: &mut dyn Write, samples: &[TimestampedSample], sample_rate: f64, range: Range) -> io::Result<()> {
    let dt = 1.0 / sample_rate;

    writeln!(output)?;
    writeln!(output, "Vibration Analysis Parameters:")?;
    writeln!(output, "  Sample rate: {:.1} Hz", sample_rate)?;
    writeln!(output, "  Time step (dt): {:.6} s", dt)?;
    writeln!(output, "  High-pass filter cutoff: 0.5 Hz")?;
    writeln!(output)?;

    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).0).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).1).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_to_g(range).2).collect();

    // RMS acceleration
    let rms_x = compute_rms(&accel_x);
    let rms_y = compute_rms(&accel_y);
    let rms_z = compute_rms(&accel_z);
    let rms_total = (rms_x * rms_x + rms_y * rms_y + rms_z * rms_z).sqrt();

    writeln!(output, "RMS Acceleration (g):")?;
    writeln!(output, "  X: {:.6}g", rms_x)?;
    writeln!(output, "  Y: {:.6}g", rms_y)?;
    writeln!(output, "  Z: {:.6}g", rms_z)?;
    writeln!(output, "  Total: {:.6}g", rms_total)?;
    writeln!(output)?;

    // Remove DC offset (gravity) before filtering and integration
    let n = accel_x.len() as f32;
    let mean_x: f32 = accel_x.iter().sum::<f32>() / n;
    let mean_y: f32 = accel_y.iter().sum::<f32>() / n;
    let mean_z: f32 = accel_z.iter().sum::<f32>() / n;

    let accel_x_ac: Vec<f32> = accel_x.iter().map(|&a| a - mean_x).collect();
    let accel_y_ac: Vec<f32> = accel_y.iter().map(|&a| a - mean_y).collect();
    let accel_z_ac: Vec<f32> = accel_z.iter().map(|&a| a - mean_z).collect();

    // High-pass filter and integrate to velocity
    const G_TO_MS2: f64 = 9.81;
    let accel_x_filtered = high_pass_filter(&accel_x_ac, 0.5, sample_rate);
    let accel_y_filtered = high_pass_filter(&accel_y_ac, 0.5, sample_rate);
    let accel_z_filtered = high_pass_filter(&accel_z_ac, 0.5, sample_rate);

    let accel_x_ms2: Vec<f32> = accel_x_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();
    let accel_y_ms2: Vec<f32> = accel_y_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();
    let accel_z_ms2: Vec<f32> = accel_z_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();

    let velocity_x = integrate_trapezoidal(&accel_x_ms2, dt);
    let velocity_y = integrate_trapezoidal(&accel_y_ms2, dt);
    let velocity_z = integrate_trapezoidal(&accel_z_ms2, dt);

    let rms_vel_x = compute_rms(&velocity_x);
    let rms_vel_y = compute_rms(&velocity_y);
    let rms_vel_z = compute_rms(&velocity_z);
    let rms_vel_total = (rms_vel_x * rms_vel_x + rms_vel_y * rms_vel_y + rms_vel_z * rms_vel_z).sqrt();

    writeln!(output, "RMS Velocity (m/s):")?;
    writeln!(output, "  X: {:.6} m/s", rms_vel_x)?;
    writeln!(output, "  Y: {:.6} m/s", rms_vel_y)?;
    writeln!(output, "  Z: {:.6} m/s", rms_vel_z)?;
    writeln!(output, "  Total: {:.6} m/s", rms_vel_total)?;
    writeln!(output)?;

    // Integrate velocity to displacement
    let displacement_x = integrate_trapezoidal(&velocity_x, dt);
    let displacement_y = integrate_trapezoidal(&velocity_y, dt);
    let displacement_z = integrate_trapezoidal(&velocity_z, dt);

    let rms_disp_x = compute_rms(&displacement_x);
    let rms_disp_y = compute_rms(&displacement_y);
    let rms_disp_z = compute_rms(&displacement_z);
    let rms_disp_total = (rms_disp_x * rms_disp_x + rms_disp_y * rms_disp_y + rms_disp_z * rms_disp_z).sqrt();

    writeln!(output, "RMS Displacement (m):")?;
    writeln!(output, "  X: {:.6} m ({:.3} mm)", rms_disp_x, rms_disp_x * 1000.0)?;
    writeln!(output, "  Y: {:.6} m ({:.3} mm)", rms_disp_y, rms_disp_y * 1000.0)?;
    writeln!(output, "  Z: {:.6} m ({:.3} mm)", rms_disp_z, rms_disp_z * 1000.0)?;
    writeln!(output, "  Total: {:.6} m ({:.3} mm)", rms_disp_total, rms_disp_total * 1000.0)?;
    writeln!(output)?;

    // Peak values
    let peak_vel_x = velocity_x.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));
    let peak_vel_y = velocity_y.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));
    let peak_vel_z = velocity_z.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));

    let peak_disp_x = displacement_x.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));
    let peak_disp_y = displacement_y.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));
    let peak_disp_z = displacement_z.iter().copied().fold(0.0f32, |a, b| a.max(b.abs()));

    writeln!(output, "Peak Velocity (m/s):")?;
    writeln!(output, "  X: {:.6} m/s", peak_vel_x)?;
    writeln!(output, "  Y: {:.6} m/s", peak_vel_y)?;
    writeln!(output, "  Z: {:.6} m/s", peak_vel_z)?;
    writeln!(output)?;

    writeln!(output, "Peak Displacement (m):")?;
    writeln!(output, "  X: {:.6} m ({:.3} mm)", peak_disp_x, peak_disp_x * 1000.0)?;
    writeln!(output, "  Y: {:.6} m ({:.3} mm)", peak_disp_y, peak_disp_y * 1000.0)?;
    writeln!(output, "  Z: {:.6} m ({:.3} mm)", peak_disp_z, peak_disp_z * 1000.0)?;

    Ok(())
}
