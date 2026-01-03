//! MPU6050 Data Analyzer
//!
//! Post-processing analysis tool for sensor data from HDF5 files.
//!
//! Usage:
//!   analyzer --input data.h5 --all
//!   analyzer --input data.h5 --fft --statistics
//!   analyzer --input data.h5 --start 5.0 --end 10.0 --fft

use clap::Parser;
use ft232_sensor_interface::{Hdf5Reader, TimestampedSample};
use num_complex::Complex;
use rustfft::FftPlanner;
use std::f64::consts::PI;
use std::fs::File;
use std::io::{self, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "analyzer")]
#[command(about = "Analyze MPU6050 sensor data from HDF5 file", long_about = None)]
struct Args {
    /// Input HDF5 file path
    #[arg(short, long)]
    input: PathBuf,

    /// Start time in seconds (optional, default: file start)
    #[arg(long)]
    start: Option<f64>,

    /// End time in seconds (optional, default: file end)
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Open HDF5 file
    let reader = Hdf5Reader::open(&args.input)?;
    let metadata = reader.metadata();

    // Determine analyses to run
    let run_statistics = args.all || args.statistics;
    let run_fft = args.all || args.fft;
    let run_vibration = args.all || args.vibration;

    if !run_statistics && !run_fft && !run_vibration {
        eprintln!("Error: Must specify at least one analysis type (--statistics, --fft, --vibration, or --all)");
        std::process::exit(1);
    }

    // Load data
    println!("Loading data from {}...", args.input.display());
    let total_samples = reader.get_total_samples()?;

    if total_samples == 0 {
        eprintln!("Error: No samples in file");
        return Ok(());
    }

    // Determine time range
    let all_samples = reader.read_range(0, total_samples)?;
    let file_start = all_samples.first().unwrap().timestamp;
    let file_end = all_samples.last().unwrap().timestamp;

    let start_time = args.start.unwrap_or(file_start);
    let end_time = args.end.unwrap_or(file_end);

    // Validate time range
    if start_time < file_start || end_time > file_end {
        eprintln!("Warning: Requested time range [{}, {}] extends beyond file range [{}, {}]",
            start_time, end_time, file_start, file_end);
    }

    if start_time >= end_time {
        eprintln!("Error: Start time must be before end time");
        std::process::exit(1);
    }

    // Filter samples by time range
    let samples: Vec<TimestampedSample> = all_samples.into_iter()
        .filter(|s| s.timestamp >= start_time && s.timestamp <= end_time)
        .collect();

    if samples.is_empty() {
        eprintln!("Error: No samples in specified time range");
        return Ok(());
    }

    println!("Loaded {} samples ({:.2}s to {:.2}s)",
        samples.len(), samples.first().unwrap().timestamp, samples.last().unwrap().timestamp);

    // Open output
    let mut output: Box<dyn Write> = if let Some(path) = args.output {
        Box::new(File::create(path)?)
    } else {
        Box::new(io::stdout())
    };

    // Write header
    write_header(&mut output, &metadata, &samples, start_time, end_time)?;

    // Run analyses
    if run_statistics {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "STATISTICAL ANALYSIS")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_statistics_analysis(&mut output, &samples)?;
    }

    if run_fft {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "FREQUENCY ANALYSIS (FFT)")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_fft_analysis(&mut output, &samples, metadata.sample_rate_hz)?;
    }

    if run_vibration {
        writeln!(output, "\n{}", "=".repeat(80))?;
        writeln!(output, "VIBRATION ANALYSIS")?;
        writeln!(output, "{}", "=".repeat(80))?;
        run_vibration_analysis(&mut output, &samples, metadata.sample_rate_hz)?;
    }

    writeln!(output, "\n{}", "=".repeat(80))?;
    writeln!(output, "Analysis complete!")?;

    Ok(())
}

fn write_header(
    output: &mut dyn Write,
    metadata: &ft232_sensor_interface::Metadata,
    samples: &[TimestampedSample],
    start_time: f64,
    end_time: f64,
) -> io::Result<()> {
    writeln!(output, "{}", "=".repeat(80))?;
    writeln!(output, "MPU6050 DATA ANALYSIS REPORT")?;
    writeln!(output, "{}", "=".repeat(80))?;
    writeln!(output)?;
    writeln!(output, "File Information:")?;
    writeln!(output, "  Acquisition mode: {}", metadata.acquisition_mode)?;
    writeln!(output, "  Sample rate: {:.1} Hz", metadata.sample_rate_hz)?;
    writeln!(output, "  Start time: {}", metadata.start_time)?;
    writeln!(output)?;
    writeln!(output, "Analysis Range:")?;
    writeln!(output, "  Start: {:.2}s", start_time)?;
    writeln!(output, "  End: {:.2}s", end_time)?;
    writeln!(output, "  Duration: {:.2}s", end_time - start_time)?;
    writeln!(output, "  Samples: {}", samples.len())?;
    Ok(())
}

// ============================================================================
// STATISTICS ANALYSIS
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

    // Mean
    let mean = data.iter().sum::<f32>() / n;

    // RMS
    let rms = (data.iter().map(|&x| x * x).sum::<f32>() / n).sqrt();

    // Standard deviation
    let variance = data.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n;
    let std_dev = variance.sqrt();

    // Min/Max
    let min = data.iter().copied().fold(f32::INFINITY, f32::min);
    let max = data.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let peak_to_peak = max - min;

    Stats { mean, rms, std_dev, min, max, peak_to_peak }
}

fn run_statistics_analysis(
    output: &mut dyn Write,
    samples: &[TimestampedSample],
) -> io::Result<()> {
    // Extract data arrays
    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_x_g()).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_y_g()).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_z_g()).collect();
    let gyro_x: Vec<f32> = samples.iter().map(|s| s.data.gyro_x_dps()).collect();
    let gyro_y: Vec<f32> = samples.iter().map(|s| s.data.gyro_y_dps()).collect();
    let gyro_z: Vec<f32> = samples.iter().map(|s| s.data.gyro_z_dps()).collect();

    // Compute statistics
    let stats_ax = compute_stats(&accel_x);
    let stats_ay = compute_stats(&accel_y);
    let stats_az = compute_stats(&accel_z);
    let stats_gx = compute_stats(&gyro_x);
    let stats_gy = compute_stats(&gyro_y);
    let stats_gz = compute_stats(&gyro_z);

    // Output accelerometer statistics
    writeln!(output)?;
    writeln!(output, "Accelerometer Statistics (g):")?;
    writeln!(output, "{:-<80}", "")?;
    writeln!(output, "{:<10} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}",
        "Axis", "Mean", "RMS", "Std Dev", "Min", "Max", "Peak-Peak")?;
    writeln!(output, "{:-<80}", "")?;

    write_stats_row(output, "X", &stats_ax)?;
    write_stats_row(output, "Y", &stats_ay)?;
    write_stats_row(output, "Z", &stats_az)?;

    // Output gyroscope statistics
    writeln!(output)?;
    writeln!(output, "Gyroscope Statistics (°/s):")?;
    writeln!(output, "{:-<80}", "")?;
    writeln!(output, "{:<10} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}",
        "Axis", "Mean", "RMS", "Std Dev", "Min", "Max", "Peak-Peak")?;
    writeln!(output, "{:-<80}", "")?;

    write_stats_row(output, "X", &stats_gx)?;
    write_stats_row(output, "Y", &stats_gy)?;
    write_stats_row(output, "Z", &stats_gz)?;

    Ok(())
}

fn write_stats_row(output: &mut dyn Write, axis: &str, stats: &Stats) -> io::Result<()> {
    writeln!(output, "{:<10} {:>12.4} {:>12.4} {:>12.4} {:>12.4} {:>12.4} {:>12.4}",
        axis, stats.mean, stats.rms, stats.std_dev, stats.min, stats.max, stats.peak_to_peak)
}

// ============================================================================
// FFT ANALYSIS
// ============================================================================

struct FrequencyPeak {
    frequency: f64,
    magnitude: f64,
}

fn apply_hann_window(data: &[f32]) -> Vec<f64> {
    let n = data.len();
    data.iter()
        .enumerate()
        .map(|(i, &x)| {
            let window = 0.5 * (1.0 - ((2.0 * PI * i as f64) / (n as f64 - 1.0)).cos());
            x as f64 * window
        })
        .collect()
}

fn analyze_frequencies(data: &[f32], sample_rate: f64, window_size: usize) -> Vec<FrequencyPeak> {
    if data.len() < window_size {
        return Vec::new();
    }

    // Take first window_size samples and apply Hann window
    let windowed: Vec<f64> = apply_hann_window(&data[..window_size]);

    // Convert to complex numbers
    let mut buffer: Vec<Complex<f64>> = windowed.iter()
        .map(|&x| Complex::new(x, 0.0))
        .collect();

    // Perform FFT
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(window_size);
    fft.process(&mut buffer);

    // Compute magnitude spectrum (only first half, as second half is mirror)
    let magnitudes: Vec<f64> = buffer.iter()
        .take(window_size / 2)
        .map(|c| c.norm() / (window_size as f64))
        .collect();

    // Find peaks (local maxima above threshold)
    let threshold = magnitudes.iter().copied().fold(0.0, f64::max) * 0.1; // 10% of max
    let mut peaks: Vec<FrequencyPeak> = Vec::new();

    for i in 1..magnitudes.len() - 1 {
        if magnitudes[i] > threshold
            && magnitudes[i] > magnitudes[i - 1]
            && magnitudes[i] > magnitudes[i + 1] {
            let frequency = (i as f64 * sample_rate) / window_size as f64;
            peaks.push(FrequencyPeak {
                frequency,
                magnitude: magnitudes[i],
            });
        }
    }

    // Sort by magnitude (descending)
    peaks.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap());

    peaks
}

fn run_fft_analysis(
    output: &mut dyn Write,
    samples: &[TimestampedSample],
    sample_rate: f64,
) -> io::Result<()> {
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

    // Extract data arrays
    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_x_g()).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_y_g()).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_z_g()).collect();
    let gyro_x: Vec<f32> = samples.iter().map(|s| s.data.gyro_x_dps()).collect();
    let gyro_y: Vec<f32> = samples.iter().map(|s| s.data.gyro_y_dps()).collect();
    let gyro_z: Vec<f32> = samples.iter().map(|s| s.data.gyro_z_dps()).collect();

    // Analyze each axis
    writeln!(output, "Accelerometer Frequency Analysis:")?;
    writeln!(output, "{:-<80}", "")?;
    analyze_and_print_peaks(output, "Accel X", &accel_x, sample_rate, WINDOW_SIZE)?;
    analyze_and_print_peaks(output, "Accel Y", &accel_y, sample_rate, WINDOW_SIZE)?;
    analyze_and_print_peaks(output, "Accel Z", &accel_z, sample_rate, WINDOW_SIZE)?;

    writeln!(output)?;
    writeln!(output, "Gyroscope Frequency Analysis:")?;
    writeln!(output, "{:-<80}", "")?;
    analyze_and_print_peaks(output, "Gyro X", &gyro_x, sample_rate, WINDOW_SIZE)?;
    analyze_and_print_peaks(output, "Gyro Y", &gyro_y, sample_rate, WINDOW_SIZE)?;
    analyze_and_print_peaks(output, "Gyro Z", &gyro_z, sample_rate, WINDOW_SIZE)?;

    Ok(())
}

fn analyze_and_print_peaks(
    output: &mut dyn Write,
    label: &str,
    data: &[f32],
    sample_rate: f64,
    window_size: usize,
) -> io::Result<()> {
    let peaks = analyze_frequencies(data, sample_rate, window_size);

    writeln!(output, "\n{} - Top 5 Frequency Peaks:", label)?;
    if peaks.is_empty() {
        writeln!(output, "  No significant peaks detected")?;
    } else {
        for (i, peak) in peaks.iter().take(5).enumerate() {
            writeln!(output, "  {}. {:.2} Hz (magnitude: {:.4})",
                i + 1, peak.frequency, peak.magnitude)?;
        }
    }

    Ok(())
}

// ============================================================================
// VIBRATION ANALYSIS
// ============================================================================

fn compute_rms(data: &[f32]) -> f32 {
    let sum_squares: f32 = data.iter().map(|&x| x * x).sum();
    (sum_squares / data.len() as f32).sqrt()
}

fn high_pass_filter(data: &[f32], cutoff_hz: f64, sample_rate: f64) -> Vec<f32> {
    // Simple first-order high-pass filter (removes DC offset)
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

    integrated.push(0.0); // Initial value

    for i in 1..data.len() {
        // Trapezoidal rule: (y[i] + y[i-1]) / 2 * dt
        accumulator += ((data[i] + data[i - 1]) as f64 / 2.0) * dt;
        integrated.push(accumulator as f32);
    }

    integrated
}

fn run_vibration_analysis(
    output: &mut dyn Write,
    samples: &[TimestampedSample],
    sample_rate: f64,
) -> io::Result<()> {
    let dt = 1.0 / sample_rate;

    writeln!(output)?;
    writeln!(output, "Vibration Analysis Parameters:")?;
    writeln!(output, "  Sample rate: {:.1} Hz", sample_rate)?;
    writeln!(output, "  Time step (dt): {:.6} s", dt)?;
    writeln!(output, "  High-pass filter cutoff: 0.5 Hz")?;
    writeln!(output)?;

    // Extract accelerometer data (only acceleration is relevant for vibration)
    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_x_g()).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_y_g()).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_z_g()).collect();

    // Compute RMS acceleration
    let rms_x = compute_rms(&accel_x);
    let rms_y = compute_rms(&accel_y);
    let rms_z = compute_rms(&accel_z);
    let rms_total = (rms_x * rms_x + rms_y * rms_y + rms_z * rms_z).sqrt();

    writeln!(output, "RMS Acceleration (g):")?;
    writeln!(output, "  X: {:.4}g", rms_x)?;
    writeln!(output, "  Y: {:.4}g", rms_y)?;
    writeln!(output, "  Z: {:.4}g", rms_z)?;
    writeln!(output, "  Total: {:.4}g", rms_total)?;
    writeln!(output)?;

    // Apply high-pass filter to remove DC offset before integration
    let accel_x_filtered = high_pass_filter(&accel_x, 0.5, sample_rate);
    let accel_y_filtered = high_pass_filter(&accel_y, 0.5, sample_rate);
    let accel_z_filtered = high_pass_filter(&accel_z, 0.5, sample_rate);

    // Integrate to velocity (m/s)
    // Note: 1g = 9.81 m/s^2
    const G_TO_MS2: f64 = 9.81;
    let accel_x_ms2: Vec<f32> = accel_x_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();
    let accel_y_ms2: Vec<f32> = accel_y_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();
    let accel_z_ms2: Vec<f32> = accel_z_filtered.iter().map(|&a| a * G_TO_MS2 as f32).collect();

    let velocity_x = integrate_trapezoidal(&accel_x_ms2, dt);
    let velocity_y = integrate_trapezoidal(&accel_y_ms2, dt);
    let velocity_z = integrate_trapezoidal(&accel_z_ms2, dt);

    // Compute RMS velocity
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

    // Integrate velocity to displacement (m)
    let displacement_x = integrate_trapezoidal(&velocity_x, dt);
    let displacement_y = integrate_trapezoidal(&velocity_y, dt);
    let displacement_z = integrate_trapezoidal(&velocity_z, dt);

    // Compute RMS displacement
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
