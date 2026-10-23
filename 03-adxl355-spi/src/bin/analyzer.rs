//! ADXL355 Data Analyzer
//!
//! Post-processing analysis tool for sensor data from HDF5 files.

use clap::Parser;
use ft232_adxl355_spi::{Hdf5Reader, Metadata, Range, TimestampedSample};
use ft232_adxl355_spi::analysis::{
    fft_segment_count, find_frequency_peaks, overall_vibration, vibration_band, SETTLE_SECONDS, STANDARD_GRAVITY, VELOCITY_BAND_HIGH_HZ,
};
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

    /// Meter-style vibration values (velocity RMS, accel peak, displacement P-P)
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
    let (origin, source) = metadata.time_origin();
    writeln!(output, "  Timestamps origin: {} ({})", origin, source)?;
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
        run_vibration_analysis(&mut output, &samples, sample_rate, metadata, range)?;
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
    mean: f64,
    rms: f64,
    std_dev: f64,
    min: f64,
    max: f64,
    peak_to_peak: f64,
}

// Accumulate in f64: f32 sums drift by several percent over long recordings
fn compute_stats(data: &[f32]) -> Stats {
    let n = data.len() as f64;
    let mean = data.iter().map(|&x| x as f64).sum::<f64>() / n;
    let rms = (data.iter().map(|&x| (x as f64).powi(2)).sum::<f64>() / n).sqrt();
    let variance = data.iter().map(|&x| (x as f64 - mean).powi(2)).sum::<f64>() / n;
    let std_dev = variance.sqrt();
    let min = data.iter().map(|&x| x as f64).fold(f64::INFINITY, f64::min);
    let max = data.iter().map(|&x| x as f64).fold(f64::NEG_INFINITY, f64::max);
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

    // Samples pinned at the 20-bit full-scale code are clipped (range too small)
    let clipped = |raw: fn(&TimestampedSample) -> i32| {
        samples.iter().filter(|s| { let v = raw(s); v >= 524_287 || v <= -524_288 }).count()
    };
    let (cx, cy, cz) = (clipped(|s| s.data.accel_x), clipped(|s| s.data.accel_y), clipped(|s| s.data.accel_z));
    writeln!(output, "Clipped samples (at full scale): X {}, Y {}, Z {} of {} ({:.4}% max)",
        cx, cy, cz, samples.len(), cx.max(cy).max(cz) as f64 / samples.len() as f64 * 100.0)?;

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
    writeln!(output, "  Window type: Hann, mean removed per segment")?;
    writeln!(output, "  Averaging: Welch, 50% overlap, {} segments over the analysis range",
        fft_segment_count(samples.len(), WINDOW_SIZE))?;
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
                writeln!(output, "  {}. {:.2} Hz (amplitude: {:.6} g)",
                    i + 1, peak.frequency_hz, peak.magnitude)?;
            }
        }
    }

    Ok(())
}

// ============================================================================
// VIBRATION
// ============================================================================

fn run_vibration_analysis(output: &mut dyn Write, samples: &[TimestampedSample], measured_rate: f64, metadata: &Metadata, range: Range) -> io::Result<()> {
    let odr = metadata.sample_rate_hz;
    // FIFO samples are spaced by the sensor clock; polling samples by the poll interval
    let sample_rate = if metadata.acquisition_mode == "fifo" { odr } else { measured_rate };
    let sensor_bandwidth = odr / 4.0; // ADXL355 digital LPF corner
    let band = vibration_band(sample_rate, sensor_bandwidth);

    writeln!(output)?;
    writeln!(output, "Overall values as reported by handheld vibration meters:")?;
    writeln!(output, "  Sample rate used: {:.1} Hz", sample_rate)?;
    writeln!(output, "  Accel / velocity band: {:.1} - {:.1} Hz (meter: 10 - 1000 Hz)", band.low_hz, band.velocity_high_hz)?;
    writeln!(output, "  Displacement band:     {:.1} - {:.1} Hz (meter: 10 - 500 Hz)", band.low_hz, band.displacement_high_hz)?;
    if band.velocity_high_hz < VELOCITY_BAND_HIGH_HZ {
        writeln!(output, "  Note: upper edge capped by ODR/4 (sensor LPF) or fs/4; vibration above {:.0} Hz is not measured,", band.velocity_high_hz)?;
        writeln!(output, "        so readings can be lower than a 10 - 1000 Hz meter. Use a higher ODR to widen the band.")?;
    }
    writeln!(output, "  First {:.1}s excluded (filter settling)", SETTLE_SECONDS)?;
    if metadata.acquisition_mode == "fifo" && measured_rate < odr * 0.97 {
        writeln!(output, "  WARNING: ~{:.0}% of samples missing (FIFO overflow). Integration assumes no gaps,",
            (1.0 - measured_rate / odr) * 100.0)?;
        writeln!(output, "           so velocity and displacement are unreliable. Record at a lower ODR.")?;
    }
    writeln!(output)?;

    let mut results = Vec::new();
    for (axis, index) in [("X", 0), ("Y", 1), ("Z", 2)] {
        let accel: Vec<f64> = samples.iter()
            .map(|s| {
                let g = s.data.accel_to_g(range);
                [g.0, g.1, g.2][index] as f64 * STANDARD_GRAVITY
            })
            .collect();
        match overall_vibration(&accel, sample_rate, sensor_bandwidth) {
            Some(v) => results.push((axis, v)),
            None => {
                writeln!(output, "Insufficient data: need at least {:.1}s of samples and ODR >= 62.5 Hz",
                    SETTLE_SECONDS + 1.0)?;
                return Ok(());
            }
        }
    }

    writeln!(output, "{:<6} {:>12} {:>12} {:>14} {:>12}", "Axis", "Accel RMS", "Accel Peak", "Velocity RMS", "Displ. P-P")?;
    writeln!(output, "{:<6} {:>12} {:>12} {:>14} {:>12}", "", "(m/s^2)", "(m/s^2)", "(mm/s)", "(um)")?;
    writeln!(output, "{:-<60}", "")?;
    for (axis, v) in &results {
        writeln!(output, "{:<6} {:>12.4} {:>12.4} {:>14.3} {:>12.2}",
            axis, v.accel_rms, v.accel_peak, v.velocity_rms, v.displacement_pp)?;
    }
    writeln!(output, "Peak and P-P are true extremes over the analysis window (they grow with window length).")?;

    // A meter measures one direction at a time; report the highest axis
    let (axis, worst) = results.iter()
        .max_by(|a, b| a.1.velocity_rms.total_cmp(&b.1.velocity_rms))
        .unwrap();
    writeln!(output)?;
    writeln!(output, "Overall velocity (highest axis): {:.3} mm/s RMS ({})", worst.velocity_rms, axis)?;

    Ok(())
}
