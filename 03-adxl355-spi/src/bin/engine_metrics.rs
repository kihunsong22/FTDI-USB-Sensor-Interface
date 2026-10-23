//! Engine vibration metrics for one or more recordings, one CSV row per segment
//!
//! Steady segments (constant rpm) get order, band and meter-style values;
//! deceleration segments (rpm falling through --decel-hi then --decel-lo) get
//! the deceleration rate. Engine speed comes from the vibration itself.

use chrono::{DateTime, Duration};
use clap::Parser;
use ft232_adxl355_spi::engine_metrics::{
    decel_segments, steady_metrics, steady_runs, track_rpm, MetricsConfig, Recording, BANDS, METRICS_VERSION, ORDERS,
};
use ft232_adxl355_spi::{Hdf5Reader, Range};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "engine-metrics")]
#[command(about = "Engine vibration metrics per steady and deceleration segment (CSV)")]
struct Args {
    /// Input HDF5 files
    #[arg(required = true)]
    inputs: Vec<PathBuf>,

    /// Write rows to this CSV file (default: print a summary only)
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Cylinders of the 4-stroke engine (firing order = cylinders / 2)
    #[arg(long, default_value_t = 4)]
    cylinders: u32,

    /// Minimum steady-segment length in seconds
    #[arg(long, default_value_t = 10.0)]
    steady_min: f64,

    /// Deceleration segment upper and lower rpm
    #[arg(long, default_value_t = 2500.0)]
    decel_hi: f64,
    #[arg(long, default_value_t = 1500.0)]
    decel_lo: f64,

    /// Engine-running threshold: 20-200 Hz RMS per tracking frame (g, 3-axis)
    #[arg(long, default_value_t = 0.02)]
    gate: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let cfg = MetricsConfig {
        cylinders: args.cylinders,
        steady_min_s: args.steady_min,
        decel_hi_rpm: args.decel_hi,
        decel_lo_rpm: args.decel_lo,
        gate_g: args.gate,
        ..MetricsConfig::default()
    };

    let mut columns: Vec<String> = [
        "metrics_version", "file", "time_origin", "time_origin_source", "segment", "kind",
        "t_start_s", "t_end_s", "duration_s", "abs_start", "rpm_mean", "rpm_sd",
    ].iter().map(|s| s.to_string()).collect();
    columns.extend(ORDERS.iter().map(|o| format!("ord_{}_g", order_label(*o))));
    columns.push("ord_0_5_re_fire_db".into());
    columns.extend(BANDS.iter().map(|(lo, hi)| format!("band_{}_{}_g", lo, hi)));
    columns.extend(["accel_rms_ms2", "accel_db", "velocity_rms_mms", "crest_factor", "kurtosis",
        "dc_x_g", "dc_y_g", "dc_z_g", "temperature_c", "decel_rate_rpm_s", "decel_fit_r2"].map(String::from));

    let mut csv = match &args.output {
        Some(path) => {
            let mut w = BufWriter::new(File::create(path)?);
            writeln!(w, "{}", columns.join(","))?;
            Some(w)
        }
        None => None,
    };

    let fire_index = ORDERS.iter().position(|&o| o == cfg.firing_order());
    for path in &args.inputs {
        let reader = Hdf5Reader::open(path)?;
        let meta = reader.metadata().clone();
        let samples = reader.read_range(0, reader.get_total_samples()?)?;
        if samples.len() < 2 {
            eprintln!("{}: no samples", path.display());
            continue;
        }
        let range = match meta.range.as_str() {
            "4g" => Range::G4,
            "8g" => Range::G8,
            _ => Range::G2,
        };
        let duration = samples.last().unwrap().timestamp - samples.first().unwrap().timestamp;
        let fs = (samples.len() - 1) as f64 / duration;
        let mut axes = [Vec::with_capacity(samples.len()), Vec::with_capacity(samples.len()), Vec::with_capacity(samples.len())];
        for s in &samples {
            let (x, y, z) = s.data.accel_to_g(range);
            axes[0].push(x as f64);
            axes[1].push(y as f64);
            axes[2].push(z as f64);
        }
        let temperature: Vec<u16> = samples.iter().map(|s| s.data.temperature).collect();
        let rec = Recording { axes: [&axes[0], &axes[1], &axes[2]], sample_rate: fs, temperature: &temperature };

        let (origin, origin_source) = meta.time_origin();
        let origin_dt = DateTime::parse_from_rfc3339(origin).ok();
        let abs = |t: f64| {
            origin_dt
                .map(|o| (o + Duration::microseconds((t * 1e6) as i64)).to_rfc3339())
                .unwrap_or_default()
        };
        let file_name = path.file_name().map(|n| n.to_string_lossy().replace(',', "_")).unwrap_or_default();

        let frames = track_rpm(&rec, &cfg);
        let runs = steady_runs(&frames, &cfg);
        let decels = decel_segments(&frames, &cfg);
        println!("{}: {:.1} s at {:.1} Hz, {} steady, {} decel segment(s)",
            file_name, duration, fs, runs.len(), decels.len());

        let mut segment = 0;
        for run in runs {
            segment += 1;
            let m = steady_metrics(&rec, &frames, run, &cfg);
            let o2 = fire_index.map(|i| m.orders_g[i]);
            let half_db = o2.filter(|&a| a > 0.0).map(|a| 20.0 * (m.orders_g[0] / a).log10());
            println!("  steady {:>2}: {:>6.1}-{:>6.1} s  {:>6.0} rpm (sd {:.1})  fire order {:.4} g  0.5 order {:+.1} dB  20-200 Hz {:.4} g",
                segment, m.t_start, m.t_end, m.rpm_mean, m.rpm_sd, o2.unwrap_or(f64::NAN),
                half_db.unwrap_or(f64::NAN), m.bands_g[1]);
            if let Some(w) = csv.as_mut() {
                let mut row: Vec<String> = vec![
                    METRICS_VERSION.to_string(), file_name.clone(), origin.to_string(), origin_source.to_string(),
                    segment.to_string(), "steady".into(), f(m.t_start, 3), f(m.t_end, 3), f(m.t_end - m.t_start, 3),
                    abs(m.t_start), f(m.rpm_mean, 1), f(m.rpm_sd, 2),
                ];
                row.extend(m.orders_g.iter().map(|v| f(*v, 6)));
                row.push(half_db.map(|v| f(v, 2)).unwrap_or_default());
                row.extend(m.bands_g.iter().map(|v| f(*v, 6)));
                row.extend([f(m.accel_rms_ms2, 5), f(m.accel_db, 2), f(m.velocity_rms_mms, 4), f(m.crest_factor, 3),
                    f(m.kurtosis, 3), f(m.dc_g[0], 6), f(m.dc_g[1], 6), f(m.dc_g[2], 6),
                    m.temperature_c.map(|t| f(t, 1)).unwrap_or_default(), String::new(), String::new()]);
                writeln!(w, "{}", row.join(","))?;
            }
        }
        for d in decels {
            segment += 1;
            println!("  decel  {:>2}: {:>6.2}-{:>6.2} s  {:.0} to {:.0} rpm in {:.3} s  ({:.0} rpm/s, R2 {:.3})",
                segment, d.t_start, d.t_end, cfg.decel_hi_rpm, cfg.decel_lo_rpm, d.t_end - d.t_start, d.rate_rpm_s, d.fit_r2);
            if let Some(w) = csv.as_mut() {
                let mut row: Vec<String> = vec![
                    METRICS_VERSION.to_string(), file_name.clone(), origin.to_string(), origin_source.to_string(),
                    segment.to_string(), "decel".into(), f(d.t_start, 3), f(d.t_end, 3), f(d.t_end - d.t_start, 4),
                    abs(d.t_start), String::new(), String::new(),
                ];
                // steady-only columns stay empty
                row.extend(std::iter::repeat(String::new()).take(columns.len() - row.len() - 2));
                row.extend([f(d.rate_rpm_s, 2), f(d.fit_r2, 4)]);
                writeln!(w, "{}", row.join(","))?;
            }
        }
    }

    if let (Some(mut w), Some(path)) = (csv, &args.output) {
        w.flush()?;
        println!("Metrics: {}", path.display());
    }
    Ok(())
}

fn order_label(order: f64) -> String {
    format!("{}", order).replace('.', "_")
}

fn f(value: f64, decimals: usize) -> String {
    if value.is_finite() { format!("{:.*}", decimals, value) } else { String::new() }
}
