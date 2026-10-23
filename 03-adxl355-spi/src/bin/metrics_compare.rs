//! Before/after comparison of engine-metrics CSV files
//!
//! Each input CSV is one session (e.g. one day). Rows of the chosen kind (and
//! rpm range) are averaged per session first, then sessions are compared:
//! difference of the group means against the minimum detectable change
//! MDC = 2.8 x pooled session SD x sqrt(1/n_before + 1/n_after)
//! (5% significance, 80% power). Amplitude metrics are also reported scaled to
//! a common rpm, amplitude x (rpm_ref / rpm)^exp.

use clap::Parser;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "metrics-compare")]
#[command(about = "Compare engine-metrics CSV sessions: before vs after, against the minimum detectable change")]
struct Args {
    /// Session CSV files before the change
    #[arg(long, num_args = 1.., required = true)]
    before: Vec<PathBuf>,

    /// Session CSV files after the change
    #[arg(long, num_args = 1.., required = true)]
    after: Vec<PathBuf>,

    /// Segment kind: steady or decel
    #[arg(long, default_value = "steady")]
    kind: String,

    /// Only steady segments with mean rpm in this range
    #[arg(long)]
    rpm_min: Option<f64>,
    #[arg(long)]
    rpm_max: Option<f64>,

    /// Exponent for scaling amplitudes to a common rpm (2 = inertia forces)
    #[arg(long, default_value_t = 2.0)]
    rpm_exp: f64,

    /// Also write the comparison table to this CSV file
    #[arg(short, long)]
    output: Option<PathBuf>,
}

/// Columns that identify a row rather than measure something
const ID_COLUMNS: [&str; 9] = [
    "metrics_version", "file", "time_origin", "time_origin_source", "segment", "kind", "t_start_s", "t_end_s", "abs_start",
];

type Row = BTreeMap<String, f64>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut columns: Vec<String> = Vec::new();
    let before = load_sessions(&args.before, &args, &mut columns)?;
    let after = load_sessions(&args.after, &args, &mut columns)?;

    // Common reference rpm: mean over every selected steady row
    let rpms: Vec<f64> = before.iter().chain(&after).flatten().filter_map(|r| r.get("rpm_mean").copied()).collect();
    let rpm_ref = if rpms.is_empty() { None } else { Some(rpms.iter().sum::<f64>() / rpms.len() as f64) };
    let mut metrics: Vec<String> = columns.clone();
    if let Some(rpm_ref) = rpm_ref {
        metrics.extend(columns.iter().filter(|c| is_amplitude(c)).map(|c| format!("{}@rpm", c)));
        println!("Amplitudes @rpm scaled to {:.0} rpm with exponent {}", rpm_ref, args.rpm_exp);
    }

    let session_means = |group: &[Vec<Row>]| -> Vec<BTreeMap<String, f64>> {
        group.iter().map(|rows| {
            let mut means = BTreeMap::new();
            for m in &metrics {
                let vals: Vec<f64> = rows.iter().filter_map(|r| value(r, m, rpm_ref, args.rpm_exp)).collect();
                if !vals.is_empty() {
                    means.insert(m.clone(), vals.iter().sum::<f64>() / vals.len() as f64);
                }
            }
            means
        }).collect()
    };
    let (mb, ma) = (session_means(&before), session_means(&after));
    let rows_used = |g: &[Vec<Row>]| g.iter().map(|s| s.len().to_string()).collect::<Vec<_>>().join("/");
    println!("Sessions: before {} (segments {}), after {} (segments {}), kind {}",
        before.len(), rows_used(&before), after.len(), rows_used(&after), args.kind);
    println!();
    println!("{:<26} {:>5} {:>24} {:>24} {:>10} {:>10} {:>8}", "metric", "unit", "before mean (sd, n)", "after mean (sd, n)", "change", "MDC", "beyond");

    let mut out = match &args.output {
        Some(p) => {
            let mut w = BufWriter::new(File::create(p)?);
            writeln!(w, "metric,change_unit,n_before,mean_before,sd_before,n_after,mean_after,sd_after,change,mdc,beyond_mdc")?;
            Some(w)
        }
        None => None,
    };
    for m in &metrics {
        let b: Vec<f64> = mb.iter().filter_map(|s| s.get(m).copied()).collect();
        let a: Vec<f64> = ma.iter().filter_map(|s| s.get(m).copied()).collect();
        if b.is_empty() || a.is_empty() {
            continue;
        }
        let (bm, bs) = mean_sd(&b);
        let (am, asd) = mean_sd(&a);
        let db = m.contains("_db");
        let unit = if db { "dB" } else { "%" };
        let change = if db { am - bm } else if bm != 0.0 { (am - bm) / bm.abs() * 100.0 } else { f64::NAN };
        let mdc = pooled_sd(&b, &a).map(|s| {
            let abs = 2.8 * s * (1.0 / b.len() as f64 + 1.0 / a.len() as f64).sqrt();
            if db { abs } else { abs / bm.abs() * 100.0 }
        });
        let beyond = match mdc {
            Some(mdc) if change.is_finite() => if change.abs() > mdc { "yes" } else { "no" },
            _ => "n/a",
        };
        println!("{:<26} {:>5} {:>24} {:>24} {:>+10.2} {:>10} {:>8}",
            m, unit, fmt_group(bm, bs, b.len()), fmt_group(am, asd, a.len()), change,
            mdc.map(|v| format!("{:.2}", v)).unwrap_or_else(|| "n/a".into()), beyond);
        if let Some(w) = out.as_mut() {
            writeln!(w, "{},{},{},{},{},{},{},{},{},{},{}", m, unit, b.len(), bm, nan_empty(bs), a.len(), am, nan_empty(asd),
                change, mdc.map(|v| v.to_string()).unwrap_or_default(), beyond)?;
        }
    }
    if let Some(mut w) = out {
        w.flush()?;
    }
    println!();
    println!("MDC needs at least 2 sessions per group. \"beyond\" = |change| > MDC.");
    Ok(())
}

/// Read each CSV as one session, keeping rows of the requested kind and rpm range
fn load_sessions(paths: &[PathBuf], args: &Args, columns: &mut Vec<String>) -> Result<Vec<Vec<Row>>, Box<dyn std::error::Error>> {
    let mut sessions = Vec::new();
    for path in paths {
        let mut lines = BufReader::new(File::open(path)?).lines();
        let header: Vec<String> = lines.next().ok_or("empty CSV")??.split(',').map(String::from).collect();
        let kind_i = header.iter().position(|h| h == "kind").ok_or("no kind column")?;
        let mut rows = Vec::new();
        for line in lines {
            let line = line?;
            let cells: Vec<&str> = line.split(',').collect();
            if cells.get(kind_i) != Some(&args.kind.as_str()) {
                continue;
            }
            let mut row = Row::new();
            for (h, c) in header.iter().zip(&cells) {
                // a steady segment's length is a choice, not a measurement
                if ID_COLUMNS.contains(&h.as_str()) || (h == "duration_s" && args.kind == "steady") {
                    continue;
                }
                if let Ok(v) = c.parse::<f64>() {
                    row.insert(h.clone(), v);
                    if !columns.contains(h) {
                        columns.push(h.clone());
                    }
                }
            }
            if let Some(&rpm) = row.get("rpm_mean") {
                if args.rpm_min.is_some_and(|lo| rpm < lo) || args.rpm_max.is_some_and(|hi| rpm > hi) {
                    continue;
                }
            }
            rows.push(row);
        }
        sessions.push(rows);
    }
    Ok(sessions)
}

/// Amplitudes in g or m/s^2 that scale with rpm
fn is_amplitude(column: &str) -> bool {
    ((column.starts_with("ord_") || column.starts_with("band_")) && column.ends_with("_g")) || column == "accel_rms_ms2"
}

/// Value of a metric in a row; "<name>@rpm" is the amplitude scaled to rpm_ref
fn value(row: &Row, metric: &str, rpm_ref: Option<f64>, exp: f64) -> Option<f64> {
    match metric.strip_suffix("@rpm") {
        Some(base) => {
            let (rpm_ref, rpm) = (rpm_ref?, *row.get("rpm_mean")?);
            Some(row.get(base)? * (rpm_ref / rpm).powf(exp))
        }
        None => row.get(metric).copied(),
    }
}

fn mean_sd(x: &[f64]) -> (f64, f64) {
    let m = x.iter().sum::<f64>() / x.len() as f64;
    let sd = if x.len() > 1 {
        (x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / (x.len() - 1) as f64).sqrt()
    } else {
        f64::NAN
    };
    (m, sd)
}

fn pooled_sd(b: &[f64], a: &[f64]) -> Option<f64> {
    if b.len() < 2 || a.len() < 2 {
        return None;
    }
    let (sb, sa) = (mean_sd(b).1, mean_sd(a).1);
    let dof = (b.len() + a.len() - 2) as f64;
    Some((((b.len() - 1) as f64 * sb * sb + (a.len() - 1) as f64 * sa * sa) / dof).sqrt())
}

fn fmt_group(mean: f64, sd: f64, n: usize) -> String {
    if sd.is_finite() { format!("{:.5} ({:.5}, {})", mean, sd, n) } else { format!("{:.5} (-, {})", mean, n) }
}

fn nan_empty(v: f64) -> String {
    if v.is_finite() { v.to_string() } else { String::new() }
}
