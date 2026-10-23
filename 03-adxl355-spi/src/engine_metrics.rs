//! Engine vibration metrics, with engine speed taken from the vibration itself
//!
//! Every value is a fixed computation on the recorded samples, so the same
//! version on the same file always gives the same numbers. Definitions and
//! defaults are documented in METRICS.md.

use crate::analysis::{band_limited_accel, overall_vibration, STANDARD_GRAVITY};
use num_complex::Complex;
use rustfft::FftPlanner;

/// Bump when a definition or default changes; written to every result row
pub const METRICS_VERSION: u32 = 1;

/// Engine orders reported for steady segments
pub const ORDERS: [f64; 7] = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0];

/// Frequency bands (Hz) reported for steady segments: rigid-body shake,
/// engine orders, higher mechanical content (kept below the sensor filter edge)
pub const BANDS: [(f64, f64); 3] = [(2.0, 20.0), (20.0, 200.0), (200.0, 800.0)];

/// Engine-speed tracking frame and hop
const FRAME_S: f64 = 0.5;
const HOP_S: f64 = 0.05;
/// Zero padding of each tracking frame (finer peak position)
const PAD: usize = 8;
/// Firing-order peak must stand this far above the median of the search band
const MIN_PROMINENCE: f64 = 10.0;
/// Welch segment length for steady-segment spectra (~0.5 Hz resolution at 4 kHz)
const WELCH_SECONDS: f64 = 2.0;
/// Reference for acceleration levels in dB (common in vehicle NVH)
const DB_REF_MS2: f64 = 1e-6;

#[derive(Debug, Clone)]
pub struct MetricsConfig {
    /// Cylinders of a 4-stroke engine; the firing order is cylinders / 2
    pub cylinders: u32,
    pub rpm_min: f64,
    pub rpm_max: f64,
    /// A tracking frame counts as "engine running" above this 20-200 Hz RMS (g, 3-axis)
    pub gate_g: f64,
    /// Steady segment: rpm stays within this fraction of the running mean
    pub steady_tol: f64,
    pub steady_min_s: f64,
    /// Deceleration segment: rpm falls through decel_hi_rpm, then decel_lo_rpm
    pub decel_hi_rpm: f64,
    pub decel_lo_rpm: f64,
    pub decel_max_s: f64,
    /// Deceleration fits below this R^2 are dropped (gear shifts, tracking errors)
    pub decel_min_r2: f64,
    /// Sensor low-pass bandwidth (ADXL355: ODR / 4)
    pub sensor_bandwidth_hz: f64,
}

impl Default for MetricsConfig {
    fn default() -> Self {
        Self {
            cylinders: 4,
            rpm_min: 500.0,
            rpm_max: 7000.0,
            gate_g: 0.02,
            steady_tol: 0.02,
            steady_min_s: 10.0,
            decel_hi_rpm: 2500.0,
            decel_lo_rpm: 1500.0,
            decel_max_s: 10.0,
            decel_min_r2: 0.9,
            sensor_bandwidth_hz: 1000.0,
        }
    }
}

impl MetricsConfig {
    pub fn firing_order(&self) -> f64 {
        self.cylinders as f64 / 2.0
    }
}

/// Engine speed at the centre of one tracking frame
#[derive(Debug, Clone, Copy)]
pub struct RpmFrame {
    pub t: f64,
    pub rpm: f64,
    pub valid: bool,
}

/// One recording: three axes in g, uniformly sampled
pub struct Recording<'a> {
    pub axes: [&'a [f64]; 3],
    pub sample_rate: f64,
    /// Raw temperature register values (0 = not recorded)
    pub temperature: &'a [u16],
}

#[derive(Debug, Clone)]
pub struct SteadyMetrics {
    pub t_start: f64,
    pub t_end: f64,
    pub rpm_mean: f64,
    pub rpm_sd: f64,
    /// RMS (g, 3-axis) of each order in ORDERS
    pub orders_g: [f64; ORDERS.len()],
    /// RMS (g, 3-axis) of each band in BANDS
    pub bands_g: [f64; BANDS.len()],
    /// 3-axis overall acceleration RMS 10-1000 Hz (m/s^2) and its level in dB re 1e-6 m/s^2
    pub accel_rms_ms2: f64,
    pub accel_db: f64,
    /// Highest axis, meter style
    pub velocity_rms_mms: f64,
    pub crest_factor: f64,
    pub kurtosis: f64,
    /// Mean acceleration per axis (g): sensor attitude, for mount checks
    pub dc_g: [f64; 3],
    pub temperature_c: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct DecelMetrics {
    /// Time the rpm crossed decel_hi_rpm and decel_lo_rpm
    pub t_start: f64,
    pub t_end: f64,
    /// Least-squares slope of rpm against time between the crossings (rpm/s, negative)
    pub rate_rpm_s: f64,
    pub fit_r2: f64,
}

/// Engine speed from the firing-order peak in short overlapping frames.
///
/// Each frame: Hann window, mean removed, zero padded, power summed over the
/// three axes. The peak is searched in the firing-order band for rpm_min..rpm_max,
/// weighted by its second harmonic to break near-ties, and refined by parabolic
/// interpolation. A frame is valid when the 20-200 Hz level exceeds gate_g and
/// the peak stands MIN_PROMINENCE above the median of the search band.
pub fn track_rpm(rec: &Recording, cfg: &MetricsConfig) -> Vec<RpmFrame> {
    let fs = rec.sample_rate;
    let len = rec.axes[0].len();
    let n = (FRAME_S * fs).round() as usize;
    let hop = (HOP_S * fs).round() as usize;
    let nfft = (n * PAD).next_power_of_two();
    let half = nfft / 2;
    let df = fs / nfft as f64;
    if len < n {
        return Vec::new();
    }

    let window = hann(n);
    let window_power: f64 = window.iter().map(|w| w * w).sum();
    let fft = FftPlanner::<f64>::new().plan_fft_forward(nfft);

    let order = cfg.firing_order();
    let k_lo = ((cfg.rpm_min / 60.0 * order) / df).ceil().max(1.0) as usize;
    let k_hi = (((cfg.rpm_max / 60.0 * order) / df).floor() as usize).min(half / 2 - 1);
    let gate_lo = (20.0 / df).ceil() as usize;
    let gate_hi = ((200.0 / df).floor() as usize).min(half);

    let mut power = vec![0.0f64; half + 1];
    let mut buf = vec![Complex::new(0.0, 0.0); nfft];
    let mut frames = Vec::with_capacity((len - n) / hop + 1);
    let mut start = 0;
    while start + n <= len {
        power.iter_mut().for_each(|p| *p = 0.0);
        for axis in rec.axes {
            let seg = &axis[start..start + n];
            let mean = seg.iter().sum::<f64>() / n as f64;
            for (i, b) in buf.iter_mut().enumerate() {
                *b = if i < n { Complex::new((seg[i] - mean) * window[i], 0.0) } else { Complex::new(0.0, 0.0) };
            }
            fft.process(&mut buf);
            for (p, b) in power.iter_mut().zip(&buf) {
                *p += b.norm_sqr();
            }
        }

        // Single-sided mean square in the 20-200 Hz band (Parseval, window corrected)
        let gate_ms = 2.0 * power[gate_lo..=gate_hi].iter().sum::<f64>() / (nfft as f64 * window_power);

        // The peak itself dominates the score; its second harmonic only breaks
        // near-ties (a plain sum would favour the sub-harmonic)
        let peak_max = power[k_lo..=k_hi].iter().cloned().fold(f64::MIN_POSITIVE, f64::max);
        let (mut best_k, mut best_score) = (k_lo, f64::MIN);
        for k in k_lo..=k_hi {
            let score = power[k] * (1.0 + power[2 * k] / peak_max);
            if score > best_score {
                best_k = k;
                best_score = score;
            }
        }
        let mut band: Vec<f64> = power[k_lo..=k_hi].to_vec();
        let mid = band.len() / 2;
        let median = *band.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1;
        let prominence = power[best_k] / median.max(f64::MIN_POSITIVE);

        let (a, b, c) = (power[best_k - 1].ln(), power[best_k].ln(), power[best_k + 1].ln());
        let denom = a - 2.0 * b + c;
        let offset = if denom.abs() > 0.0 { (0.5 * (a - c) / denom).clamp(-0.5, 0.5) } else { 0.0 };
        let freq = (best_k as f64 + offset) * df;

        frames.push(RpmFrame {
            t: (start as f64 + n as f64 / 2.0) / fs,
            rpm: freq / order * 60.0,
            valid: gate_ms.sqrt() >= cfg.gate_g && prominence >= MIN_PROMINENCE,
        });
        start += hop;
    }
    frames
}

/// Runs of valid frames where the rpm stays within steady_tol of the run mean,
/// lasting at least steady_min_s. Returns (first, last) frame indices.
pub fn steady_runs(frames: &[RpmFrame], cfg: &MetricsConfig) -> Vec<(usize, usize)> {
    let mut runs = Vec::new();
    let mut i = 0;
    while i < frames.len() {
        if !frames[i].valid {
            i += 1;
            continue;
        }
        let (mut j, mut sum) = (i, frames[i].rpm);
        while j + 1 < frames.len() && frames[j + 1].valid {
            let mean = sum / (j - i + 1) as f64;
            if (frames[j + 1].rpm - mean).abs() > cfg.steady_tol * mean {
                break;
            }
            j += 1;
            sum += frames[j].rpm;
        }
        if frames[j].t - frames[i].t >= cfg.steady_min_s {
            runs.push((i, j));
        }
        i = j + 1;
    }
    runs
}

/// Metrics over the samples between two tracking frames
pub fn steady_metrics(rec: &Recording, frames: &[RpmFrame], run: (usize, usize), cfg: &MetricsConfig) -> SteadyMetrics {
    let fs = rec.sample_rate;
    let (t_start, t_end) = (frames[run.0].t, frames[run.1].t);
    let s0 = (t_start * fs).round() as usize;
    let s1 = ((t_end * fs).round() as usize).min(rec.axes[0].len());

    let rpms: Vec<f64> = frames[run.0..=run.1].iter().map(|f| f.rpm).collect();
    let rpm_mean = mean(&rpms);
    let rpm_sd = (rpms.iter().map(|r| (r - rpm_mean).powi(2)).sum::<f64>() / (rpms.len() - 1).max(1) as f64).sqrt();

    // Welch PSD per axis, summed (g^2/Hz)
    let nper = ((WELCH_SECONDS * fs) as usize).next_power_of_two();
    let mut psd = vec![0.0; nper / 2 + 1];
    let mut df = fs / nper as f64;
    for axis in rec.axes {
        let (axis_psd, axis_df) = welch_psd(&axis[s0..s1], fs, nper);
        df = axis_df;
        psd.iter_mut().zip(&axis_psd).for_each(|(p, a)| *p += a);
    }
    let band_rms = |lo: f64, hi: f64| -> f64 {
        let k0 = (lo / df).ceil() as usize;
        let k1 = ((hi / df).floor() as usize).min(psd.len() - 1);
        if k1 < k0 { 0.0 } else { (psd[k0..=k1].iter().sum::<f64>() * df).sqrt() }
    };
    let f1 = rpm_mean / 60.0;
    let mut orders_g = [0.0; ORDERS.len()];
    for (o, &order) in orders_g.iter_mut().zip(ORDERS.iter()) {
        let f = order * f1;
        // the rpm may drift by steady_tol within the segment
        let half_width = (cfg.steady_tol * 1.25 * f).max(1.0);
        *o = band_rms(f - half_width, f + half_width);
    }
    let mut bands_g = [0.0; BANDS.len()];
    for (b, &(lo, hi)) in bands_g.iter_mut().zip(BANDS.iter()) {
        *b = band_rms(lo, hi);
    }

    // Meter-style values per axis (10-1000 Hz, m/s^2)
    let mut accel_ms = 0.0;
    let (mut velocity_rms_mms, mut crest_factor, mut kurtosis) = (0.0f64, 0.0f64, 0.0f64);
    let mut dc_g = [0.0; 3];
    for (axis, dc) in rec.axes.iter().zip(dc_g.iter_mut()) {
        let seg = &axis[s0..s1];
        *dc = mean(seg);
        let ms2: Vec<f64> = seg.iter().map(|g| g * STANDARD_GRAVITY).collect();
        if let Some(v) = overall_vibration(&ms2, fs, cfg.sensor_bandwidth_hz) {
            accel_ms += v.accel_rms * v.accel_rms;
            velocity_rms_mms = velocity_rms_mms.max(v.velocity_rms);
            if v.accel_rms > 0.0 {
                crest_factor = crest_factor.max(v.accel_peak / v.accel_rms);
            }
        }
        if let Some(x) = band_limited_accel(&ms2, fs, cfg.sensor_bandwidth_hz) {
            let m = mean(&x);
            let m2 = x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / x.len() as f64;
            let m4 = x.iter().map(|v| (v - m).powi(4)).sum::<f64>() / x.len() as f64;
            if m2 > 0.0 {
                kurtosis = kurtosis.max(m4 / (m2 * m2));
            }
        }
    }
    let accel_rms_ms2 = accel_ms.sqrt();

    let temps: Vec<f64> = rec.temperature[s0.min(rec.temperature.len())..s1.min(rec.temperature.len())]
        .iter()
        .filter(|&&raw| raw > 0)
        .map(|&raw| ((raw as f64 - 1885.0) / -9.05) + 25.0)
        .collect();

    SteadyMetrics {
        t_start,
        t_end,
        rpm_mean,
        rpm_sd,
        orders_g,
        bands_g,
        accel_rms_ms2,
        accel_db: 20.0 * (accel_rms_ms2 / DB_REF_MS2).log10(),
        velocity_rms_mms,
        crest_factor,
        kurtosis,
        dc_g,
        temperature_c: if temps.is_empty() { None } else { Some(mean(&temps)) },
    }
}

/// Engine-speed drops from decel_hi_rpm to decel_lo_rpm on valid frames
/// (e.g. throttle released in neutral). The rate is the least-squares slope of
/// the frames between the two crossings. A step of more than MAX_FRAME_STEP
/// between neighbouring frames is a tracking jump, not a deceleration; fits
/// with fewer than 3 frames or R^2 below decel_min_r2 are dropped.
pub fn decel_segments(frames: &[RpmFrame], cfg: &MetricsConfig) -> Vec<DecelMetrics> {
    const MAX_FRAME_STEP: f64 = 0.1;
    let (hi, lo) = (cfg.decel_hi_rpm, cfg.decel_lo_rpm);
    let smooth = |a: &RpmFrame, b: &RpmFrame| a.valid && b.valid && (b.rpm - a.rpm).abs() <= MAX_FRAME_STEP * a.rpm;
    let cross = |a: &RpmFrame, b: &RpmFrame, level: f64| a.t + (a.rpm - level) / (a.rpm - b.rpm) * (b.t - a.t);
    let mut out = Vec::new();
    let mut i = 1;
    while i < frames.len() {
        let (a, b) = (&frames[i - 1], &frames[i]);
        if !(smooth(a, b) && a.rpm >= hi && b.rpm < hi) {
            i += 1;
            continue;
        }
        let t_hi = cross(a, b, hi);
        let mut j = i + 1;
        let mut found = None;
        while j < frames.len() && smooth(&frames[j - 1], &frames[j]) && frames[j].t - t_hi <= cfg.decel_max_s {
            if frames[j].rpm > hi {
                break; // went back up: not a clean deceleration
            }
            if frames[j].rpm < lo {
                found = Some(j);
                break;
            }
            j += 1;
        }
        match found {
            Some(j) if j - i >= 3 => {
                let t_lo = cross(&frames[j - 1], &frames[j], lo);
                let pts: Vec<(f64, f64)> = frames[i..j].iter().map(|f| (f.t, f.rpm)).collect();
                let (rate, r2) = linear_fit(&pts);
                if r2 >= cfg.decel_min_r2 {
                    out.push(DecelMetrics { t_start: t_hi, t_end: t_lo, rate_rpm_s: rate, fit_r2: r2 });
                }
                i = j + 1;
            }
            _ => i += 1,
        }
    }
    out
}

fn hann(n: usize) -> Vec<f64> {
    (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos()).collect()
}

fn mean(x: &[f64]) -> f64 {
    if x.is_empty() { 0.0 } else { x.iter().sum::<f64>() / x.len() as f64 }
}

/// One-sided PSD (units^2/Hz): Hann segments of `nper` with 50% overlap, mean removed
fn welch_psd(x: &[f64], fs: f64, nper: usize) -> (Vec<f64>, f64) {
    let df = fs / nper as f64;
    let mut psd = vec![0.0; nper / 2 + 1];
    if x.len() < nper {
        return (psd, df);
    }
    let window = hann(nper);
    let scale = 2.0 / (fs * window.iter().map(|w| w * w).sum::<f64>());
    let fft = FftPlanner::<f64>::new().plan_fft_forward(nper);
    let mut buf = vec![Complex::new(0.0, 0.0); nper];
    let mut count = 0;
    let mut start = 0;
    while start + nper <= x.len() {
        let seg = &x[start..start + nper];
        let m = mean(seg);
        for (i, b) in buf.iter_mut().enumerate() {
            *b = Complex::new((seg[i] - m) * window[i], 0.0);
        }
        fft.process(&mut buf);
        for (p, b) in psd.iter_mut().zip(&buf) {
            *p += b.norm_sqr() * scale;
        }
        count += 1;
        start += nper / 2;
    }
    psd.iter_mut().for_each(|p| *p /= count as f64);
    (psd, df)
}

/// Slope and R^2 of y against x
fn linear_fit(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len() as f64;
    if pts.len() < 2 {
        return (f64::NAN, f64::NAN);
    }
    let (mx, my) = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
    let sxx: f64 = pts.iter().map(|p| (p.0 - mx).powi(2)).sum();
    let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    let syy: f64 = pts.iter().map(|p| (p.1 - my).powi(2)).sum();
    let slope = sxy / sxx;
    let r2 = if syy > 0.0 { sxy * sxy / (sxx * syy) } else { 1.0 };
    (slope, r2)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthetic 4-cylinder signal: 2nd order sine at a known rpm plus a weaker 0.5 order
    fn synth(rpm: impl Fn(f64) -> f64, seconds: f64, fs: f64) -> Vec<f64> {
        let mut phase = 0.0;
        (0..(seconds * fs) as usize)
            .map(|i| {
                let t = i as f64 / fs;
                phase += 2.0 * std::f64::consts::PI * rpm(t) / 60.0 / fs;
                0.3 * (2.0 * phase).sin() + 0.03 * (0.5 * phase).sin()
            })
            .collect()
    }

    #[test]
    fn tracks_steady_rpm_and_orders() {
        let fs = 4000.0;
        let x = synth(|_| 1800.0, 20.0, fs);
        let zeros = vec![0.0; x.len()];
        let rec = Recording { axes: [&x, &zeros, &zeros], sample_rate: fs, temperature: &[] };
        let cfg = MetricsConfig::default();
        let frames = track_rpm(&rec, &cfg);
        let bad: Vec<_> = frames.iter().filter(|f| !(f.valid && (f.rpm - 1800.0).abs() < 2.0)).take(3).collect();
        assert!(bad.is_empty(), "{} frames, bad: {bad:?}", frames.len());
        let runs = steady_runs(&frames, &cfg);
        assert_eq!(runs.len(), 1);
        let m = steady_metrics(&rec, &frames, runs[0], &cfg);
        let o2 = m.orders_g[ORDERS.iter().position(|&o| o == 2.0).unwrap()];
        let o05 = m.orders_g[0];
        assert!((o2 - 0.3 / 2f64.sqrt()).abs() < 0.01, "2nd order {o2}");
        assert!((o05 - 0.03 / 2f64.sqrt()).abs() < 0.003, "0.5 order {o05}");
    }

    #[test]
    fn measures_deceleration_rate() {
        let fs = 4000.0;
        // 3000 rpm held 2 s, then -800 rpm/s down to 1000 rpm
        let x = synth(|t| if t < 2.0 { 3000.0 } else { (3000.0 - 800.0 * (t - 2.0)).max(1000.0) }, 6.0, fs);
        let zeros = vec![0.0; x.len()];
        let rec = Recording { axes: [&x, &zeros, &zeros], sample_rate: fs, temperature: &[] };
        let cfg = MetricsConfig::default();
        let decels = decel_segments(&track_rpm(&rec, &cfg), &cfg);
        assert_eq!(decels.len(), 1);
        assert!((decels[0].rate_rpm_s + 800.0).abs() < 20.0, "rate {}", decels[0].rate_rpm_s);
        assert!(((decels[0].t_end - decels[0].t_start) - 1.25).abs() < 0.05);
    }
}
