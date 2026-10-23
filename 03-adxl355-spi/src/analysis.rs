//! Vibration analysis: RMS, FFT frequency peak detection and meter-style
//! overall values (velocity RMS)
//!
//! Shared by the real-time display renderer and the offline analyzer.

use num_complex::Complex;
use rustfft::FftPlanner;
use std::collections::VecDeque;
use std::f64::consts::{FRAC_1_SQRT_2, PI};

/// A detected frequency peak from FFT analysis
#[derive(Debug, Clone)]
pub struct FrequencyPeak {
    pub frequency_hz: f64,
    /// Sine peak amplitude, in the units of the input data
    pub magnitude: f64,
}

/// RMS (root mean square) of a signal buffer
pub fn compute_rms(data: &[f32]) -> f32 {
    let sum_squares: f64 = data.iter().map(|&x| x as f64 * x as f64).sum();
    (sum_squares / data.len() as f64).sqrt() as f32
}

/// Number of `window_size` segments (50% overlap) `find_frequency_peaks` averages
pub fn fft_segment_count(data_len: usize, window_size: usize) -> usize {
    if data_len < window_size { 0 } else { (data_len - window_size) / (window_size / 2) + 1 }
}

/// FFT-based frequency peak detection over the whole buffer.
///
/// Welch average: `window_size` segments with 50% overlap, each with its mean
/// removed (gravity would otherwise dominate) and a Hann window applied; the
/// power spectra are averaged. Returns peaks sorted by magnitude (descending).
/// Caller takes `.iter().take(n)` for top-N peaks. Returns empty if
/// `data.len() < window_size`.
pub fn find_frequency_peaks(data: &[f32], sample_rate: f64, window_size: usize) -> Vec<FrequencyPeak> {
    let segments = fft_segment_count(data.len(), window_size);
    if segments == 0 || window_size < 4 {
        return Vec::new();
    }

    let window: Vec<f64> = (0..window_size)
        .map(|i| 0.5 * (1.0 - ((2.0 * PI * i as f64) / (window_size as f64 - 1.0)).cos()))
        .collect();
    let window_sum: f64 = window.iter().sum();

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(window_size);
    let mut buffer = vec![Complex::new(0.0, 0.0); window_size];
    let mut power = vec![0.0; window_size / 2];

    for segment in 0..segments {
        let start = segment * (window_size / 2);
        let chunk = &data[start..start + window_size];
        let mean = chunk.iter().map(|&x| x as f64).sum::<f64>() / window_size as f64;
        for ((b, &x), &w) in buffer.iter_mut().zip(chunk).zip(&window) {
            *b = Complex::new((x as f64 - mean) * w, 0.0);
        }
        fft.process(&mut buffer);
        for (p, c) in power.iter_mut().zip(&buffer) {
            *p += c.norm_sqr();
        }
    }

    // Single-sided sine peak amplitude: 2 |X| / sum(window)
    let magnitudes: Vec<f64> = power.iter()
        .map(|&p| 2.0 * (p / segments as f64).sqrt() / window_sum)
        .collect();

    let threshold = magnitudes.iter().copied().fold(0.0, f64::max) * 0.1;
    let mut peaks: Vec<FrequencyPeak> = Vec::new();

    for i in 1..magnitudes.len() - 1 {
        if magnitudes[i] > threshold
            && magnitudes[i] > magnitudes[i - 1]
            && magnitudes[i] > magnitudes[i + 1]
        {
            peaks.push(FrequencyPeak {
                frequency_hz: (i as f64 * sample_rate) / window_size as f64,
                magnitude: magnitudes[i],
            });
        }
    }

    peaks.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap());
    peaks
}

// ============================================================================
// Meter-style overall vibration values
// ============================================================================
//
// Handheld vibration meters report broadband "overall" values: acceleration
// peak, velocity RMS (the usual severity metric) and displacement P-P,
// each band-limited before the statistic is taken. These functions reproduce
// that so results can be compared with a meter reading.

/// m/s^2 per g
pub const STANDARD_GRAVITY: f64 = 9.80665;
/// Lower band edge for all overall values
pub const BAND_LOW_HZ: f64 = 10.0;
/// Upper band edge for acceleration and velocity (typical meter velocity band)
pub const VELOCITY_BAND_HIGH_HZ: f64 = 1000.0;
/// Upper band edge for displacement (typical meter spec, e.g. UNI-T UT311)
pub const DISPLACEMENT_BAND_HIGH_HZ: f64 = 500.0;
/// Filter start-up transient excluded from the statistics
pub const SETTLE_SECONDS: f64 = 1.0;
/// DC blocker ahead of the integrators keeps them bounded through gravity and
/// tilt changes during long live runs; its gain at 10 Hz is 0.99995
const DC_BLOCK_HZ: f64 = 1.0;

/// Effective measurement band after sensor and sampling limits
#[derive(Debug, Clone, Copy)]
pub struct VibrationBand {
    pub low_hz: f64,
    pub velocity_high_hz: f64,
    pub displacement_high_hz: f64,
}

/// Overall values for one axis
#[derive(Debug, Clone, Copy)]
pub struct OverallVibration {
    /// m/s^2 RMS
    pub accel_rms: f64,
    /// m/s^2 zero-to-peak
    pub accel_peak: f64,
    /// mm/s RMS
    pub velocity_rms: f64,
    /// um peak-to-peak
    pub displacement_pp: f64,
}

/// Upper band edges are capped by the sensor LPF (ODR/4 on the ADXL355) and
/// by fs/4, above which the integrator gain error exceeds ~2%.
pub fn vibration_band(sample_rate: f64, sensor_bandwidth_hz: f64) -> VibrationBand {
    let cap = sensor_bandwidth_hz.min(sample_rate / 4.0);
    VibrationBand {
        low_hz: BAND_LOW_HZ,
        velocity_high_hz: VELOCITY_BAND_HIGH_HZ.min(cap),
        displacement_high_hz: DISPLACEMENT_BAND_HIGH_HZ.min(cap),
    }
}

/// Compute overall values from one axis of acceleration in m/s^2 (gravity may be included).
///
/// Runs the same per-sample chain as `VibrationMeter`, with statistics over the
/// whole record after the first SETTLE_SECONDS.
/// Returns None if the band is empty or there is less than 1 s of data after settling.
pub fn overall_vibration(accel_ms2: &[f64], sample_rate: f64, sensor_bandwidth_hz: f64) -> Option<OverallVibration> {
    let band = vibration_band(sample_rate, sensor_bandwidth_hz);
    let skip = (SETTLE_SECONDS * sample_rate) as usize;
    if band.velocity_high_hz <= band.low_hz || accel_ms2.len() < skip + sample_rate as usize {
        return None;
    }

    let mut chain = AxisChain::new(sample_rate, sensor_bandwidth_hz);
    Some(summarize(accel_ms2.iter().map(|&a| chain.process(a)).skip(skip)))
}

/// Band-limited acceleration (same band and filters as `overall_vibration`),
/// after the first SETTLE_SECONDS. Input and output in m/s^2.
pub fn band_limited_accel(accel_ms2: &[f64], sample_rate: f64, sensor_bandwidth_hz: f64) -> Option<Vec<f64>> {
    let band = vibration_band(sample_rate, sensor_bandwidth_hz);
    let skip = (SETTLE_SECONDS * sample_rate) as usize;
    if band.velocity_high_hz <= band.low_hz || accel_ms2.len() < skip + sample_rate as usize {
        return None;
    }

    let mut chain = AxisChain::new(sample_rate, sensor_bandwidth_hz);
    Some(accel_ms2.iter().map(|&a| chain.process(a).0).skip(skip).collect())
}

/// Streaming overall values for live display. Every sample runs through the
/// filter chain; statistics cover a rolling window of the most recent samples.
pub struct VibrationMeter {
    sample_rate: f64,
    sensor_bandwidth_hz: f64,
    settle: usize,
    window: usize,
    seen: usize,
    chains: [AxisChain; 3],
    history: [VecDeque<(f64, f64, f64)>; 3],
}

impl VibrationMeter {
    pub fn new(sample_rate: f64, sensor_bandwidth_hz: f64, window_seconds: f64) -> Self {
        let window = ((window_seconds * sample_rate) as usize).max(1);
        Self {
            sample_rate,
            sensor_bandwidth_hz,
            settle: (SETTLE_SECONDS * sample_rate) as usize,
            window,
            seen: 0,
            chains: std::array::from_fn(|_| AxisChain::new(sample_rate, sensor_bandwidth_hz)),
            history: std::array::from_fn(|_| VecDeque::with_capacity(window)),
        }
    }

    pub fn band(&self) -> VibrationBand {
        vibration_band(self.sample_rate, self.sensor_bandwidth_hz)
    }

    /// Restart after a gap in the data; integration assumes contiguous samples
    pub fn reset(&mut self) {
        *self = Self::new(self.sample_rate, self.sensor_bandwidth_hz, self.window as f64 / self.sample_rate);
    }

    /// Feed one X/Y/Z sample in m/s^2 (gravity may be included)
    pub fn push(&mut self, accel_ms2: [f64; 3]) {
        self.seen += 1;
        let (settled, window) = (self.seen > self.settle, self.window);
        for ((chain, history), a) in self.chains.iter_mut().zip(&mut self.history).zip(accel_ms2) {
            let out = chain.process(a);
            if settled {
                if history.len() >= window {
                    history.pop_front();
                }
                history.push_back(out);
            }
        }
    }

    /// Progress through settling and the first window (1.0 once values are available)
    pub fn progress(&self) -> f64 {
        (self.seen as f64 / (self.settle + self.window) as f64).min(1.0)
    }

    /// X/Y/Z values over the rolling window; None while settling or filling
    pub fn values(&self) -> Option<[OverallVibration; 3]> {
        let band = self.band();
        if band.velocity_high_hz <= band.low_hz || self.history[0].len() < self.window {
            return None;
        }
        Some(std::array::from_fn(|axis| summarize(self.history[axis].iter().copied())))
    }
}

/// Statistics over band-limited (acceleration m/s^2, velocity m/s, displacement m) samples
fn summarize(outputs: impl Iterator<Item = (f64, f64, f64)>) -> OverallVibration {
    let (mut n, mut accel_sq, mut accel_peak, mut velocity_sq) = (0usize, 0.0, 0.0f64, 0.0);
    let (mut disp_min, mut disp_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for (a, v, d) in outputs {
        n += 1;
        accel_sq += a * a;
        accel_peak = accel_peak.max(a.abs());
        velocity_sq += v * v;
        disp_min = disp_min.min(d);
        disp_max = disp_max.max(d);
    }
    let n = n as f64;
    OverallVibration {
        accel_rms: (accel_sq / n).sqrt(),
        accel_peak,
        velocity_rms: (velocity_sq / n).sqrt() * 1e3,
        displacement_pp: (disp_max - disp_min) * 1e6,
    }
}

/// Per-sample chain for one axis: DC block, integrate twice, then a 4th-order
/// Butterworth band on each of acceleration, velocity and displacement, so
/// every metric sees a single Butterworth band edge.
struct AxisChain {
    offset: Option<f64>,
    dc_block: Biquad,
    accel_band: Vec<Biquad>,
    velocity: Integrator,
    velocity_band: Vec<Biquad>,
    displacement: Integrator,
    displacement_band: Vec<Biquad>,
}

impl AxisChain {
    fn new(sample_rate: f64, sensor_bandwidth_hz: f64) -> Self {
        let band = vibration_band(sample_rate, sensor_bandwidth_hz);
        let band_pass = |high_hz: f64| {
            let mut sections = butterworth4(FilterKind::HighPass, band.low_hz, sample_rate);
            // At the sensor LPF the data is already band-limited
            if high_hz < sensor_bandwidth_hz * 0.99 {
                sections.extend(butterworth4(FilterKind::LowPass, high_hz, sample_rate));
            }
            sections
        };
        Self {
            offset: None,
            dc_block: Biquad::new(FilterKind::HighPass, DC_BLOCK_HZ, FRAC_1_SQRT_2, sample_rate),
            accel_band: band_pass(band.velocity_high_hz),
            velocity: Integrator::new(sample_rate),
            velocity_band: band_pass(band.velocity_high_hz),
            displacement: Integrator::new(sample_rate),
            displacement_band: band_pass(band.displacement_high_hz),
        }
    }

    /// Returns band-limited (acceleration m/s^2, velocity m/s, displacement m)
    fn process(&mut self, accel_ms2: f64) -> (f64, f64, f64) {
        // Start from zero so gravity doesn't kick the DC blocker
        let offset = *self.offset.get_or_insert(accel_ms2);
        let a = self.dc_block.process(accel_ms2 - offset);
        let v = self.velocity.process(a);
        let d = self.displacement.process(v);
        (
            cascade(&mut self.accel_band, a),
            cascade(&mut self.velocity_band, v),
            cascade(&mut self.displacement_band, d),
        )
    }
}

/// Al-Alaoui integrator: y[n] = y[n-1] + T * (7 x[n] + x[n-1]) / 8.
/// Gain is within ~2% of an ideal integrator up to fs/4, where trapezoidal
/// integration is already ~21% low.
struct Integrator {
    dt: f64,
    acc: f64,
    prev: f64,
}

impl Integrator {
    fn new(sample_rate: f64) -> Self {
        Self { dt: 1.0 / sample_rate, acc: 0.0, prev: 0.0 }
    }

    fn process(&mut self, x: f64) -> f64 {
        self.acc += self.dt * (7.0 * x + self.prev) / 8.0;
        self.prev = x;
        self.acc
    }
}

#[derive(Clone, Copy)]
enum FilterKind {
    HighPass,
    LowPass,
}

/// RBJ cookbook biquad, transposed direct form II
struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    z1: f64,
    z2: f64,
}

impl Biquad {
    fn new(kind: FilterKind, cutoff_hz: f64, q: f64, sample_rate: f64) -> Self {
        let w0 = 2.0 * PI * cutoff_hz / sample_rate;
        let cos_w0 = w0.cos();
        let alpha = w0.sin() / (2.0 * q);
        let a0 = 1.0 + alpha;
        let (b0, b1, b2) = match kind {
            FilterKind::HighPass => ((1.0 + cos_w0) / 2.0, -(1.0 + cos_w0), (1.0 + cos_w0) / 2.0),
            FilterKind::LowPass => ((1.0 - cos_w0) / 2.0, 1.0 - cos_w0, (1.0 - cos_w0) / 2.0),
        };
        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha) / a0,
            z1: 0.0,
            z2: 0.0,
        }
    }

    fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// 4th-order Butterworth as two biquad sections, -3 dB at `cutoff_hz`
fn butterworth4(kind: FilterKind, cutoff_hz: f64, sample_rate: f64) -> Vec<Biquad> {
    // Q = 1 / (2 cos(pi/8)) and 1 / (2 cos(3pi/8))
    [0.541_196_100_146_197, 1.306_562_964_876_376_6]
        .iter()
        .map(|&q| Biquad::new(kind, cutoff_hz, q, sample_rate))
        .collect()
}

fn cascade(sections: &mut [Biquad], x: f64) -> f64 {
    sections.iter_mut().fold(x, |y, s| s.process(y))
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f64 = STANDARD_GRAVITY;

    fn sine(freq_hz: f64, amplitude: f64, sample_rate: f64, seconds: f64) -> Vec<f64> {
        let n = (sample_rate * seconds) as usize;
        (0..n).map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin()).collect()
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64, what: &str) {
        let err = (actual - expected).abs() / expected;
        assert!(err < tolerance, "{}: got {:.5}, expected {:.5} ({:.2}% off)", what, actual, expected, err * 100.0);
    }

    /// 80 Hz, 1 m/s^2 sine on top of 1 g: compare with closed-form values
    #[test]
    fn sine_matches_closed_form() {
        let fs = 1000.0;
        let (f, a) = (80.0, 1.0);
        let w = 2.0 * PI * f;
        let accel: Vec<f64> = sine(f, a, fs, 10.0).iter().map(|x| x + G).collect();

        let v = overall_vibration(&accel, fs, fs / 4.0).unwrap();
        assert_close(v.accel_rms, a / 2f64.sqrt(), 0.01, "accel RMS");
        assert_close(v.accel_peak, a, 0.01, "accel peak");
        assert_close(v.velocity_rms, a / w / 2f64.sqrt() * 1e3, 0.01, "velocity RMS");
        // P-P is taken from discrete samples, so allow for missing the exact crest
        assert_close(v.displacement_pp, 2.0 * a / (w * w) * 1e6, 0.02, "displacement P-P");
    }

    /// Slow tilt/sway below the band must not inflate the readings
    #[test]
    fn low_frequency_motion_is_rejected() {
        let fs = 1000.0;
        let (f, a) = (80.0, 1.0);
        let w = 2.0 * PI * f;
        let sway = sine(0.5, 0.01 * G, fs, 10.0);
        let accel: Vec<f64> = sine(f, a, fs, 10.0).iter().zip(&sway)
            .enumerate()
            .map(|(i, (x, s))| x + s + G + 0.002 * G * i as f64 / fs) // + slow offset drift
            .collect();

        let v = overall_vibration(&accel, fs, fs / 4.0).unwrap();
        assert_close(v.velocity_rms, a / w / 2f64.sqrt() * 1e3, 0.02, "velocity RMS");
        assert_close(v.displacement_pp, 2.0 * a / (w * w) * 1e6, 0.05, "displacement P-P");
    }

    /// Band edge: 4th-order Butterworth is -3 dB at 10 Hz
    #[test]
    fn low_band_edge_is_minus_3db() {
        let fs = 1000.0;
        let (f, a) = (10.0, 1.0);
        let v = overall_vibration(&sine(f, a, fs, 20.0), fs, fs / 4.0).unwrap();
        let unfiltered = a / (2.0 * PI * f) / 2f64.sqrt() * 1e3;
        assert_close(v.velocity_rms, unfiltered / 2f64.sqrt(), 0.02, "velocity RMS at 10 Hz");
    }

    /// Integration stays accurate close to the sensor LPF (ODR/4)
    #[test]
    fn integrator_accurate_near_sensor_bandwidth() {
        let fs = 1000.0;
        let (f, a) = (200.0, 1.0);
        let v = overall_vibration(&sine(f, a, fs, 10.0), fs, fs / 4.0).unwrap();
        assert_close(v.velocity_rms, a / (2.0 * PI * f) / 2f64.sqrt() * 1e3, 0.025, "velocity RMS at 200 Hz");
    }

    /// Gravity offset must not hide peaks; magnitude is the sine amplitude
    #[test]
    fn fft_finds_peak_despite_gravity() {
        let (fs, n) = (4000.0, 2048);
        let f = 12.0 * fs / n as f64; // bin centre, 23.4 Hz
        let data: Vec<f32> = (0..n * 4)
            .map(|i| (1.0 + 0.002 * (2.0 * PI * f * i as f64 / fs).sin()) as f32)
            .collect();
        let peaks = find_frequency_peaks(&data, fs, n);
        assert!(!peaks.is_empty(), "no peaks with 1 g offset");
        assert!((peaks[0].frequency_hz - f).abs() < 1e-9);
        assert_close(peaks[0].magnitude, 0.002, 0.02, "peak amplitude");
    }

    /// The whole buffer is analysed, not just the first window
    #[test]
    fn fft_uses_whole_buffer() {
        let (fs, n) = (1000.0, 256);
        let f = 40.0 * fs / n as f64;
        let data: Vec<f32> = (0..n * 8)
            .map(|i| if i < n * 4 { 0.0 } else { (0.01 * (2.0 * PI * f * i as f64 / fs).sin()) as f32 })
            .collect();
        assert_eq!(fft_segment_count(data.len(), n), 15);
        let peaks = find_frequency_peaks(&data, fs, n);
        assert!(!peaks.is_empty() && (peaks[0].frequency_hz - f).abs() < 1e-9);
    }

    #[test]
    fn band_follows_odr() {
        let b = vibration_band(1000.0, 250.0);
        assert_eq!((b.velocity_high_hz, b.displacement_high_hz), (250.0, 250.0));
        let b = vibration_band(4000.0, 1000.0);
        assert_eq!((b.velocity_high_hz, b.displacement_high_hz), (1000.0, 500.0));
    }

    #[test]
    fn short_or_empty_band_returns_none() {
        assert!(overall_vibration(&sine(80.0, 1.0, 1000.0, 1.5), 1000.0, 250.0).is_none());
        assert!(overall_vibration(&sine(5.0, 1.0, 31.25, 60.0), 31.25, 7.8).is_none());
    }

    /// Rolling meter: no values until settled + one window, then closed-form results
    #[test]
    fn meter_matches_closed_form() {
        let fs = 1000.0;
        let (f, a) = (80.0, 1.0);
        let mut meter = VibrationMeter::new(fs, fs / 4.0, 1.0);
        for (i, v) in sine(f, a, fs, 5.0).into_iter().enumerate() {
            meter.push([v, 0.0, v + G]);
            if i + 1 < 2000 {
                assert!(meter.values().is_none(), "values before settle + window at sample {}", i);
            }
        }
        let v = meter.values().unwrap();
        let expected = a / (2.0 * PI * f) / 2f64.sqrt() * 1e3;
        assert_close(v[0].velocity_rms, expected, 0.01, "X velocity RMS");
        assert_close(v[2].velocity_rms, expected, 0.01, "Z velocity RMS (with gravity)");
        assert!(v[1].velocity_rms < 1e-9);
    }

    /// A sudden tilt (0.5 g shift on the axis) only disturbs the reading briefly
    #[test]
    fn meter_recovers_after_tilt() {
        let fs = 1000.0;
        let (f, a) = (80.0, 1.0);
        let w = 2.0 * PI * f;
        let mut meter = VibrationMeter::new(fs, fs / 4.0, 1.0);
        for (i, v) in sine(f, a, fs, 8.0).into_iter().enumerate() {
            let tilt = if i >= 5000 { 0.5 * G } else { 0.0 };
            meter.push([v + G + tilt, 0.0, 0.0]);
        }
        let v = meter.values().unwrap();
        assert_close(v[0].velocity_rms, a / w / 2f64.sqrt() * 1e3, 0.02, "velocity RMS after tilt");
        assert_close(v[0].displacement_pp, 2.0 * a / (w * w) * 1e6, 0.03, "displacement P-P after tilt");
    }

    #[test]
    fn meter_reset_restarts_settling() {
        let mut meter = VibrationMeter::new(1000.0, 250.0, 1.0);
        for v in sine(80.0, 1.0, 1000.0, 3.0) {
            meter.push([v, v, v]);
        }
        assert!(meter.values().is_some());
        meter.reset();
        assert!(meter.values().is_none());
        assert_eq!(meter.progress(), 0.0);
    }
}
