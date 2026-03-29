//! Vibration analysis: RMS and FFT frequency peak detection
//!
//! Shared by the real-time display renderer and the offline analyzer.

use num_complex::Complex;
use rustfft::FftPlanner;
use std::f64::consts::PI;

/// A detected frequency peak from FFT analysis
#[derive(Debug, Clone)]
pub struct FrequencyPeak {
    pub frequency_hz: f64,
    pub magnitude: f64,
}

/// RMS (root mean square) of a signal buffer
pub fn compute_rms(data: &[f32]) -> f32 {
    let sum_squares: f32 = data.iter().map(|&x| x * x).sum();
    (sum_squares / data.len() as f32).sqrt()
}

/// Apply Hann window, promoting to f64 for FFT precision
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

/// FFT-based frequency peak detection
///
/// Returns peaks sorted by magnitude (descending). Caller takes `.iter().take(n)`
/// for top-N peaks. Returns empty if `data.len() < window_size`.
pub fn find_frequency_peaks(data: &[f32], sample_rate: f64, window_size: usize) -> Vec<FrequencyPeak> {
    if data.len() < window_size {
        return Vec::new();
    }

    let windowed = apply_hann_window(&data[..window_size]);

    let mut buffer: Vec<Complex<f64>> = windowed.iter()
        .map(|&x| Complex::new(x, 0.0))
        .collect();

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(window_size);
    fft.process(&mut buffer);

    let magnitudes: Vec<f64> = buffer.iter()
        .take(window_size / 2)
        .map(|c| c.norm() / (window_size as f64))
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
