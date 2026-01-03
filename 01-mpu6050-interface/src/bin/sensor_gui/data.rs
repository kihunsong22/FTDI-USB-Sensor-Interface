//! Data loading, downsampling, and FFT computation

use crate::state::{DisplayData, FftResults, LoadedData};
use ft232_sensor_interface::{Hdf5Reader, TimestampedSample};
use num_complex::Complex;
use rustfft::FftPlanner;
use std::f64::consts::PI;
use std::path::Path;

/// Maximum points to display (for performance)
const MAX_DISPLAY_POINTS: usize = 4000;

/// Load data from HDF5 file
pub fn load_file(path: &Path) -> Result<LoadedData, String> {
    let reader = Hdf5Reader::open(path)
        .map_err(|e| format!("Failed to open file: {}", e))?;

    let total = reader.get_total_samples()
        .map_err(|e| format!("Failed to get sample count: {}", e))?;

    if total == 0 {
        return Err("File contains no samples".to_string());
    }

    let samples = reader.read_range(0, total)
        .map_err(|e| format!("Failed to read samples: {}", e))?;

    let time_range = (
        samples.first().unwrap().timestamp,
        samples.last().unwrap().timestamp,
    );

    Ok(LoadedData {
        metadata: reader.metadata().clone(),
        samples,
        time_range,
    })
}

/// Downsample data for display using MinMax algorithm
pub fn downsample(
    samples: &[TimestampedSample],
    time_range: (f64, f64),
) -> DisplayData {
    // Filter to time range
    let filtered: Vec<_> = samples
        .iter()
        .filter(|s| s.timestamp >= time_range.0 && s.timestamp <= time_range.1)
        .collect();

    if filtered.is_empty() {
        return DisplayData {
            timestamps: vec![],
            accel_x: vec![],
            accel_y: vec![],
            accel_z: vec![],
            gyro_x: vec![],
            gyro_y: vec![],
            gyro_z: vec![],
        };
    }

    let n = filtered.len();
    if n <= MAX_DISPLAY_POINTS {
        // No downsampling needed
        return extract_display_data(&filtered);
    }

    // MinMax downsampling: preserve peaks
    downsample_minmax(&filtered, MAX_DISPLAY_POINTS)
}

/// Extract display data from samples
fn extract_display_data(samples: &[&TimestampedSample]) -> DisplayData {
    DisplayData {
        timestamps: samples.iter().map(|s| s.timestamp).collect(),
        accel_x: samples.iter().map(|s| s.data.accel_x_g()).collect(),
        accel_y: samples.iter().map(|s| s.data.accel_y_g()).collect(),
        accel_z: samples.iter().map(|s| s.data.accel_z_g()).collect(),
        gyro_x: samples.iter().map(|s| s.data.gyro_x_dps()).collect(),
        gyro_y: samples.iter().map(|s| s.data.gyro_y_dps()).collect(),
        gyro_z: samples.iter().map(|s| s.data.gyro_z_dps()).collect(),
    }
}

/// MinMax downsampling - keeps min and max from each bucket
fn downsample_minmax(samples: &[&TimestampedSample], target_points: usize) -> DisplayData {
    let bucket_size = samples.len() / (target_points / 2);
    if bucket_size < 2 {
        return extract_display_data(samples);
    }

    let mut timestamps = Vec::with_capacity(target_points);
    let mut accel_x = Vec::with_capacity(target_points);
    let mut accel_y = Vec::with_capacity(target_points);
    let mut accel_z = Vec::with_capacity(target_points);
    let mut gyro_x = Vec::with_capacity(target_points);
    let mut gyro_y = Vec::with_capacity(target_points);
    let mut gyro_z = Vec::with_capacity(target_points);

    for chunk in samples.chunks(bucket_size) {
        if chunk.is_empty() {
            continue;
        }

        // Find min and max by total acceleration magnitude
        let (min_idx, max_idx) = chunk
            .iter()
            .enumerate()
            .fold((0, 0), |(min_i, max_i), (i, s)| {
                let mag = s.data.accel_x_g().powi(2)
                    + s.data.accel_y_g().powi(2)
                    + s.data.accel_z_g().powi(2);
                let min_mag = chunk[min_i].data.accel_x_g().powi(2)
                    + chunk[min_i].data.accel_y_g().powi(2)
                    + chunk[min_i].data.accel_z_g().powi(2);
                let max_mag = chunk[max_i].data.accel_x_g().powi(2)
                    + chunk[max_i].data.accel_y_g().powi(2)
                    + chunk[max_i].data.accel_z_g().powi(2);

                (
                    if mag < min_mag { i } else { min_i },
                    if mag > max_mag { i } else { max_i },
                )
            });

        // Add in time order
        let (first_idx, second_idx) = if min_idx < max_idx {
            (min_idx, max_idx)
        } else {
            (max_idx, min_idx)
        };

        for &idx in &[first_idx, second_idx] {
            let s = chunk[idx];
            timestamps.push(s.timestamp);
            accel_x.push(s.data.accel_x_g());
            accel_y.push(s.data.accel_y_g());
            accel_z.push(s.data.accel_z_g());
            gyro_x.push(s.data.gyro_x_dps());
            gyro_y.push(s.data.gyro_y_dps());
            gyro_z.push(s.data.gyro_z_dps());
        }
    }

    DisplayData {
        timestamps,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
    }
}

/// Compute FFT for all axes
pub fn compute_fft(
    samples: &[TimestampedSample],
    sample_rate: f64,
    window_size: usize,
) -> Option<FftResults> {
    if samples.len() < window_size {
        return None;
    }

    let mut planner = FftPlanner::new();

    // Extract axis data
    let accel_x: Vec<f32> = samples.iter().map(|s| s.data.accel_x_g()).collect();
    let accel_y: Vec<f32> = samples.iter().map(|s| s.data.accel_y_g()).collect();
    let accel_z: Vec<f32> = samples.iter().map(|s| s.data.accel_z_g()).collect();
    let gyro_x: Vec<f32> = samples.iter().map(|s| s.data.gyro_x_dps()).collect();
    let gyro_y: Vec<f32> = samples.iter().map(|s| s.data.gyro_y_dps()).collect();
    let gyro_z: Vec<f32> = samples.iter().map(|s| s.data.gyro_z_dps()).collect();

    // Compute frequency bins
    let freq_resolution = sample_rate / window_size as f64;
    let frequencies: Vec<f64> = (0..window_size / 2)
        .map(|i| i as f64 * freq_resolution)
        .collect();

    Some(FftResults {
        frequencies,
        accel_magnitudes: [
            compute_magnitude(&mut planner, &accel_x, window_size),
            compute_magnitude(&mut planner, &accel_y, window_size),
            compute_magnitude(&mut planner, &accel_z, window_size),
        ],
        gyro_magnitudes: [
            compute_magnitude(&mut planner, &gyro_x, window_size),
            compute_magnitude(&mut planner, &gyro_y, window_size),
            compute_magnitude(&mut planner, &gyro_z, window_size),
        ],
        sample_rate,
        window_size,
    })
}

/// Apply Hann window
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

/// Compute magnitude spectrum for a single axis
fn compute_magnitude(planner: &mut FftPlanner<f64>, data: &[f32], window_size: usize) -> Vec<f64> {
    if data.len() < window_size {
        return vec![];
    }

    // Apply Hann window
    let windowed = apply_hann_window(&data[..window_size]);

    // Convert to complex
    let mut buffer: Vec<Complex<f64>> = windowed
        .iter()
        .map(|&x| Complex::new(x, 0.0))
        .collect();

    // Perform FFT
    let fft = planner.plan_fft_forward(window_size);
    fft.process(&mut buffer);

    // Compute magnitude (first half only)
    buffer
        .iter()
        .take(window_size / 2)
        .map(|c| c.norm() / (window_size as f64))
        .collect()
}
