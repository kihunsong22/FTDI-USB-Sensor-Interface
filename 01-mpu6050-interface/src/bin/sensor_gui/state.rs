//! Application state management

use ft232_sensor_interface::{Hdf5Writer, Metadata, SensorData, TimestampedSample};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Instant;

/// Main application state
pub struct AppState {
    /// Current application mode
    pub mode: AppMode,

    /// Live streaming state
    pub live: LiveState,

    /// File playback state
    pub file_path: Option<PathBuf>,
    pub file_data: Option<LoadedData>,

    /// Display data (shared between modes)
    pub display_data: Option<DisplayData>,

    /// FFT analysis results
    pub fft_results: Option<FftResults>,

    /// UI state
    pub ui: UiState,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            mode: AppMode::Idle,
            live: LiveState::default(),
            file_path: None,
            file_data: None,
            display_data: None,
            fft_results: None,
            ui: UiState::default(),
        }
    }
}

/// Application mode
#[derive(Default, PartialEq, Clone, Copy)]
pub enum AppMode {
    #[default]
    Idle,     // No connection, no file
    Live,     // Connected to sensor
    Playback, // Viewing saved file
}

/// Live streaming state
pub struct LiveState {
    /// Sensor thread handle
    pub sensor_handle: Option<SensorHandle>,

    /// Circular buffer for live data
    pub buffer: CircularBuffer,

    /// Recording state
    pub is_recording: bool,
    pub recording_start: Option<Instant>,
    pub recording_samples: usize,
    pub hdf5_writer: Option<Hdf5Writer>,
    pub recording_path: Option<PathBuf>,

    /// Display is paused (buffer still fills)
    pub paused: bool,

    /// Sample rate being used
    pub sample_rate: f64,

    /// Time window to display (seconds)
    pub time_window: f64,
}

impl Default for LiveState {
    fn default() -> Self {
        Self {
            sensor_handle: None,
            buffer: CircularBuffer::new(10000), // ~10 seconds at 1000 Hz
            is_recording: false,
            recording_start: None,
            recording_samples: 0,
            hdf5_writer: None,
            recording_path: None,
            paused: false,
            sample_rate: 850.0, // FIFO mode default
            time_window: 5.0,   // 5 second display window
        }
    }
}

/// Handle to the sensor streaming thread
pub struct SensorHandle {
    pub rx: Receiver<SensorData>,
    pub stop_signal: Arc<AtomicBool>,
    pub thread: Option<JoinHandle<()>>,
    pub start_time: Instant,
}

impl SensorHandle {
    pub fn new(rx: Receiver<SensorData>, stop_signal: Arc<AtomicBool>, thread: JoinHandle<()>) -> Self {
        Self {
            rx,
            stop_signal,
            thread: Some(thread),
            start_time: Instant::now(),
        }
    }

    /// Signal the thread to stop
    pub fn stop(&self) {
        self.stop_signal.store(true, Ordering::SeqCst);
    }

    /// Check if thread is still running
    pub fn is_running(&self) -> bool {
        self.thread.as_ref().map(|t| !t.is_finished()).unwrap_or(false)
    }
}

/// Circular buffer for live sensor data
pub struct CircularBuffer {
    data: VecDeque<TimestampedSample>,
    max_samples: usize,
}

impl CircularBuffer {
    pub fn new(max_samples: usize) -> Self {
        Self {
            data: VecDeque::with_capacity(max_samples),
            max_samples,
        }
    }

    pub fn push(&mut self, sample: TimestampedSample) {
        if self.data.len() >= self.max_samples {
            self.data.pop_front();
        }
        self.data.push_back(sample);
    }

    pub fn clear(&mut self) {
        self.data.clear();
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Get samples within time window (last N seconds)
    pub fn get_window(&self, window_secs: f64) -> Vec<&TimestampedSample> {
        if self.data.is_empty() {
            return vec![];
        }

        let latest_time = self.data.back().map(|s| s.timestamp).unwrap_or(0.0);
        let cutoff = latest_time - window_secs;

        self.data
            .iter()
            .filter(|s| s.timestamp >= cutoff)
            .collect()
    }

    /// Get all samples as slice
    pub fn as_slice(&self) -> &VecDeque<TimestampedSample> {
        &self.data
    }

    /// Get the latest sample
    pub fn latest(&self) -> Option<&TimestampedSample> {
        self.data.back()
    }
}

/// Data loaded from HDF5 file
pub struct LoadedData {
    pub metadata: Metadata,
    pub samples: Vec<TimestampedSample>,
    pub time_range: (f64, f64),
}

/// Downsampled data ready for plotting
#[derive(Clone)]
pub struct DisplayData {
    pub timestamps: Vec<f64>,
    pub accel_x: Vec<f32>,
    pub accel_y: Vec<f32>,
    pub accel_z: Vec<f32>,
    pub gyro_x: Vec<f32>,
    pub gyro_y: Vec<f32>,
    pub gyro_z: Vec<f32>,
}

/// FFT analysis results
#[derive(Clone)]
pub struct FftResults {
    pub frequencies: Vec<f64>,
    pub accel_magnitudes: [Vec<f64>; 3], // X, Y, Z
    pub gyro_magnitudes: [Vec<f64>; 3],  // X, Y, Z
    pub sample_rate: f64,
    pub window_size: usize,
}

/// UI-specific state
pub struct UiState {
    /// Current tab
    pub active_tab: Tab,

    /// Axis visibility
    pub show_accel: [bool; 3],
    pub show_gyro: [bool; 3],

    /// Time range selection for file playback (start, end) in seconds
    pub time_range: (f64, f64),

    /// FFT window size (samples)
    pub fft_window_size: usize,

    /// FFT time window for live mode (seconds)
    pub fft_time_window: f64,

    /// FFT update interval (seconds)
    pub fft_update_interval: f64,

    /// Last FFT update time
    pub fft_last_update: Option<Instant>,

    /// Status message
    pub status: String,

    /// Connection error message
    pub error: Option<String>,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            active_tab: Tab::Live,
            show_accel: [true, true, true],
            show_gyro: [true, true, true],
            time_range: (0.0, 0.0),
            fft_window_size: 2048,
            fft_time_window: 3.0,      // 3 seconds of data for FFT
            fft_update_interval: 1.0,  // Update every 1 second
            fft_last_update: None,
            status: String::from("Ready"),
            error: None,
        }
    }
}

#[derive(Default, PartialEq, Clone, Copy)]
pub enum Tab {
    #[default]
    Live,
    TimeSeries,
    FftAnalysis,
}
