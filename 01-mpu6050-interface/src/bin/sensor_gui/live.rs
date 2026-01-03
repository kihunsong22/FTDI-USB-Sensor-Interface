//! Live sensor streaming thread management

use crate::state::SensorHandle;
use ft232_sensor_interface::{Mpu6050, SensorData, StreamControl};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;
use std::thread;

/// Result of attempting to connect to sensor
pub enum ConnectResult {
    Success(SensorHandle),
    Error(String),
}

/// Connect to sensor and start FIFO streaming
///
/// Returns a SensorHandle that can be used to receive samples
/// and control the streaming thread.
pub fn connect_sensor() -> ConnectResult {
    let (tx, rx) = mpsc::channel::<SensorData>();
    let stop_signal = Arc::new(AtomicBool::new(false));
    let stop_clone = stop_signal.clone();

    // Try to initialize sensor on main thread first to get immediate error feedback
    let sensor_init = Mpu6050::new(0);
    if let Err(e) = sensor_init {
        return ConnectResult::Error(format!("Failed to connect: {}", e));
    }

    let thread = thread::spawn(move || {
        run_sensor_thread(tx, stop_clone);
    });

    ConnectResult::Success(SensorHandle::new(rx, stop_signal, thread))
}

/// Sensor thread main loop
fn run_sensor_thread(tx: Sender<SensorData>, stop_signal: Arc<AtomicBool>) {
    // Initialize sensor
    let sensor = match Mpu6050::new(0) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Sensor thread: Failed to initialize: {}", e);
            return;
        }
    };

    // Enable FIFO mode for high-speed sampling
    let mut sensor = sensor;
    if let Err(e) = sensor.enable_fifo(1000) {
        eprintln!("Sensor thread: Failed to enable FIFO: {}", e);
        // Fall back to polling mode
        run_polling_mode(sensor, tx, stop_signal);
        return;
    }

    // Run FIFO streaming
    run_fifo_mode(sensor, tx, stop_signal);
}

/// Run in FIFO mode (~850 Hz)
fn run_fifo_mode(mut sensor: Mpu6050, tx: Sender<SensorData>, stop_signal: Arc<AtomicBool>) {
    let result = sensor.stream_fifo(20, |batch| {
        if stop_signal.load(Ordering::Relaxed) {
            return StreamControl::Break;
        }

        for sample in batch {
            if tx.send(*sample).is_err() {
                // Receiver dropped, stop streaming
                return StreamControl::Break;
            }
        }

        StreamControl::Continue
    });

    if let Err(e) = result {
        eprintln!("Sensor thread: FIFO stream error: {}", e);
    }

    // Disable FIFO on exit
    let _ = sensor.disable_fifo();
}

/// Run in polling mode (~100 Hz) as fallback
fn run_polling_mode(mut sensor: Mpu6050, tx: Sender<SensorData>, stop_signal: Arc<AtomicBool>) {
    let result = sensor.stream(100, |data| {
        if stop_signal.load(Ordering::Relaxed) {
            return StreamControl::Break;
        }

        if tx.send(data).is_err() {
            return StreamControl::Break;
        }

        StreamControl::Continue
    });

    if let Err(e) = result {
        eprintln!("Sensor thread: Polling stream error: {}", e);
    }
}
