//! MPU6050 Sensor Data GUI Visualizer
//!
//! Interactive visualization of sensor data with time-series plots and FFT analysis.
//! Supports both live sensor streaming and playback of recorded HDF5 files.

mod app;
mod data;
mod live;
mod state;

use app::SensorGuiApp;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_min_inner_size([800.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "MPU6050 Sensor Visualizer",
        options,
        Box::new(|cc| Ok(Box::new(SensorGuiApp::new(cc)))),
    )
}
