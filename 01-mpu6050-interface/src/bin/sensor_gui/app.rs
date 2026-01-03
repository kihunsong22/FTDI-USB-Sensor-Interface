//! Main GUI application

use crate::data;
use crate::live::{self, ConnectResult};
use crate::state::{AppMode, AppState, DisplayData, Tab};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
use ft232_sensor_interface::{Hdf5Writer, TimestampedSample};
use std::sync::mpsc::TryRecvError;
use std::time::Instant;

/// Main application struct
pub struct SensorGuiApp {
    state: AppState,
}

impl SensorGuiApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            state: AppState::default(),
        }
    }

    /// Poll for new sensor data from the streaming thread
    fn poll_sensor_data(&mut self) {
        if let Some(handle) = &self.state.live.sensor_handle {
            let start_time = handle.start_time;

            // Receive all available samples
            loop {
                match handle.rx.try_recv() {
                    Ok(sensor_data) => {
                        let timestamp = start_time.elapsed().as_secs_f64();
                        let sample = TimestampedSample {
                            timestamp,
                            data: sensor_data,
                        };

                        // Add to circular buffer
                        self.state.live.buffer.push(sample.clone());

                        // Write to HDF5 if recording
                        if self.state.live.is_recording {
                            if let Some(writer) = &mut self.state.live.hdf5_writer {
                                let _ = writer.append_sample(sample);
                            }
                            self.state.live.recording_samples += 1;
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        // Thread died, clean up
                        self.disconnect_sensor();
                        self.state.ui.error = Some("Sensor connection lost".to_string());
                        break;
                    }
                }
            }
        }
    }

    /// Connect to sensor
    fn connect_sensor(&mut self) {
        self.state.ui.error = None;
        self.state.ui.status = "Connecting...".to_string();

        match live::connect_sensor() {
            ConnectResult::Success(handle) => {
                self.state.live.sensor_handle = Some(handle);
                self.state.live.buffer.clear();
                self.state.mode = AppMode::Live;
                self.state.ui.status = "Connected".to_string();
                self.state.ui.active_tab = Tab::Live;
            }
            ConnectResult::Error(e) => {
                self.state.ui.error = Some(e);
                self.state.ui.status = "Connection failed".to_string();
            }
        }
    }

    /// Disconnect from sensor
    fn disconnect_sensor(&mut self) {
        if let Some(handle) = self.state.live.sensor_handle.take() {
            handle.stop();
            // Wait for thread to finish (with timeout)
            if let Some(thread) = handle.thread {
                let _ = thread.join();
            }
        }

        // Stop recording if active
        self.stop_recording();

        self.state.mode = AppMode::Idle;
        self.state.ui.status = "Disconnected".to_string();
    }

    /// Start recording to HDF5
    fn start_recording(&mut self) {
        if self.state.live.is_recording {
            return;
        }

        // Generate filename with timestamp
        let filename = format!(
            "recording_{}.h5",
            chrono::Local::now().format("%Y%m%d_%H%M%S")
        );

        match Hdf5Writer::create(&filename, "fifo", self.state.live.sample_rate) {
            Ok(writer) => {
                self.state.live.hdf5_writer = Some(writer);
                self.state.live.recording_path = Some(filename.into());
                self.state.live.recording_start = Some(Instant::now());
                self.state.live.recording_samples = 0;
                self.state.live.is_recording = true;
                self.state.ui.status = "Recording...".to_string();
            }
            Err(e) => {
                self.state.ui.error = Some(format!("Failed to start recording: {}", e));
            }
        }
    }

    /// Stop recording
    fn stop_recording(&mut self) {
        if !self.state.live.is_recording {
            return;
        }

        if let Some(mut writer) = self.state.live.hdf5_writer.take() {
            let _ = writer.flush();
        }

        self.state.live.is_recording = false;
        self.state.live.recording_start = None;

        if let Some(path) = &self.state.live.recording_path {
            self.state.ui.status = format!(
                "Saved {} samples to {}",
                self.state.live.recording_samples,
                path.display()
            );
        }
    }

    /// Render the top toolbar
    fn render_toolbar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                // Connection controls
                ui.heading("MPU6050 Sensor");
                ui.separator();

                let is_connected = self.state.mode == AppMode::Live;

                if is_connected {
                    if ui.button("⏹ Disconnect").clicked() {
                        self.disconnect_sensor();
                    }
                    ui.label("🟢 Connected");
                } else {
                    if ui.button("▶ Connect Sensor").clicked() {
                        self.connect_sensor();
                    }
                }

                ui.separator();

                // File controls
                if ui.button("📂 Open File").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("HDF5 Files", &["h5", "hdf5"])
                        .pick_file()
                    {
                        self.load_file(&path);
                    }
                }

                ui.separator();

                // Recording controls (only when connected)
                if is_connected {
                    if self.state.live.is_recording {
                        if ui.button("⏹ Stop Recording").clicked() {
                            self.stop_recording();
                        }

                        // Show recording indicator
                        let elapsed = self.state.live.recording_start
                            .map(|s| s.elapsed().as_secs())
                            .unwrap_or(0);
                        let mins = elapsed / 60;
                        let secs = elapsed % 60;
                        ui.label(format!(
                            "🔴 REC {:02}:{:02} | {} samples",
                            mins, secs, self.state.live.recording_samples
                        ));
                    } else {
                        if ui.button("⏺ Start Recording").clicked() {
                            self.start_recording();
                        }
                    }
                }

                // Status on the right
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(error) = &self.state.ui.error {
                        ui.colored_label(egui::Color32::RED, error);
                    } else {
                        ui.label(&self.state.ui.status);
                    }
                });
            });
        });
    }

    /// Render the left sidebar
    fn render_sidebar(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("sidebar")
            .resizable(true)
            .default_width(180.0)
            .show(ctx, |ui| {
                // Mode indicator
                ui.heading("Mode");
                match self.state.mode {
                    AppMode::Idle => ui.label("⚪ Idle"),
                    AppMode::Live => ui.label("🟢 Live Streaming"),
                    AppMode::Playback => ui.label("📁 File Playback"),
                };
                ui.separator();

                // Metadata
                ui.heading("Info");
                match self.state.mode {
                    AppMode::Live => {
                        ui.label(format!("Rate: ~{:.0} Hz", self.state.live.sample_rate));
                        ui.label(format!("Buffer: {} samples", self.state.live.buffer.len()));
                        if let Some(sample) = self.state.live.buffer.latest() {
                            ui.label(format!("Time: {:.1}s", sample.timestamp));
                        }
                    }
                    AppMode::Playback => {
                        if let Some(data) = &self.state.file_data {
                            ui.label(format!("Mode: {}", data.metadata.acquisition_mode));
                            ui.label(format!("Rate: {:.0} Hz", data.metadata.sample_rate_hz));
                            ui.label(format!("Samples: {}", data.samples.len()));
                            ui.label(format!(
                                "Duration: {:.1}s",
                                data.time_range.1 - data.time_range.0
                            ));
                        }
                    }
                    AppMode::Idle => {
                        ui.label("No data");
                    }
                }
                ui.separator();

                // Axis toggles
                ui.heading("Axes");
                ui.horizontal(|ui| {
                    ui.label("Accel:");
                    ui.checkbox(&mut self.state.ui.show_accel[0], "X");
                    ui.checkbox(&mut self.state.ui.show_accel[1], "Y");
                    ui.checkbox(&mut self.state.ui.show_accel[2], "Z");
                });
                ui.horizontal(|ui| {
                    ui.label("Gyro:");
                    ui.checkbox(&mut self.state.ui.show_gyro[0], "X");
                    ui.checkbox(&mut self.state.ui.show_gyro[1], "Y");
                    ui.checkbox(&mut self.state.ui.show_gyro[2], "Z");
                });
                ui.separator();

                // Live controls
                if self.state.mode == AppMode::Live {
                    ui.heading("Live View");
                    ui.horizontal(|ui| {
                        ui.label("Window:");
                        ui.add(
                            egui::Slider::new(&mut self.state.live.time_window, 1.0..=30.0)
                                .suffix("s")
                                .logarithmic(true),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.state.live.paused, "Pause display");
                    });
                    if ui.button("Clear buffer").clicked() {
                        self.state.live.buffer.clear();
                    }
                    ui.separator();
                }

                // FFT settings
                if self.state.ui.active_tab == Tab::FftAnalysis {
                    ui.heading("FFT Settings");
                    ui.horizontal(|ui| {
                        ui.label("Samples:");
                        for &size in &[512, 1024, 2048, 4096] {
                            if ui
                                .selectable_label(
                                    self.state.ui.fft_window_size == size,
                                    format!("{}", size),
                                )
                                .clicked()
                            {
                                self.state.ui.fft_window_size = size;
                                self.state.ui.fft_last_update = None; // Force update
                            }
                        }
                    });

                    // Live FFT settings
                    if self.state.mode == AppMode::Live {
                        ui.separator();
                        ui.label("Live FFT");
                        ui.horizontal(|ui| {
                            ui.label("Window:");
                            ui.add(
                                egui::Slider::new(&mut self.state.ui.fft_time_window, 1.0..=10.0)
                                    .suffix("s")
                            );
                        });
                        ui.horizontal(|ui| {
                            ui.label("Update:");
                            ui.add(
                                egui::Slider::new(&mut self.state.ui.fft_update_interval, 0.5..=5.0)
                                    .suffix("s")
                            );
                        });
                        if ui.button("Update Now").clicked() {
                            self.state.ui.fft_last_update = None; // Force update
                        }
                    }
                }
            });
    }

    /// Render the bottom panel with time controls (for file playback)
    fn render_bottom_panel(&mut self, ctx: &egui::Context) {
        if self.state.mode != AppMode::Playback {
            return;
        }

        let full_range = self.state.file_data.as_ref().map(|d| d.time_range);
        let mut new_range = None;

        egui::TopBottomPanel::bottom("controls").show(ctx, |ui| {
            if let Some(full) = full_range {
                ui.horizontal(|ui| {
                    ui.label("Time Range:");

                    let mut range = self.state.ui.time_range;

                    ui.add(
                        egui::Slider::new(&mut range.0, full.0..=full.1)
                            .text("Start")
                            .suffix("s"),
                    );

                    ui.add(
                        egui::Slider::new(&mut range.1, full.0..=full.1)
                            .text("End")
                            .suffix("s"),
                    );

                    if ui.button("Reset").clicked() {
                        range = full;
                    }

                    if range != self.state.ui.time_range && range.0 < range.1 {
                        new_range = Some(range);
                    }
                });
            }
        });

        if let Some(range) = new_range {
            self.state.ui.time_range = range;
            self.update_display_data();
        }
    }

    /// Render the main content area with tabs
    fn render_main_content(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            // Tab bar
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.state.ui.active_tab, Tab::Live, "📡 Live View");
                ui.selectable_value(&mut self.state.ui.active_tab, Tab::TimeSeries, "📈 Time Series");
                ui.selectable_value(&mut self.state.ui.active_tab, Tab::FftAnalysis, "📊 FFT Analysis");
            });
            ui.separator();

            // Tab content
            match self.state.ui.active_tab {
                Tab::Live => self.render_live_view(ui),
                Tab::TimeSeries => self.render_time_series(ui),
                Tab::FftAnalysis => self.render_fft_analysis(ui),
            }
        });
    }

    /// Render live view with scrolling plots and current values
    fn render_live_view(&mut self, ui: &mut egui::Ui) {
        if self.state.mode != AppMode::Live {
            ui.centered_and_justified(|ui| {
                ui.vertical_centered(|ui| {
                    ui.heading("No Sensor Connected");
                    ui.label("Click 'Connect Sensor' to start live streaming.");
                });
            });
            return;
        }

        // Get samples for display
        let window = self.state.live.time_window;
        let samples: Vec<_> = self.state.live.buffer.get_window(window);

        if samples.is_empty() {
            ui.centered_and_justified(|ui| {
                ui.label("Waiting for data...");
            });
            return;
        }

        // Current values display
        if let Some(latest) = samples.last() {
            let (ax, ay, az) = latest.data.accel_to_g();
            let (gx, gy, gz) = latest.data.gyro_to_dps();

            ui.horizontal(|ui| {
                ui.label("Current Values:");
                ui.colored_label(egui::Color32::from_rgb(255, 100, 100), format!("Ax={:+.3}g", ax));
                ui.colored_label(egui::Color32::from_rgb(100, 255, 100), format!("Ay={:+.3}g", ay));
                ui.colored_label(egui::Color32::from_rgb(100, 100, 255), format!("Az={:+.3}g", az));
                ui.separator();
                ui.colored_label(egui::Color32::from_rgb(255, 150, 150), format!("Gx={:+.1}°/s", gx));
                ui.colored_label(egui::Color32::from_rgb(150, 255, 150), format!("Gy={:+.1}°/s", gy));
                ui.colored_label(egui::Color32::from_rgb(150, 150, 255), format!("Gz={:+.1}°/s", gz));
            });
            ui.separator();
        }

        let available_height = ui.available_height();

        // Accelerometer plot
        ui.label("Accelerometer (g)");
        let accel_plot = Plot::new("live_accel")
            .height(available_height * 0.45)
            .allow_zoom(false)
            .allow_drag(false)
            .include_y(-2.0)
            .include_y(2.0)
            .x_axis_label("Time (s)")
            .legend(egui_plot::Legend::default());

        accel_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 100, 100),
                egui::Color32::from_rgb(100, 255, 100),
                egui::Color32::from_rgb(100, 100, 255),
            ];
            let labels = ["X", "Y", "Z"];

            for (i, show) in self.state.ui.show_accel.iter().enumerate() {
                if *show {
                    let points: PlotPoints = samples
                        .iter()
                        .map(|s| {
                            let (ax, ay, az) = s.data.accel_to_g();
                            let v = [ax, ay, az][i];
                            [s.timestamp, v as f64]
                        })
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });

        ui.add_space(5.0);

        // Gyroscope plot
        ui.label("Gyroscope (°/s)");
        let gyro_plot = Plot::new("live_gyro")
            .height(available_height * 0.45)
            .allow_zoom(false)
            .allow_drag(false)
            .include_y(-250.0)
            .include_y(250.0)
            .x_axis_label("Time (s)")
            .legend(egui_plot::Legend::default());

        gyro_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 150, 150),
                egui::Color32::from_rgb(150, 255, 150),
                egui::Color32::from_rgb(150, 150, 255),
            ];
            let labels = ["X", "Y", "Z"];

            for (i, show) in self.state.ui.show_gyro.iter().enumerate() {
                if *show {
                    let points: PlotPoints = samples
                        .iter()
                        .map(|s| {
                            let (gx, gy, gz) = s.data.gyro_to_dps();
                            let v = [gx, gy, gz][i];
                            [s.timestamp, v as f64]
                        })
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });
    }

    /// Render time series plots (for file playback or paused live)
    fn render_time_series(&self, ui: &mut egui::Ui) {
        let display = match self.state.mode {
            AppMode::Playback => self.state.display_data.as_ref(),
            AppMode::Live => {
                // Show live buffer as time series when paused
                return self.render_live_buffer_as_timeseries(ui);
            }
            AppMode::Idle => None,
        };

        let Some(display) = display else {
            ui.centered_and_justified(|ui| {
                ui.label("No data loaded. Open an HDF5 file or connect to sensor.");
            });
            return;
        };

        if display.timestamps.is_empty() {
            ui.label("No data in selected time range");
            return;
        }

        let available_height = ui.available_height();

        // Accelerometer plot
        ui.heading("Accelerometer (g)");
        let accel_plot = Plot::new("accel_plot")
            .height(available_height * 0.45)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Time (s)")
            .y_axis_label("Acceleration (g)")
            .legend(egui_plot::Legend::default());

        accel_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 100, 100),
                egui::Color32::from_rgb(100, 255, 100),
                egui::Color32::from_rgb(100, 100, 255),
            ];
            let labels = ["X", "Y", "Z"];
            let data_arrays = [&display.accel_x, &display.accel_y, &display.accel_z];

            for (i, (data, show)) in data_arrays
                .iter()
                .zip(self.state.ui.show_accel.iter())
                .enumerate()
            {
                if *show {
                    let points: PlotPoints = display
                        .timestamps
                        .iter()
                        .zip(data.iter())
                        .map(|(&t, &v)| [t, v as f64])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });

        ui.add_space(10.0);

        // Gyroscope plot
        ui.heading("Gyroscope (°/s)");
        let gyro_plot = Plot::new("gyro_plot")
            .height(available_height * 0.45)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Time (s)")
            .y_axis_label("Angular Rate (°/s)")
            .legend(egui_plot::Legend::default());

        gyro_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 150, 150),
                egui::Color32::from_rgb(150, 255, 150),
                egui::Color32::from_rgb(150, 150, 255),
            ];
            let labels = ["X", "Y", "Z"];
            let data_arrays = [&display.gyro_x, &display.gyro_y, &display.gyro_z];

            for (i, (data, show)) in data_arrays
                .iter()
                .zip(self.state.ui.show_gyro.iter())
                .enumerate()
            {
                if *show {
                    let points: PlotPoints = display
                        .timestamps
                        .iter()
                        .zip(data.iter())
                        .map(|(&t, &v)| [t, v as f64])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });
    }

    /// Render live buffer as time series (when on Time Series tab)
    fn render_live_buffer_as_timeseries(&self, ui: &mut egui::Ui) {
        let samples: Vec<_> = self.state.live.buffer.as_slice().iter().collect();

        if samples.is_empty() {
            ui.centered_and_justified(|ui| {
                ui.label("No data in buffer. Connect to sensor to collect data.");
            });
            return;
        }

        // Convert to display format
        let display = DisplayData {
            timestamps: samples.iter().map(|s| s.timestamp).collect(),
            accel_x: samples.iter().map(|s| s.data.accel_x_g()).collect(),
            accel_y: samples.iter().map(|s| s.data.accel_y_g()).collect(),
            accel_z: samples.iter().map(|s| s.data.accel_z_g()).collect(),
            gyro_x: samples.iter().map(|s| s.data.gyro_x_dps()).collect(),
            gyro_y: samples.iter().map(|s| s.data.gyro_y_dps()).collect(),
            gyro_z: samples.iter().map(|s| s.data.gyro_z_dps()).collect(),
        };

        let available_height = ui.available_height();

        ui.label(format!("Showing {} samples from live buffer", samples.len()));

        // Accelerometer plot
        ui.heading("Accelerometer (g)");
        let accel_plot = Plot::new("buffer_accel")
            .height(available_height * 0.43)
            .allow_zoom(true)
            .allow_drag(true)
            .legend(egui_plot::Legend::default());

        accel_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 100, 100),
                egui::Color32::from_rgb(100, 255, 100),
                egui::Color32::from_rgb(100, 100, 255),
            ];
            let labels = ["X", "Y", "Z"];
            let data_arrays = [&display.accel_x, &display.accel_y, &display.accel_z];

            for (i, (data, show)) in data_arrays
                .iter()
                .zip(self.state.ui.show_accel.iter())
                .enumerate()
            {
                if *show {
                    let points: PlotPoints = display
                        .timestamps
                        .iter()
                        .zip(data.iter())
                        .map(|(&t, &v)| [t, v as f64])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });

        ui.add_space(5.0);

        // Gyroscope plot
        ui.heading("Gyroscope (°/s)");
        let gyro_plot = Plot::new("buffer_gyro")
            .height(available_height * 0.43)
            .allow_zoom(true)
            .allow_drag(true)
            .legend(egui_plot::Legend::default());

        gyro_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 150, 150),
                egui::Color32::from_rgb(150, 255, 150),
                egui::Color32::from_rgb(150, 150, 255),
            ];
            let labels = ["X", "Y", "Z"];
            let data_arrays = [&display.gyro_x, &display.gyro_y, &display.gyro_z];

            for (i, (data, show)) in data_arrays
                .iter()
                .zip(self.state.ui.show_gyro.iter())
                .enumerate()
            {
                if *show {
                    let points: PlotPoints = display
                        .timestamps
                        .iter()
                        .zip(data.iter())
                        .map(|(&t, &v)| [t, v as f64])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.5));
                }
            }
        });
    }

    /// Render FFT analysis
    fn render_fft_analysis(&mut self, ui: &mut egui::Ui) {
        // Compute FFT if needed (first time or for playback mode)
        if self.state.fft_results.is_none() {
            self.recompute_fft();
        }

        let Some(fft) = &self.state.fft_results else {
            ui.centered_and_justified(|ui| {
                ui.label("Insufficient data for FFT. Need at least 512 samples.");
            });
            return;
        };

        if fft.frequencies.is_empty() {
            ui.label("Insufficient samples for FFT analysis");
            return;
        }

        let available_height = ui.available_height();

        ui.horizontal(|ui| {
            ui.label(format!(
                "Window: {} samples | Resolution: {:.2} Hz | Nyquist: {:.0} Hz",
                fft.window_size,
                fft.sample_rate / fft.window_size as f64,
                fft.sample_rate / 2.0
            ));

            // Show live update status
            if self.state.mode == AppMode::Live {
                ui.separator();
                if let Some(last_update) = self.state.ui.fft_last_update {
                    let elapsed = last_update.elapsed().as_secs_f64();
                    let next_in = (self.state.ui.fft_update_interval - elapsed).max(0.0);
                    ui.label(format!("Next update: {:.1}s", next_in));
                }
            }
        });
        ui.separator();

        // Accelerometer FFT
        ui.heading("Accelerometer Frequency Spectrum");
        let accel_fft_plot = Plot::new("accel_fft")
            .height(available_height * 0.42)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Frequency (Hz)")
            .y_axis_label("Magnitude")
            .legend(egui_plot::Legend::default());

        accel_fft_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 100, 100),
                egui::Color32::from_rgb(100, 255, 100),
                egui::Color32::from_rgb(100, 100, 255),
            ];
            let labels = ["X", "Y", "Z"];

            for (i, (mag, show)) in fft
                .accel_magnitudes
                .iter()
                .zip(self.state.ui.show_accel.iter())
                .enumerate()
            {
                if *show && !mag.is_empty() {
                    let points: PlotPoints = fft
                        .frequencies
                        .iter()
                        .zip(mag.iter())
                        .map(|(&f, &m)| [f, m])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.0));
                }
            }
        });

        ui.add_space(10.0);

        // Gyroscope FFT
        ui.heading("Gyroscope Frequency Spectrum");
        let gyro_fft_plot = Plot::new("gyro_fft")
            .height(available_height * 0.42)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Frequency (Hz)")
            .y_axis_label("Magnitude")
            .legend(egui_plot::Legend::default());

        gyro_fft_plot.show(ui, |plot_ui| {
            let colors = [
                egui::Color32::from_rgb(255, 150, 150),
                egui::Color32::from_rgb(150, 255, 150),
                egui::Color32::from_rgb(150, 150, 255),
            ];
            let labels = ["X", "Y", "Z"];

            for (i, (mag, show)) in fft
                .gyro_magnitudes
                .iter()
                .zip(self.state.ui.show_gyro.iter())
                .enumerate()
            {
                if *show && !mag.is_empty() {
                    let points: PlotPoints = fft
                        .frequencies
                        .iter()
                        .zip(mag.iter())
                        .map(|(&f, &m)| [f, m])
                        .collect();
                    plot_ui.line(Line::new(points).name(labels[i]).color(colors[i]).width(1.0));
                }
            }
        });
    }

    /// Load a file
    fn load_file(&mut self, path: &std::path::Path) {
        // Disconnect sensor if connected
        if self.state.mode == AppMode::Live {
            self.disconnect_sensor();
        }

        self.state.ui.status = format!("Loading {}...", path.display());
        self.state.ui.error = None;

        match data::load_file(path) {
            Ok(loaded) => {
                self.state.ui.time_range = loaded.time_range;
                self.state.file_path = Some(path.to_path_buf());
                self.state.file_data = Some(loaded);
                self.state.fft_results = None;
                self.state.mode = AppMode::Playback;
                self.state.ui.active_tab = Tab::TimeSeries;
                self.update_display_data();
                self.state.ui.status = "File loaded".to_string();
            }
            Err(e) => {
                self.state.ui.error = Some(format!("Failed to load file: {}", e));
            }
        }
    }

    /// Update display data after time range change
    fn update_display_data(&mut self) {
        if let Some(file_data) = &self.state.file_data {
            self.state.display_data =
                Some(data::downsample(&file_data.samples, self.state.ui.time_range));
            self.state.fft_results = None;
        }
    }

    /// Recompute FFT
    fn recompute_fft(&mut self) {
        let samples: Vec<TimestampedSample> = match self.state.mode {
            AppMode::Playback => {
                if let Some(file_data) = &self.state.file_data {
                    file_data
                        .samples
                        .iter()
                        .filter(|s| {
                            s.timestamp >= self.state.ui.time_range.0
                                && s.timestamp <= self.state.ui.time_range.1
                        })
                        .cloned()
                        .collect()
                } else {
                    return;
                }
            }
            AppMode::Live => {
                // Use the FFT time window for live mode
                self.state.live.buffer
                    .get_window(self.state.ui.fft_time_window)
                    .into_iter()
                    .cloned()
                    .collect()
            }
            AppMode::Idle => return,
        };

        let sample_rate = match self.state.mode {
            AppMode::Playback => self
                .state
                .file_data
                .as_ref()
                .map(|d| d.metadata.sample_rate_hz)
                .unwrap_or(850.0),
            AppMode::Live => self.state.live.sample_rate,
            AppMode::Idle => 850.0,
        };

        self.state.fft_results =
            data::compute_fft(&samples, sample_rate, self.state.ui.fft_window_size);
        self.state.ui.fft_last_update = Some(Instant::now());
    }

    /// Check if FFT needs updating (for live mode)
    fn maybe_update_live_fft(&mut self) {
        if self.state.mode != AppMode::Live {
            return;
        }
        if self.state.ui.active_tab != Tab::FftAnalysis {
            return;
        }

        let should_update = match self.state.ui.fft_last_update {
            None => true,
            Some(last) => last.elapsed().as_secs_f64() >= self.state.ui.fft_update_interval,
        };

        if should_update {
            self.recompute_fft();
        }
    }
}

impl eframe::App for SensorGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Poll for sensor data
        self.poll_sensor_data();

        // Check if live FFT needs updating
        self.maybe_update_live_fft();

        // Request repaint for live updates
        if self.state.mode == AppMode::Live && !self.state.live.paused {
            ctx.request_repaint();
        }

        // Render UI
        self.render_toolbar(ctx);
        self.render_sidebar(ctx);
        self.render_bottom_panel(ctx);
        self.render_main_content(ctx);
    }
}
