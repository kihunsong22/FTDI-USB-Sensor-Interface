//! HDF5 file format for ADXL355 sensor data storage

use crate::{Adxl355Error, Result, SensorData};
use hdf5::{Dataset, File, Group};
use std::path::Path;
use std::time::Instant;

/// Sample with timestamp
#[derive(Debug, Clone)]
pub struct TimestampedSample {
    pub timestamp: f64,
    pub data: SensorData,
}

/// Metadata stored in HDF5 file
#[derive(Debug, Clone)]
pub struct Metadata {
    pub start_time: String,
    pub sample_rate_hz: f64,
    pub acquisition_mode: String,
    pub sensor_type: String,
    pub range: String,
    pub version: String,
}

/// Handles for HDF5 datasets
struct DatasetHandles {
    timestamps: Dataset,
    accel_x: Dataset,
    accel_y: Dataset,
    accel_z: Dataset,
    temperature: Dataset,
}

/// HDF5 writer for sensor data collection
pub struct Hdf5Writer {
    file: File,
    datasets: DatasetHandles,
    start_time: Instant,
    sample_count: usize,
}

impl Hdf5Writer {
    /// Create a new HDF5 file for data collection
    pub fn create<P: AsRef<Path>>(path: P, mode: &str, rate: f64, range: &str) -> Result<Self> {
        let file = File::create(path)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to create HDF5 file: {}", e)))?;

        // Create metadata group
        let metadata_group = file.create_group("metadata")
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to create metadata group: {}", e)))?;

        // Write metadata attributes
        let start_time = chrono::Local::now().to_rfc3339();
        let write_str_attr = |group: &Group, name: &str, value: &str| -> Result<()> {
            let vlu: hdf5::types::VarLenUnicode = value.parse().unwrap();
            group.new_attr::<hdf5::types::VarLenUnicode>()
                .create(name)
                .and_then(|attr| attr.write_scalar(&vlu))
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to write {}: {}", name, e)))
        };

        write_str_attr(&metadata_group, "start_time", &start_time)?;
        write_str_attr(&metadata_group, "acquisition_mode", mode)?;
        write_str_attr(&metadata_group, "sensor_type", "adxl355")?;
        write_str_attr(&metadata_group, "range", range)?;
        write_str_attr(&metadata_group, "version", "1.0")?;

        metadata_group.new_attr::<f64>()
            .create("sample_rate_hz")
            .and_then(|attr| attr.write_scalar(&rate))
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to write sample_rate_hz: {}", e)))?;

        // Create sensor_data group
        let data_group = file.create_group("sensor_data")
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to create sensor_data group: {}", e)))?;

        let chunk_size = 1024;

        let timestamps = Self::create_dataset::<f64>(&data_group, "timestamps", chunk_size)?;
        let accel_x = Self::create_dataset::<i32>(&data_group, "accel_x", chunk_size)?;
        let accel_y = Self::create_dataset::<i32>(&data_group, "accel_y", chunk_size)?;
        let accel_z = Self::create_dataset::<i32>(&data_group, "accel_z", chunk_size)?;
        let temperature = Self::create_dataset::<u16>(&data_group, "temperature", chunk_size)?;

        let datasets = DatasetHandles {
            timestamps,
            accel_x,
            accel_y,
            accel_z,
            temperature,
        };

        Ok(Self {
            file,
            datasets,
            start_time: Instant::now(),
            sample_count: 0,
        })
    }

    fn create_dataset<T: hdf5::H5Type>(group: &Group, name: &str, chunk_size: usize) -> Result<Dataset> {
        group.new_dataset::<T>()
            .shape((0..,))
            .chunk((chunk_size,))
            .deflate(4)
            .create(name)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to create dataset {}: {}", name, e)))
    }

    /// Append a single sample
    pub fn append_sample(&mut self, sample: TimestampedSample) -> Result<()> {
        self.append_batch(&[sample])
    }

    /// Append a batch of samples
    pub fn append_batch(&mut self, samples: &[TimestampedSample]) -> Result<()> {
        if samples.is_empty() {
            return Ok(());
        }

        let new_size = self.sample_count + samples.len();

        let timestamps: Vec<f64> = samples.iter().map(|s| s.timestamp).collect();
        let accel_x: Vec<i32> = samples.iter().map(|s| s.data.accel_x).collect();
        let accel_y: Vec<i32> = samples.iter().map(|s| s.data.accel_y).collect();
        let accel_z: Vec<i32> = samples.iter().map(|s| s.data.accel_z).collect();
        let temperature: Vec<u16> = samples.iter().map(|s| s.data.temperature).collect();

        self.append_to_dataset(&self.datasets.timestamps, new_size, &timestamps)?;
        self.append_to_dataset(&self.datasets.accel_x, new_size, &accel_x)?;
        self.append_to_dataset(&self.datasets.accel_y, new_size, &accel_y)?;
        self.append_to_dataset(&self.datasets.accel_z, new_size, &accel_z)?;
        self.append_to_dataset(&self.datasets.temperature, new_size, &temperature)?;

        self.sample_count = new_size;
        Ok(())
    }

    fn append_to_dataset<T: hdf5::H5Type>(&self, dataset: &Dataset, new_size: usize, data: &[T]) -> Result<()> {
        dataset.resize((new_size,))
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to resize dataset: {}", e)))?;

        let start = new_size - data.len();
        dataset.write_slice(data, start..)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to write to dataset: {}", e)))?;

        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.file.flush()
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to flush HDF5 file: {}", e)))?;
        Ok(())
    }

    pub fn sample_count(&self) -> usize {
        self.sample_count
    }

    pub fn elapsed_secs(&self) -> f64 {
        self.start_time.elapsed().as_secs_f64()
    }
}

/// HDF5 reader for accessing collected sensor data
pub struct Hdf5Reader {
    #[allow(dead_code)]
    file: File,
    datasets: DatasetHandles,
    metadata: Metadata,
}

impl Hdf5Reader {
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open HDF5 file: {}", e)))?;

        let metadata = Self::read_metadata(&file)?;

        let data_group = file.group("sensor_data")
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open sensor_data group: {}", e)))?;

        let datasets = DatasetHandles {
            timestamps: data_group.dataset("timestamps")
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open timestamps: {}", e)))?,
            accel_x: data_group.dataset("accel_x")
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open accel_x: {}", e)))?,
            accel_y: data_group.dataset("accel_y")
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open accel_y: {}", e)))?,
            accel_z: data_group.dataset("accel_z")
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open accel_z: {}", e)))?,
            temperature: data_group.dataset("temperature")
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open temperature: {}", e)))?,
        };

        Ok(Self { file, datasets, metadata })
    }

    fn read_metadata(file: &File) -> Result<Metadata> {
        let group = file.group("metadata")
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to open metadata: {}", e)))?;

        let read_str = |name: &str| -> Result<String> {
            group.attr(name)
                .and_then(|attr| attr.read_scalar::<hdf5::types::VarLenUnicode>())
                .map(|s| s.to_string())
                .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read {}: {}", name, e)))
        };

        let start_time = read_str("start_time")?;
        let acquisition_mode = read_str("acquisition_mode")?;
        let sensor_type = read_str("sensor_type")?;
        let range = read_str("range")?;
        let version = read_str("version")?;

        let sample_rate_hz = group.attr("sample_rate_hz")
            .and_then(|attr| attr.read_scalar::<f64>())
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read sample_rate_hz: {}", e)))?;

        Ok(Metadata {
            start_time,
            sample_rate_hz,
            acquisition_mode,
            sensor_type,
            range,
            version,
        })
    }

    pub fn metadata(&self) -> &Metadata {
        &self.metadata
    }

    pub fn get_total_samples(&self) -> Result<usize> {
        Ok(self.datasets.timestamps.size())
    }

    pub fn read_range(&self, start: usize, count: usize) -> Result<Vec<TimestampedSample>> {
        let total = self.get_total_samples()?;
        if start >= total {
            return Ok(Vec::new());
        }

        let actual_count = count.min(total - start);
        let end = start + actual_count;

        let timestamps: Vec<f64> = self.datasets.timestamps.read_slice_1d(start..end)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read timestamps: {}", e)))?
            .to_vec();
        let accel_x: Vec<i32> = self.datasets.accel_x.read_slice_1d(start..end)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read accel_x: {}", e)))?
            .to_vec();
        let accel_y: Vec<i32> = self.datasets.accel_y.read_slice_1d(start..end)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read accel_y: {}", e)))?
            .to_vec();
        let accel_z: Vec<i32> = self.datasets.accel_z.read_slice_1d(start..end)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read accel_z: {}", e)))?
            .to_vec();
        let temperature: Vec<u16> = self.datasets.temperature.read_slice_1d(start..end)
            .map_err(|e| Adxl355Error::CommunicationError(format!("Failed to read temperature: {}", e)))?
            .to_vec();

        let samples = timestamps.into_iter()
            .zip(accel_x)
            .zip(accel_y)
            .zip(accel_z)
            .zip(temperature)
            .map(|((((ts, ax), ay), az), temp)| {
                TimestampedSample {
                    timestamp: ts,
                    data: SensorData {
                        accel_x: ax,
                        accel_y: ay,
                        accel_z: az,
                        temperature: temp,
                    },
                }
            })
            .collect();

        Ok(samples)
    }

    pub fn read_latest(&self, count: usize) -> Result<Vec<TimestampedSample>> {
        let total = self.get_total_samples()?;
        if total == 0 {
            return Ok(Vec::new());
        }
        let start = total.saturating_sub(count);
        self.read_range(start, count)
    }
}
