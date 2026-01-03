//! HDF5 file format for sensor data storage
//!
//! Provides writer and reader interfaces for storing MPU6050 sensor data
//! in HDF5 format.

use crate::{Mpu6050Error, Result, SensorData};
use hdf5::{Dataset, File, Group};
use std::path::Path;
use std::time::Instant;

/// Sample with timestamp
#[derive(Debug, Clone)]
pub struct TimestampedSample {
    pub timestamp: f64,  // Seconds since collection start
    pub data: SensorData,
}

/// Metadata stored in HDF5 file
#[derive(Debug, Clone)]
pub struct Metadata {
    pub start_time: String,      // ISO 8601 timestamp
    pub sample_rate_hz: f64,     // Target sample rate
    pub acquisition_mode: String, // "polling" or "fifo"
    pub version: String,         // Format version
}

/// Handles for HDF5 datasets
struct DatasetHandles {
    timestamps: Dataset,
    accel_x: Dataset,
    accel_y: Dataset,
    accel_z: Dataset,
    gyro_x: Dataset,
    gyro_y: Dataset,
    gyro_z: Dataset,
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
    ///
    /// # Arguments
    /// * `path` - File path
    /// * `mode` - Acquisition mode ("polling" or "fifo")
    /// * `rate` - Target sample rate in Hz
    pub fn create<P: AsRef<Path>>(path: P, mode: &str, rate: f64) -> Result<Self> {
        // Create HDF5 file
        let file = File::create(path)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to create HDF5 file: {}", e)))?;

        // Create metadata group
        let metadata_group = file.create_group("metadata")
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to create metadata group: {}", e)))?;

        // Write metadata attributes
        let start_time = chrono::Local::now().to_rfc3339();
        let start_time_vlu: hdf5::types::VarLenUnicode = start_time.parse().unwrap();
        metadata_group.new_attr::<hdf5::types::VarLenUnicode>()
            .create("start_time")
            .and_then(|attr| attr.write_scalar(&start_time_vlu))
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to write start_time: {}", e)))?;

        metadata_group.new_attr::<f64>()
            .create("sample_rate_hz")
            .and_then(|attr| attr.write_scalar(&rate))
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to write sample_rate_hz: {}", e)))?;

        let mode_vlu: hdf5::types::VarLenUnicode = mode.parse().unwrap();
        metadata_group.new_attr::<hdf5::types::VarLenUnicode>()
            .create("acquisition_mode")
            .and_then(|attr| attr.write_scalar(&mode_vlu))
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to write acquisition_mode: {}", e)))?;

        let version_vlu: hdf5::types::VarLenUnicode = "1.0".parse().unwrap();
        metadata_group.new_attr::<hdf5::types::VarLenUnicode>()
            .create("version")
            .and_then(|attr| attr.write_scalar(&version_vlu))
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to write version: {}", e)))?;

        // Create sensor_data group
        let data_group = file.create_group("sensor_data")
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to create sensor_data group: {}", e)))?;

        // Create chunked, compressed datasets
        let chunk_size = 1024;

        let timestamps = Self::create_dataset::<f64>(&data_group, "timestamps", chunk_size)?;
        let accel_x = Self::create_dataset::<i16>(&data_group, "accel_x", chunk_size)?;
        let accel_y = Self::create_dataset::<i16>(&data_group, "accel_y", chunk_size)?;
        let accel_z = Self::create_dataset::<i16>(&data_group, "accel_z", chunk_size)?;
        let gyro_x = Self::create_dataset::<i16>(&data_group, "gyro_x", chunk_size)?;
        let gyro_y = Self::create_dataset::<i16>(&data_group, "gyro_y", chunk_size)?;
        let gyro_z = Self::create_dataset::<i16>(&data_group, "gyro_z", chunk_size)?;

        let datasets = DatasetHandles {
            timestamps,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        };

        Ok(Self {
            file,
            datasets,
            start_time: Instant::now(),
            sample_count: 0,
        })
    }

    /// Create a resizable, chunked, compressed dataset
    fn create_dataset<T: hdf5::H5Type>(group: &Group, name: &str, chunk_size: usize) -> Result<Dataset> {
        group.new_dataset::<T>()
            .shape((0..,))  // Resizable, starts at 0
            .chunk((chunk_size,))  // Chunk size for efficient I/O
            .deflate(4)  // DEFLATE compression level 4
            .create(name)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to create dataset {}: {}", name, e)))
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

        // Prepare data arrays
        let timestamps: Vec<f64> = samples.iter().map(|s| s.timestamp).collect();
        let accel_x: Vec<i16> = samples.iter().map(|s| s.data.accel_x).collect();
        let accel_y: Vec<i16> = samples.iter().map(|s| s.data.accel_y).collect();
        let accel_z: Vec<i16> = samples.iter().map(|s| s.data.accel_z).collect();
        let gyro_x: Vec<i16> = samples.iter().map(|s| s.data.gyro_x).collect();
        let gyro_y: Vec<i16> = samples.iter().map(|s| s.data.gyro_y).collect();
        let gyro_z: Vec<i16> = samples.iter().map(|s| s.data.gyro_z).collect();

        // Resize and append to each dataset
        self.append_to_dataset(&self.datasets.timestamps, new_size, &timestamps)?;
        self.append_to_dataset(&self.datasets.accel_x, new_size, &accel_x)?;
        self.append_to_dataset(&self.datasets.accel_y, new_size, &accel_y)?;
        self.append_to_dataset(&self.datasets.accel_z, new_size, &accel_z)?;
        self.append_to_dataset(&self.datasets.gyro_x, new_size, &gyro_x)?;
        self.append_to_dataset(&self.datasets.gyro_y, new_size, &gyro_y)?;
        self.append_to_dataset(&self.datasets.gyro_z, new_size, &gyro_z)?;

        self.sample_count = new_size;
        Ok(())
    }

    /// Append data to a dataset
    fn append_to_dataset<T: hdf5::H5Type>(&self, dataset: &Dataset, new_size: usize, data: &[T]) -> Result<()> {
        dataset.resize((new_size,))
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to resize dataset: {}", e)))?;

        let start = new_size - data.len();
        dataset.write_slice(data, start..)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to write to dataset: {}", e)))?;

        Ok(())
    }

    /// Flush data to disk
    pub fn flush(&mut self) -> Result<()> {
        self.file.flush()
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to flush HDF5 file: {}", e)))?;
        Ok(())
    }

    /// Get current sample count
    pub fn sample_count(&self) -> usize {
        self.sample_count
    }

    /// Get elapsed time since start
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
    /// Open an existing HDF5 file for reading
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open HDF5 file: {}", e)))?;

        // Read metadata
        let metadata = Self::read_metadata(&file)?;

        // Open datasets
        let data_group = file.group("sensor_data")
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open sensor_data group: {}", e)))?;

        let datasets = DatasetHandles {
            timestamps: data_group.dataset("timestamps")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open timestamps dataset: {}", e)))?,
            accel_x: data_group.dataset("accel_x")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open accel_x dataset: {}", e)))?,
            accel_y: data_group.dataset("accel_y")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open accel_y dataset: {}", e)))?,
            accel_z: data_group.dataset("accel_z")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open accel_z dataset: {}", e)))?,
            gyro_x: data_group.dataset("gyro_x")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open gyro_x dataset: {}", e)))?,
            gyro_y: data_group.dataset("gyro_y")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open gyro_y dataset: {}", e)))?,
            gyro_z: data_group.dataset("gyro_z")
                .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open gyro_z dataset: {}", e)))?,
        };

        Ok(Self {
            file,
            datasets,
            metadata,
        })
    }

    /// Read metadata from file
    fn read_metadata(file: &File) -> Result<Metadata> {
        let metadata_group = file.group("metadata")
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to open metadata group: {}", e)))?;

        let start_time = metadata_group.attr("start_time")
            .and_then(|attr| attr.read_scalar::<hdf5::types::VarLenUnicode>())
            .map(|s| s.to_string())
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read start_time: {}", e)))?;

        let sample_rate_hz = metadata_group.attr("sample_rate_hz")
            .and_then(|attr| attr.read_scalar::<f64>())
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read sample_rate_hz: {}", e)))?;

        let acquisition_mode = metadata_group.attr("acquisition_mode")
            .and_then(|attr| attr.read_scalar::<hdf5::types::VarLenUnicode>())
            .map(|s| s.to_string())
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read acquisition_mode: {}", e)))?;

        let version = metadata_group.attr("version")
            .and_then(|attr| attr.read_scalar::<hdf5::types::VarLenUnicode>())
            .map(|s| s.to_string())
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read version: {}", e)))?;

        Ok(Metadata {
            start_time,
            sample_rate_hz,
            acquisition_mode,
            version,
        })
    }

    /// Get metadata
    pub fn metadata(&self) -> &Metadata {
        &self.metadata
    }

    /// Get total number of samples in file
    pub fn get_total_samples(&self) -> Result<usize> {
        let size = self.datasets.timestamps.size();
        Ok(size)
    }

    /// Read a range of samples
    pub fn read_range(&self, start: usize, count: usize) -> Result<Vec<TimestampedSample>> {
        let total = self.get_total_samples()?;
        if start >= total {
            return Ok(Vec::new());
        }

        let actual_count = count.min(total - start);
        let end = start + actual_count;

        // Read each dataset slice
        let timestamps: Vec<f64> = self.datasets.timestamps.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read timestamps: {}", e)))?
            .to_vec();

        let accel_x: Vec<i16> = self.datasets.accel_x.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read accel_x: {}", e)))?
            .to_vec();

        let accel_y: Vec<i16> = self.datasets.accel_y.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read accel_y: {}", e)))?
            .to_vec();

        let accel_z: Vec<i16> = self.datasets.accel_z.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read accel_z: {}", e)))?
            .to_vec();

        let gyro_x: Vec<i16> = self.datasets.gyro_x.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read gyro_x: {}", e)))?
            .to_vec();

        let gyro_y: Vec<i16> = self.datasets.gyro_y.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read gyro_y: {}", e)))?
            .to_vec();

        let gyro_z: Vec<i16> = self.datasets.gyro_z.read_slice_1d(start..end)
            .map_err(|e| Mpu6050Error::CommunicationError(format!("Failed to read gyro_z: {}", e)))?
            .to_vec();

        // Combine into samples
        let samples: Vec<TimestampedSample> = timestamps.into_iter()
            .zip(accel_x.into_iter())
            .zip(accel_y.into_iter())
            .zip(accel_z.into_iter())
            .zip(gyro_x.into_iter())
            .zip(gyro_y.into_iter())
            .zip(gyro_z.into_iter())
            .map(|((((((ts, ax), ay), az), gx), gy), gz)| {
                TimestampedSample {
                    timestamp: ts,
                    data: SensorData {
                        accel_x: ax,
                        accel_y: ay,
                        accel_z: az,
                        gyro_x: gx,
                        gyro_y: gy,
                        gyro_z: gz,
                    },
                }
            })
            .collect();

        Ok(samples)
    }

    /// Read the latest N samples
    pub fn read_latest(&self, count: usize) -> Result<Vec<TimestampedSample>> {
        let total = self.get_total_samples()?;
        if total == 0 {
            return Ok(Vec::new());
        }

        let start = total.saturating_sub(count);
        self.read_range(start, count)
    }
}
