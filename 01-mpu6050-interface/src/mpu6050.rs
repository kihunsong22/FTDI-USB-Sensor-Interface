//! MPU6050 sensor driver using FTDI MPSSE I2C interface

use crate::error::{Mpu6050Error, Result};
use crate::ffi::*;
use std::ptr;
use std::time::{Duration, Instant};

// MPU6050 I2C addresses
const MPU6050_ADDRESS: u8 = 0x68; // Default I2C address

// MPU6050 Register addresses
const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_GYRO_XOUT_H: u8 = 0x43;

// Expected WHO_AM_I value
const WHO_AM_I_VALUE: u8 = 0x68;

/// Control flow for streaming operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamControl {
    /// Continue streaming
    Continue,
    /// Stop streaming
    Break,
}

/// Sensor data structure containing accelerometer and gyroscope readings
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    /// Accelerometer X-axis (raw value)
    pub accel_x: i16,
    /// Accelerometer Y-axis (raw value)
    pub accel_y: i16,
    /// Accelerometer Z-axis (raw value)
    pub accel_z: i16,
    /// Gyroscope X-axis (raw value)
    pub gyro_x: i16,
    /// Gyroscope Y-axis (raw value)
    pub gyro_y: i16,
    /// Gyroscope Z-axis (raw value)
    pub gyro_z: i16,
}

impl SensorData {
    /// Convert raw accelerometer values to g (assuming +/-2g range)
    pub fn accel_to_g(&self) -> (f32, f32, f32) {
        const ACCEL_SCALE: f32 = 16384.0; // LSB/g for +/-2g range
        (
            self.accel_x as f32 / ACCEL_SCALE,
            self.accel_y as f32 / ACCEL_SCALE,
            self.accel_z as f32 / ACCEL_SCALE,
        )
    }

    /// Convert raw gyroscope values to degrees/second (assuming +/-250°/s range)
    pub fn gyro_to_dps(&self) -> (f32, f32, f32) {
        const GYRO_SCALE: f32 = 131.0; // LSB/(°/s) for +/-250°/s range
        (
            self.gyro_x as f32 / GYRO_SCALE,
            self.gyro_y as f32 / GYRO_SCALE,
            self.gyro_z as f32 / GYRO_SCALE,
        )
    }

    /// Get accelerometer X-axis in g
    pub fn accel_x_g(&self) -> f32 {
        self.accel_x as f32 / 16384.0
    }

    /// Get accelerometer Y-axis in g
    pub fn accel_y_g(&self) -> f32 {
        self.accel_y as f32 / 16384.0
    }

    /// Get accelerometer Z-axis in g
    pub fn accel_z_g(&self) -> f32 {
        self.accel_z as f32 / 16384.0
    }

    /// Get gyroscope X-axis in degrees/second
    pub fn gyro_x_dps(&self) -> f32 {
        self.gyro_x as f32 / 131.0
    }

    /// Get gyroscope Y-axis in degrees/second
    pub fn gyro_y_dps(&self) -> f32 {
        self.gyro_y as f32 / 131.0
    }

    /// Get gyroscope Z-axis in degrees/second
    pub fn gyro_z_dps(&self) -> f32 {
        self.gyro_z as f32 / 131.0
    }
}

/// MPU6050 sensor interface
pub struct Mpu6050 {
    handle: FT_HANDLE,
    address: u8,
}

impl Mpu6050 {
    /// Create a new MPU6050 instance and initialize the sensor
    ///
    /// # Arguments
    /// * `channel_index` - Index of the I2C channel to use (usually 0)
    ///
    /// # Returns
    /// * `Ok(Mpu6050)` - Initialized sensor
    /// * `Err(Mpu6050Error)` - If initialization fails
    pub fn new(channel_index: u32) -> Result<Self> {
        // Check number of available channels
        let mut num_channels: DWORD = 0;
        let status = unsafe { I2C_GetNumChannels(&mut num_channels) };
        if status != FT_OK {
            return Err(status.into());
        }

        if num_channels == 0 {
            return Err(Mpu6050Error::NoChannelsFound);
        }

        if channel_index >= num_channels {
            return Err(Mpu6050Error::InvalidChannel(channel_index));
        }

        // Open the channel
        let mut handle: FT_HANDLE = ptr::null_mut();
        let status = unsafe { I2C_OpenChannel(channel_index, &mut handle) };
        if status != FT_OK {
            return Err(status.into());
        }

        // Configure the channel
        let mut config = ChannelConfig {
            ClockRate: I2C_CLOCK_FAST_MODE_PLUS, // 1 MHz
            LatencyTimer: 1,                      // 1ms latency (minimum stable value)
            Options: 0,
            Pin: 0,
            currentPinState: 0,
        };

        let status = unsafe { I2C_InitChannel(handle, &mut config) };
        if status != FT_OK {
            unsafe { I2C_CloseChannel(handle) };
            return Err(status.into());
        }

        let mut sensor = Mpu6050 {
            handle,
            address: MPU6050_ADDRESS,
        };

        // Initialize the sensor
        sensor.init()?;

        Ok(sensor)
    }

    /// Initialize the MPU6050 sensor
    fn init(&mut self) -> Result<()> {
        // Wake up the sensor (clear sleep bit)
        self.write_register(REG_PWR_MGMT_1, 0x00)?;

        // Small delay for sensor to wake up
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Verify device ID
        let who_am_i = self.read_register(REG_WHO_AM_I)?;
        if who_am_i != WHO_AM_I_VALUE {
            return Err(Mpu6050Error::InvalidDeviceId(who_am_i));
        }

        // Configure accelerometer (default +/-2g)
        self.write_register(REG_ACCEL_CONFIG, 0x00)?;

        // Configure gyroscope (default +/-250°/s)
        self.write_register(REG_GYRO_CONFIG, 0x00)?;

        Ok(())
    }

    /// Write a single byte to a register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<()> {
        let mut buffer = [reg, value];
        let mut transferred: DWORD = 0;

        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;

        let status = unsafe {
            I2C_DeviceWrite(
                self.handle,
                self.address,
                2,
                buffer.as_mut_ptr(),
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        // Note: With FAST_TRANSFER_BYTES, transferred count is in bits, not bytes
        // Only check status per FTDI sample code pattern

        Ok(())
    }

    /// Read a single byte from a register
    fn read_register(&mut self, reg: u8) -> Result<u8> {
        let mut reg_buf = [reg];
        let mut transferred: DWORD = 0;

        // Write register address
        let options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_BREAK_ON_NACK;

        let status = unsafe {
            I2C_DeviceWrite(
                self.handle,
                self.address,
                1,
                reg_buf.as_mut_ptr(),
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        // Read the data
        let mut data = [0u8];
        transferred = 0;

        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE;

        let status = unsafe {
            I2C_DeviceRead(
                self.handle,
                self.address,
                1,
                data.as_mut_ptr(),
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        if transferred != 1 {
            return Err(Mpu6050Error::TransferError {
                expected: 1,
                actual: transferred,
            });
        }

        Ok(data[0])
    }

    /// Read multiple bytes from consecutive registers
    fn read_registers(&mut self, reg: u8, count: usize) -> Result<Vec<u8>> {
        let mut reg_buf = [reg];
        let mut transferred: DWORD = 0;

        // Write register address (without STOP - keep bus for read)
        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_BREAK_ON_NACK
            | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;

        let status = unsafe {
            I2C_DeviceWrite(
                self.handle,
                self.address,
                1,
                reg_buf.as_mut_ptr(),
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        // Read the data immediately (repeated START)
        let mut data = vec![0u8; count];
        transferred = 0;

        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE
            | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;

        let status = unsafe {
            I2C_DeviceRead(
                self.handle,
                self.address,
                count as DWORD,
                data.as_mut_ptr(),
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        // Note: When using FAST_TRANSFER_BYTES, the transferred count is in bits, not bytes
        // (e.g., 6 bytes = 48 bits). Based on FTDI sample code, we should only check status.
        // If status is FT_OK, the data is valid regardless of the transferred count.

        Ok(data)
    }

    /// Read accelerometer data
    ///
    /// # Returns
    /// * `Ok((x, y, z))` - Raw 16-bit accelerometer values
    pub fn read_accel(&mut self) -> Result<(i16, i16, i16)> {
        let data = self.read_registers(REG_ACCEL_XOUT_H, 6)?;

        let x = i16::from_be_bytes([data[0], data[1]]);
        let y = i16::from_be_bytes([data[2], data[3]]);
        let z = i16::from_be_bytes([data[4], data[5]]);

        Ok((x, y, z))
    }

    /// Read gyroscope data
    ///
    /// # Returns
    /// * `Ok((x, y, z))` - Raw 16-bit gyroscope values
    pub fn read_gyro(&mut self) -> Result<(i16, i16, i16)> {
        let data = self.read_registers(REG_GYRO_XOUT_H, 6)?;

        let x = i16::from_be_bytes([data[0], data[1]]);
        let y = i16::from_be_bytes([data[2], data[3]]);
        let z = i16::from_be_bytes([data[4], data[5]]);

        Ok((x, y, z))
    }

    /// Read both accelerometer and gyroscope data
    ///
    /// This reads all 14 bytes (accel + temp + gyro) in a single I2C transaction
    /// for maximum performance. Temperature data is read but not returned.
    ///
    /// # Returns
    /// * `Ok(SensorData)` - Structure containing all sensor readings
    pub fn read_all(&mut self) -> Result<SensorData> {
        // Read all 14 bytes starting from ACCEL_XOUT_H (0x3B):
        // Bytes 0-5:   ACCEL_XOUT (X, Y, Z) - 6 bytes
        // Bytes 6-7:   TEMP_OUT - 2 bytes (skipped)
        // Bytes 8-13:  GYRO_XOUT (X, Y, Z) - 6 bytes
        let data = self.read_registers(REG_ACCEL_XOUT_H, 14)?;

        let accel_x = i16::from_be_bytes([data[0], data[1]]);
        let accel_y = i16::from_be_bytes([data[2], data[3]]);
        let accel_z = i16::from_be_bytes([data[4], data[5]]);
        // data[6..8] is temperature (ignored)
        let gyro_x = i16::from_be_bytes([data[8], data[9]]);
        let gyro_y = i16::from_be_bytes([data[10], data[11]]);
        let gyro_z = i16::from_be_bytes([data[12], data[13]]);

        Ok(SensorData {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        })
    }

    /// Stream sensor data at a specified rate with a callback function
    ///
    /// This method continuously reads sensor data and calls the provided callback
    /// function with each new reading. The callback can return `StreamControl::Break`
    /// to stop the stream.
    ///
    /// # Arguments
    /// * `rate_hz` - Target sample rate in Hz (1-1000). Actual rate may be lower
    ///               depending on I2C speed and system performance.
    /// * `callback` - Function called for each sample. Return `StreamControl::Continue`
    ///                to keep streaming or `StreamControl::Break` to stop.
    ///
    /// # Returns
    /// * `Ok(u64)` - Number of samples collected before stopping
    /// * `Err(Mpu6050Error)` - If a read error occurs
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::{Mpu6050, StreamControl};
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// let mut samples = Vec::new();
    ///
    /// // Collect 1000 samples at 100 Hz
    /// sensor.stream(100, |data| {
    ///     samples.push(data);
    ///     if samples.len() >= 1000 {
    ///         StreamControl::Break
    ///     } else {
    ///         StreamControl::Continue
    ///     }
    /// })?;
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn stream<F>(&mut self, rate_hz: u32, mut callback: F) -> Result<u64>
    where
        F: FnMut(SensorData) -> StreamControl,
    {
        if rate_hz == 0 || rate_hz > 1000 {
            return Err(Mpu6050Error::InvalidParameter(format!(
                "Sample rate must be between 1-1000 Hz, got {}",
                rate_hz
            )));
        }

        let interval = Duration::from_micros(1_000_000 / rate_hz as u64);
        let mut sample_count = 0u64;
        let mut next_sample_time = Instant::now();

        loop {
            // Read sensor data
            let data = self.read_all()?;
            sample_count += 1;

            // Call user callback
            if callback(data) == StreamControl::Break {
                break;
            }

            // Wait until next sample time
            next_sample_time += interval;
            let now = Instant::now();
            if next_sample_time > now {
                std::thread::sleep(next_sample_time - now);
            }
            // If we're running behind, don't sleep and continue immediately
        }

        Ok(sample_count)
    }

    /// Stream sensor data for a specified duration
    ///
    /// # Arguments
    /// * `rate_hz` - Target sample rate in Hz (1-1000)
    /// * `duration` - How long to stream data
    /// * `callback` - Function called for each sample
    ///
    /// # Returns
    /// * `Ok(u64)` - Number of samples collected
    /// * `Err(Mpu6050Error)` - If a read error occurs
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    /// use std::time::Duration;
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// let mut max_accel = 0.0f32;
    ///
    /// // Monitor peak acceleration for 5 seconds at 500 Hz
    /// sensor.stream_for(500, Duration::from_secs(5), |data| {
    ///     let (ax, ay, az) = data.accel_to_g();
    ///     let magnitude = (ax*ax + ay*ay + az*az).sqrt();
    ///     max_accel = max_accel.max(magnitude);
    /// })?;
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn stream_for<F>(&mut self, rate_hz: u32, duration: Duration, mut callback: F) -> Result<u64>
    where
        F: FnMut(SensorData),
    {
        let end_time = Instant::now() + duration;

        self.stream(rate_hz, |data| {
            callback(data);
            if Instant::now() >= end_time {
                StreamControl::Break
            } else {
                StreamControl::Continue
            }
        })
    }

    /// Collect a specified number of samples at a given rate
    ///
    /// # Arguments
    /// * `rate_hz` - Target sample rate in Hz (1-1000)
    /// * `num_samples` - Number of samples to collect
    ///
    /// # Returns
    /// * `Ok(Vec<SensorData>)` - Vector of collected samples
    /// * `Err(Mpu6050Error)` - If a read error occurs
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    ///
    /// // Collect 1000 samples at 500 Hz for vibration analysis
    /// let samples = sensor.collect_samples(500, 1000)?;
    ///
    /// // Perform FFT or other analysis on samples...
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn collect_samples(&mut self, rate_hz: u32, num_samples: usize) -> Result<Vec<SensorData>> {
        let mut samples = Vec::with_capacity(num_samples);

        self.stream(rate_hz, |data| {
            samples.push(data);
            if samples.len() >= num_samples {
                StreamControl::Break
            } else {
                StreamControl::Continue
            }
        })?;

        Ok(samples)
    }
}

impl Drop for Mpu6050 {
    fn drop(&mut self) {
        unsafe {
            I2C_CloseChannel(self.handle);
        }
    }
}
