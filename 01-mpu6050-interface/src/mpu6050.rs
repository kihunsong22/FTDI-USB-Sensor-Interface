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

// FIFO and sample rate registers
const REG_SMPLRT_DIV: u8 = 0x19;      // Sample rate divider
const REG_CONFIG: u8 = 0x1A;          // DLPF configuration
const REG_FIFO_EN: u8 = 0x23;         // FIFO enable register
const REG_INT_STATUS: u8 = 0x3A;      // Interrupt status
const REG_USER_CTRL: u8 = 0x6A;       // User control (FIFO enable/reset)
const REG_FIFO_COUNTH: u8 = 0x72;     // FIFO count high byte
const REG_FIFO_COUNTL: u8 = 0x73;     // FIFO count low byte
const REG_FIFO_R_W: u8 = 0x74;        // FIFO read/write

// FIFO enable bits (REG_FIFO_EN)
const FIFO_EN_ACCEL: u8 = 0x08;       // Enable accelerometer to FIFO
const FIFO_EN_GYRO_X: u8 = 0x40;      // Enable gyro X to FIFO
const FIFO_EN_GYRO_Y: u8 = 0x20;      // Enable gyro Y to FIFO
const FIFO_EN_GYRO_Z: u8 = 0x10;      // Enable gyro Z to FIFO
const FIFO_EN_ALL_SENSORS: u8 = FIFO_EN_ACCEL | FIFO_EN_GYRO_X
                               | FIFO_EN_GYRO_Y | FIFO_EN_GYRO_Z; // 0x78

// User control bits (REG_USER_CTRL)
const USER_CTRL_FIFO_EN: u8 = 0x40;   // Enable FIFO
const USER_CTRL_FIFO_RESET: u8 = 0x04; // Reset FIFO

// Interrupt status bits (REG_INT_STATUS)
const INT_STATUS_FIFO_OVERFLOW: u8 = 0x10; // FIFO overflow interrupt

// FIFO constants
const FIFO_SAMPLE_SIZE: usize = 12;   // Bytes per sample (accel + gyro, no temp)
const FIFO_MAX_SIZE: usize = 1024;    // Maximum FIFO size in bytes
const FIFO_MAX_SAMPLES: usize = FIFO_MAX_SIZE / FIFO_SAMPLE_SIZE; // 85

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
    fifo_enabled: bool,  // Track FIFO mode state
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
            fifo_enabled: false,  // Start with FIFO disabled
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

    /// Read FIFO byte count
    fn read_fifo_count_raw(&mut self) -> Result<u16> {
        let high = self.read_register(REG_FIFO_COUNTH)?;
        let low = self.read_register(REG_FIFO_COUNTL)?;
        Ok(u16::from_be_bytes([high, low]))
    }

    /// Read raw bytes from FIFO
    fn read_fifo_raw(&mut self, count: usize) -> Result<Vec<u8>> {
        if count == 0 {
            return Ok(Vec::new());
        }

        // FIFO_R_W auto-increments, so we can read multiple bytes
        self.read_registers(REG_FIFO_R_W, count)
    }

    /// Check for FIFO overflow condition
    fn check_fifo_overflow(&mut self) -> Result<bool> {
        let int_status = self.read_register(REG_INT_STATUS)?;
        Ok(int_status & INT_STATUS_FIFO_OVERFLOW != 0)
    }

    /// Parse FIFO data into SensorData structs
    fn parse_fifo_data(buffer: &[u8]) -> Result<Vec<SensorData>> {
        if buffer.len() % FIFO_SAMPLE_SIZE != 0 {
            return Err(Mpu6050Error::InvalidFifoConfig(
                format!("FIFO data length {} is not a multiple of sample size {}",
                        buffer.len(), FIFO_SAMPLE_SIZE)
            ));
        }

        let num_samples = buffer.len() / FIFO_SAMPLE_SIZE;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * FIFO_SAMPLE_SIZE;
            let chunk = &buffer[offset..offset + FIFO_SAMPLE_SIZE];

            // FIFO order: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ...
            let accel_x = i16::from_be_bytes([chunk[0], chunk[1]]);
            let accel_y = i16::from_be_bytes([chunk[2], chunk[3]]);
            let accel_z = i16::from_be_bytes([chunk[4], chunk[5]]);
            let gyro_x = i16::from_be_bytes([chunk[6], chunk[7]]);
            let gyro_y = i16::from_be_bytes([chunk[8], chunk[9]]);
            let gyro_z = i16::from_be_bytes([chunk[10], chunk[11]]);

            samples.push(SensorData {
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
            });
        }

        Ok(samples)
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

    /// Enable FIFO mode and configure sample rate
    ///
    /// This configures the MPU6050 to store sensor data in its internal FIFO buffer
    /// at the specified sample rate. The FIFO allows achieving higher effective sample
    /// rates (up to 1kHz) than direct polling, at the cost of buffering latency.
    ///
    /// # Arguments
    /// * `sample_rate_hz` - Target sample rate (4-1000 Hz)
    ///
    /// # Returns
    /// * `Ok(())` - FIFO enabled successfully
    /// * `Err(Mpu6050Error)` - Configuration failed
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// sensor.enable_fifo(1000)?;  // 1kHz sampling
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn enable_fifo(&mut self, sample_rate_hz: u16) -> Result<()> {
        if sample_rate_hz < 4 || sample_rate_hz > 1000 {
            return Err(Mpu6050Error::InvalidParameter(format!(
                "Sample rate must be 4-1000 Hz, got {}",
                sample_rate_hz
            )));
        }

        // Disable FIFO first if it's enabled
        if self.fifo_enabled {
            self.disable_fifo()?;
        }

        // Configure DLPF for 1kHz gyro output rate (DLPF_CFG=1, 188Hz BW)
        self.write_register(REG_CONFIG, 0x01)?;

        // Calculate sample rate divider
        // Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
        // For DLPF_CFG=1: Gyro Output Rate = 1kHz
        let gyro_rate = 1000u16;
        let divider = (gyro_rate / sample_rate_hz).saturating_sub(1);
        self.write_register(REG_SMPLRT_DIV, divider as u8)?;

        // Reset FIFO
        self.write_register(REG_USER_CTRL, USER_CTRL_FIFO_RESET)?;
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Enable accelerometer and gyroscope data to FIFO
        self.write_register(REG_FIFO_EN, FIFO_EN_ALL_SENSORS)?;

        // Enable FIFO
        self.write_register(REG_USER_CTRL, USER_CTRL_FIFO_EN)?;

        self.fifo_enabled = true;
        Ok(())
    }

    /// Disable FIFO mode and return to direct read mode
    ///
    /// # Returns
    /// * `Ok(())` - FIFO disabled successfully
    pub fn disable_fifo(&mut self) -> Result<()> {
        // Disable FIFO enable bit
        self.write_register(REG_USER_CTRL, 0x00)?;

        // Disable all sensors to FIFO
        self.write_register(REG_FIFO_EN, 0x00)?;

        // Reset sample rate divider to default
        self.write_register(REG_SMPLRT_DIV, 0x00)?;

        // Reset CONFIG to default
        self.write_register(REG_CONFIG, 0x00)?;

        self.fifo_enabled = false;
        Ok(())
    }

    /// Get the current number of bytes in the FIFO buffer
    ///
    /// # Returns
    /// * `Ok(u16)` - Number of bytes in FIFO
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// sensor.enable_fifo(1000)?;
    ///
    /// let count = sensor.get_fifo_count()?;
    /// println!("FIFO contains {} bytes ({} samples)",
    ///          count, count / 12);
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn get_fifo_count(&mut self) -> Result<u16> {
        self.read_fifo_count_raw()
    }

    /// Reset (clear) the FIFO buffer
    ///
    /// This clears all data from the FIFO without disabling it.
    ///
    /// # Returns
    /// * `Ok(())` - FIFO reset successfully
    pub fn reset_fifo(&mut self) -> Result<()> {
        self.write_register(REG_USER_CTRL, USER_CTRL_FIFO_RESET | USER_CTRL_FIFO_EN)?;
        std::thread::sleep(std::time::Duration::from_millis(1));
        self.write_register(REG_USER_CTRL, USER_CTRL_FIFO_EN)?;
        Ok(())
    }

    /// Read all available samples from the FIFO buffer
    ///
    /// This reads all currently available sensor data from the FIFO and returns
    /// it as a vector of SensorData structs. If FIFO overflow is detected, an
    /// error is returned and the FIFO is automatically reset.
    ///
    /// IMPORTANT: Call this frequently enough to prevent overflow. At 1kHz,
    /// the 1024-byte FIFO fills in ~85ms. Recommended read interval: 50ms.
    ///
    /// # Returns
    /// * `Ok(Vec<SensorData>)` - Vector of samples (may be empty)
    /// * `Err(Mpu6050Error::FifoOverflow)` - FIFO overflowed, data lost
    /// * `Err(Mpu6050Error::FifoNotEnabled)` - FIFO not enabled
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    /// use std::{thread, time::Duration};
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// sensor.enable_fifo(1000)?;
    ///
    /// // Read FIFO every 50ms
    /// loop {
    ///     thread::sleep(Duration::from_millis(50));
    ///     let samples = sensor.read_fifo_batch()?;
    ///     println!("Read {} samples", samples.len());
    ///
    ///     for sample in samples {
    ///         // Process each sample...
    ///     }
    /// }
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn read_fifo_batch(&mut self) -> Result<Vec<SensorData>> {
        if !self.fifo_enabled {
            return Err(Mpu6050Error::FifoNotEnabled);
        }

        // Check for overflow first
        if self.check_fifo_overflow()? {
            let count = self.read_fifo_count_raw()?;
            let samples_lost = count / FIFO_SAMPLE_SIZE as u16;

            // Reset FIFO to recover
            self.reset_fifo()?;

            return Err(Mpu6050Error::FifoOverflow {
                samples_lost: format!("~{}", samples_lost),
            });
        }

        // Read FIFO count
        let fifo_count = self.read_fifo_count_raw()?;

        if fifo_count == 0 {
            return Ok(Vec::new());
        }

        // Calculate number of complete samples
        let num_samples = (fifo_count as usize) / FIFO_SAMPLE_SIZE;
        let bytes_to_read = num_samples * FIFO_SAMPLE_SIZE;

        if bytes_to_read == 0 {
            return Ok(Vec::new());
        }

        // Read FIFO data
        let fifo_data = self.read_fifo_raw(bytes_to_read)?;

        // Parse into SensorData structs
        Self::parse_fifo_data(&fifo_data)
    }

    /// Stream FIFO data with periodic batch reads
    ///
    /// This method continuously reads batches of samples from the FIFO at the
    /// specified interval and calls the callback function for each batch.
    /// Unlike `stream()`, which provides individual samples at a fixed rate,
    /// this provides batches of buffered samples, allowing for higher throughput
    /// (up to 1kHz) at the cost of latency.
    ///
    /// # Arguments
    /// * `batch_interval_ms` - How often to read FIFO in milliseconds (10-1000)
    /// * `callback` - Function called for each batch. Receives a slice of samples.
    ///                Return `StreamControl::Continue` to keep streaming or
    ///                `StreamControl::Break` to stop.
    ///
    /// # Returns
    /// * `Ok(u64)` - Total number of samples collected before stopping
    /// * `Err(Mpu6050Error)` - If a read error or FIFO overflow occurs
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::{Mpu6050, StreamControl};
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    /// sensor.enable_fifo(1000)?;  // 1kHz internal sampling
    ///
    /// let mut all_samples = Vec::new();
    ///
    /// // Read FIFO every 50ms (expect ~50 samples per batch)
    /// sensor.stream_fifo(50, |batch| {
    ///     all_samples.extend_from_slice(batch);
    ///
    ///     if all_samples.len() >= 10000 {
    ///         StreamControl::Break
    ///     } else {
    ///         StreamControl::Continue
    ///     }
    /// })?;
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn stream_fifo<F>(&mut self, batch_interval_ms: u64, mut callback: F) -> Result<u64>
    where
        F: FnMut(&[SensorData]) -> StreamControl,
    {
        if !self.fifo_enabled {
            return Err(Mpu6050Error::FifoNotEnabled);
        }

        if batch_interval_ms < 10 || batch_interval_ms > 1000 {
            return Err(Mpu6050Error::InvalidParameter(format!(
                "Batch interval must be 10-1000 ms, got {}",
                batch_interval_ms
            )));
        }

        let interval = Duration::from_millis(batch_interval_ms);
        let mut total_samples = 0u64;
        let mut next_read_time = Instant::now();

        loop {
            // Read FIFO batch
            let batch = self.read_fifo_batch()?;

            if !batch.is_empty() {
                total_samples += batch.len() as u64;

                // Call user callback with batch
                if callback(&batch) == StreamControl::Break {
                    break;
                }
            }

            // Wait until next read time
            next_read_time += interval;
            let now = Instant::now();
            if next_read_time > now {
                std::thread::sleep(next_read_time - now);
            }
            // If we're running behind, don't sleep and continue immediately
        }

        Ok(total_samples)
    }

    /// Collect samples using FIFO mode
    ///
    /// This is a convenience method that enables FIFO, collects the specified
    /// number of samples, then returns them as a vector. FIFO remains enabled
    /// after this call.
    ///
    /// # Arguments
    /// * `sample_rate_hz` - Internal sample rate (4-1000 Hz)
    /// * `num_samples` - Number of samples to collect
    ///
    /// # Returns
    /// * `Ok(Vec<SensorData>)` - Vector of collected samples
    ///
    /// # Example
    /// ```no_run
    /// use ft232_sensor_interface::Mpu6050;
    ///
    /// let mut sensor = Mpu6050::new(0)?;
    ///
    /// // Collect 2048 samples at 1kHz for FFT analysis
    /// let samples = sensor.collect_samples_fifo(1000, 2048)?;
    ///
    /// // Perform FFT or other frequency analysis...
    /// # Ok::<(), ft232_sensor_interface::Mpu6050Error>(())
    /// ```
    pub fn collect_samples_fifo(&mut self, sample_rate_hz: u16, num_samples: usize) -> Result<Vec<SensorData>> {
        // Enable FIFO if not already enabled
        if !self.fifo_enabled {
            self.enable_fifo(sample_rate_hz)?;
        }

        // Reset FIFO to start fresh
        self.reset_fifo()?;

        let mut samples = Vec::with_capacity(num_samples);

        // Calculate appropriate batch interval
        // At 1kHz, 50 samples = 50ms, read interval should be slightly longer
        let batch_interval_ms = 50u64;

        self.stream_fifo(batch_interval_ms, |batch| {
            samples.extend_from_slice(batch);

            if samples.len() >= num_samples {
                StreamControl::Break
            } else {
                StreamControl::Continue
            }
        })?;

        // Truncate to exact count if we got more
        samples.truncate(num_samples);

        Ok(samples)
    }
}

impl Drop for Mpu6050 {
    fn drop(&mut self) {
        // Disable FIFO if it was enabled
        let _ = self.disable_fifo();

        unsafe {
            I2C_CloseChannel(self.handle);
        }
    }
}
