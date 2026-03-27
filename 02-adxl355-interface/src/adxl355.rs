//! ADXL355 sensor driver using FTDI MPSSE I2C interface

use crate::error::{Adxl355Error, Result};
use crate::ffi::*;
use std::ptr;
use std::time::{Duration, Instant};

// ADXL355 I2C addresses
const ADXL355_ADDRESS_LOW: u8 = 0x1D;  // ASEL pin low (default)
const ADXL355_ADDRESS_HIGH: u8 = 0x53; // ASEL pin high

// Device identification registers
const REG_DEVID_AD: u8 = 0x00;    // Analog Devices ID (expect 0xAD)
#[allow(dead_code)]
const REG_DEVID_MST: u8 = 0x01;   // MEMS device ID (expect 0x1D)
const REG_PARTID: u8 = 0x02;      // Part ID (expect 0xED for ADXL355)
#[allow(dead_code)]
const REG_REVID: u8 = 0x03;       // Silicon revision

// Status and FIFO
#[allow(dead_code)]
const REG_STATUS: u8 = 0x04;      // Status register
const REG_FIFO_ENTRIES: u8 = 0x05; // Number of valid FIFO samples (0-96)

// Temperature data (12-bit: TEMP2[3:0] = bits[11:8], TEMP1[7:0] = bits[7:0])
const REG_TEMP2: u8 = 0x06;       // Temperature bits [11:8] in lower nibble
#[allow(dead_code)]
const REG_TEMP1: u8 = 0x07;       // Temperature bits [7:0]

// Accelerometer data (20-bit per axis, 3 registers each)
const REG_XDATA3: u8 = 0x08;      // X-axis [19:12]
#[allow(dead_code)]
const REG_XDATA2: u8 = 0x09;      // X-axis [11:4]
#[allow(dead_code)]
const REG_XDATA1: u8 = 0x0A;      // X-axis [3:0] in bits [7:4]
#[allow(dead_code)]
const REG_YDATA3: u8 = 0x0B;      // Y-axis [19:12]
#[allow(dead_code)]
const REG_YDATA2: u8 = 0x0C;      // Y-axis [11:4]
#[allow(dead_code)]
const REG_YDATA1: u8 = 0x0D;      // Y-axis [3:0] in bits [7:4]
#[allow(dead_code)]
const REG_ZDATA3: u8 = 0x0E;      // Z-axis [19:12]
#[allow(dead_code)]
const REG_ZDATA2: u8 = 0x0F;      // Z-axis [11:4]
#[allow(dead_code)]
const REG_ZDATA1: u8 = 0x10;      // Z-axis [3:0] in bits [7:4]

// FIFO data register
const REG_FIFO_DATA: u8 = 0x11;   // FIFO read port

// Configuration registers
const REG_FILTER: u8 = 0x28;      // ODR and HPF settings
const REG_RANGE: u8 = 0x2C;       // Measurement range
const REG_POWER_CTL: u8 = 0x2D;   // Power control
const REG_RESET: u8 = 0x2F;       // Software reset (write 0x52)

// Expected device ID values
const DEVID_AD_VALUE: u8 = 0xAD;  // Analog Devices
const PARTID_VALUE: u8 = 0xED;    // ADXL355

// Status register bits
#[allow(dead_code)]
const STATUS_DATA_RDY: u8 = 0x01;     // Data ready
#[allow(dead_code)]
const STATUS_FIFO_FULL: u8 = 0x02;    // FIFO full
#[allow(dead_code)]
const STATUS_FIFO_OVR: u8 = 0x04;     // FIFO overrun

// Power control bits
const POWER_CTL_STANDBY: u8 = 0x01;   // Standby mode

// Reset command
const RESET_CODE: u8 = 0x52;

// FIFO constants
#[allow(dead_code)]
const FIFO_SAMPLE_SIZE: usize = 9;    // 3 bytes per axis × 3 axes (no temperature)
#[allow(dead_code)]
const FIFO_MAX_SAMPLES: usize = 96;   // Maximum FIFO depth

/// Measurement range
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Range {
    /// +/-2g (default), 256000 LSB/g, 3.9 ug/LSB
    G2 = 0x01,
    /// +/-4g, 128000 LSB/g, 7.8 ug/LSB
    G4 = 0x02,
    /// +/-8g, 64000 LSB/g, 15.6 ug/LSB
    G8 = 0x03,
}

impl Range {
    /// Scale factor in LSB/g for this range
    pub fn scale_factor(&self) -> f32 {
        match self {
            Range::G2 => 256_000.0,
            Range::G4 => 128_000.0,
            Range::G8 => 64_000.0,
        }
    }
}

/// Output data rate
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputDataRate {
    /// 4000 Hz
    Odr4000 = 0x00,
    /// 2000 Hz
    Odr2000 = 0x01,
    /// 1000 Hz
    Odr1000 = 0x02,
    /// 500 Hz
    Odr500 = 0x03,
    /// 250 Hz
    Odr250 = 0x04,
    /// 125 Hz
    Odr125 = 0x05,
    /// 62.5 Hz
    Odr62_5 = 0x06,
    /// 31.25 Hz
    Odr31_25 = 0x07,
    /// 15.625 Hz
    Odr15_625 = 0x08,
    /// 7.813 Hz
    Odr7_813 = 0x09,
    /// 3.906 Hz
    Odr3_906 = 0x0A,
}

impl OutputDataRate {
    /// Get the rate in Hz
    pub fn as_hz(&self) -> f64 {
        match self {
            OutputDataRate::Odr4000 => 4000.0,
            OutputDataRate::Odr2000 => 2000.0,
            OutputDataRate::Odr1000 => 1000.0,
            OutputDataRate::Odr500 => 500.0,
            OutputDataRate::Odr250 => 250.0,
            OutputDataRate::Odr125 => 125.0,
            OutputDataRate::Odr62_5 => 62.5,
            OutputDataRate::Odr31_25 => 31.25,
            OutputDataRate::Odr15_625 => 15.625,
            OutputDataRate::Odr7_813 => 7.813,
            OutputDataRate::Odr3_906 => 3.906,
        }
    }
}

/// Control flow for streaming operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamControl {
    /// Continue streaming
    Continue,
    /// Stop streaming
    Break,
}

/// Sensor data structure containing accelerometer and temperature readings
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    /// Accelerometer X-axis (raw 20-bit value, sign-extended to i32)
    pub accel_x: i32,
    /// Accelerometer Y-axis (raw 20-bit value, sign-extended to i32)
    pub accel_y: i32,
    /// Accelerometer Z-axis (raw 20-bit value, sign-extended to i32)
    pub accel_z: i32,
    /// Temperature (raw 12-bit value)
    pub temperature: u16,
}

impl SensorData {
    /// Convert raw accelerometer values to g using the given range
    pub fn accel_to_g(&self, range: Range) -> (f32, f32, f32) {
        let scale = range.scale_factor();
        (
            self.accel_x as f32 / scale,
            self.accel_y as f32 / scale,
            self.accel_z as f32 / scale,
        )
    }

    /// Convert raw temperature to Celsius
    ///
    /// Note: The temperature sensor is uncalibrated (~10% tolerance on slope).
    /// Useful for drift compensation, not absolute measurement.
    pub fn temperature_c(&self) -> f32 {
        ((self.temperature as f32 - 1885.0) / -9.05) + 25.0
    }
}

/// Parse 20-bit two's complement value from 3 bytes
///
/// The ADXL355 packs 20-bit data as: DATA3[7:0] | DATA2[7:0] | DATA1[7:4]
/// Lower 4 bits of DATA1 contain flags (ignored here).
fn parse_20bit(high: u8, mid: u8, low: u8) -> i32 {
    let raw = ((high as u32) << 12) | ((mid as u32) << 4) | ((low as u32) >> 4);
    // Sign-extend from 20 bits to 32 bits
    if raw & 0x80000 != 0 {
        (raw | 0xFFF00000) as i32
    } else {
        raw as i32
    }
}

/// ADXL355 sensor interface
pub struct Adxl355 {
    handle: FT_HANDLE,
    address: u8,
    range: Range,
    fifo_enabled: bool,
}

impl Adxl355 {
    /// Create a new ADXL355 instance, auto-detecting the I2C address
    ///
    /// Tries 0x1D first, then 0x53. Use `with_address()` to skip detection.
    pub fn new(channel_index: u32) -> Result<Self> {
        // Open channel first, then probe both addresses
        let handle = Self::open_channel(channel_index)?;

        for &address in &[ADXL355_ADDRESS_LOW, ADXL355_ADDRESS_HIGH] {
            let mut sensor = Adxl355 {
                handle,
                address,
                range: Range::G2,
                fifo_enabled: false,
            };

            match sensor.init() {
                Ok(()) => {
                    return Ok(sensor);
                }
                Err(_) => {
                    // Reset state for next attempt
                    sensor.fifo_enabled = false;
                    // Prevent Drop from closing the handle — we'll reuse it
                    std::mem::forget(sensor);
                    continue;
                }
            }
        }

        // Neither address worked — close the channel and report
        unsafe { I2C_CloseChannel(handle) };
        Err(Adxl355Error::CommunicationError(
            "ADXL355 not found at 0x1D or 0x53. Check wiring and VDDIO.".to_string()
        ))
    }

    /// Create a new ADXL355 instance with a specific I2C address
    pub fn with_address(channel_index: u32, address: u8) -> Result<Self> {
        if address != ADXL355_ADDRESS_LOW && address != ADXL355_ADDRESS_HIGH {
            return Err(Adxl355Error::InvalidParameter(format!(
                "Invalid I2C address: 0x{:02X}. Must be 0x1D or 0x53",
                address
            )));
        }

        let handle = Self::open_channel(channel_index)?;

        let mut sensor = Adxl355 {
            handle,
            address,
            range: Range::G2,
            fifo_enabled: false,
        };

        sensor.init()?;

        Ok(sensor)
    }

    /// Open and configure an I2C channel
    fn open_channel(channel_index: u32) -> Result<FT_HANDLE> {
        let mut num_channels: DWORD = 0;
        let status = unsafe { I2C_GetNumChannels(&mut num_channels) };
        if status != FT_OK {
            return Err(status.into());
        }

        if num_channels == 0 {
            return Err(Adxl355Error::NoChannelsFound);
        }

        if channel_index >= num_channels {
            return Err(Adxl355Error::InvalidChannel(channel_index));
        }

        let mut handle: FT_HANDLE = ptr::null_mut();
        let status = unsafe { I2C_OpenChannel(channel_index, &mut handle) };
        if status != FT_OK {
            return Err(status.into());
        }

        let mut config = ChannelConfig {
            ClockRate: I2C_CLOCK_FAST_MODE_PLUS, // 1 MHz (3.4 MHz fails on breadboard)
            LatencyTimer: 1,
            Options: 0,
            Pin: 0,
            currentPinState: 0,
        };

        let status = unsafe { I2C_InitChannel(handle, &mut config) };
        if status != FT_OK {
            unsafe { I2C_CloseChannel(handle) };
            return Err(status.into());
        }

        Ok(handle)
    }

    /// Initialize the ADXL355 sensor
    fn init(&mut self) -> Result<()> {
        // Software reset
        self.write_register(REG_RESET, RESET_CODE)?;
        std::thread::sleep(Duration::from_millis(100));

        // Verify device identity
        let devid_ad = self.read_register(REG_DEVID_AD)?;
        if devid_ad != DEVID_AD_VALUE {
            return Err(Adxl355Error::InvalidDeviceId(devid_ad));
        }

        let partid = self.read_register(REG_PARTID)?;
        if partid != PARTID_VALUE {
            return Err(Adxl355Error::InvalidPartId(partid));
        }

        // Configure default range (+/-2g)
        self.write_register(REG_RANGE, Range::G2 as u8)?;

        // Configure default ODR (1000 Hz)
        self.write_register(REG_FILTER, OutputDataRate::Odr1000 as u8)?;

        // Enter measurement mode (clear standby bit)
        self.write_register(REG_POWER_CTL, 0x00)?;

        // Wait for first data to be ready
        std::thread::sleep(Duration::from_millis(10));

        Ok(())
    }

    // ========================================================================
    // I2C register operations (isolated for future SPI migration)
    // ========================================================================

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
            return Err(Adxl355Error::TransferError {
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

        Ok(data)
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    /// Set the measurement range
    ///
    /// Automatically enters standby mode for configuration, then resumes.
    pub fn set_range(&mut self, range: Range) -> Result<()> {
        // Enter standby
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        // Read current RANGE register to preserve INT_POL bit
        let current = self.read_register(REG_RANGE)?;
        let new_val = (current & 0xFC) | (range as u8);
        self.write_register(REG_RANGE, new_val)?;

        self.range = range;

        // Resume measurement
        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    /// Set the output data rate
    ///
    /// Automatically enters standby mode for configuration, then resumes.
    pub fn set_odr(&mut self, odr: OutputDataRate) -> Result<()> {
        // Enter standby
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        // Read current FILTER register to preserve HPF settings
        let current = self.read_register(REG_FILTER)?;
        let new_val = (current & 0xF0) | (odr as u8);
        self.write_register(REG_FILTER, new_val)?;

        // Resume measurement
        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    /// Get the current measurement range
    pub fn get_range(&self) -> Range {
        self.range
    }

    // ========================================================================
    // Data reading
    // ========================================================================

    /// Read temperature (12-bit raw value)
    pub fn read_temperature(&mut self) -> Result<u16> {
        let data = self.read_registers(REG_TEMP2, 2)?;
        let temp = ((data[0] as u16 & 0x0F) << 8) | (data[1] as u16);
        Ok(temp)
    }

    /// Read accelerometer data only (9 bytes burst read)
    pub fn read_accel(&mut self) -> Result<(i32, i32, i32)> {
        let data = self.read_registers(REG_XDATA3, 9)?;
        let x = parse_20bit(data[0], data[1], data[2]);
        let y = parse_20bit(data[3], data[4], data[5]);
        let z = parse_20bit(data[6], data[7], data[8]);
        Ok((x, y, z))
    }

    /// Read all sensor data (temperature + XYZ accelerometer) in one burst
    ///
    /// Reads 11 bytes from TEMP2 (0x06) through ZDATA1 (0x10).
    pub fn read_all(&mut self) -> Result<SensorData> {
        let data = self.read_registers(REG_TEMP2, 11)?;

        // Bytes 0-1: Temperature (12-bit, stored as high byte + low byte)
        let temperature = ((data[0] as u16) << 8) | (data[1] as u16);

        // Bytes 2-10: Accel X, Y, Z (20-bit each, 3 bytes per axis)
        let accel_x = parse_20bit(data[2], data[3], data[4]);
        let accel_y = parse_20bit(data[5], data[6], data[7]);
        let accel_z = parse_20bit(data[8], data[9], data[10]);

        Ok(SensorData {
            accel_x,
            accel_y,
            accel_z,
            temperature,
        })
    }

    // ========================================================================
    // Streaming API
    // ========================================================================

    /// Stream sensor data at a specified rate with a callback function
    pub fn stream<F>(&mut self, rate_hz: u32, mut callback: F) -> Result<u64>
    where
        F: FnMut(SensorData) -> StreamControl,
    {
        if rate_hz == 0 || rate_hz > 4000 {
            return Err(Adxl355Error::InvalidParameter(format!(
                "Sample rate must be between 1-4000 Hz, got {}",
                rate_hz
            )));
        }

        let interval = Duration::from_micros(1_000_000 / rate_hz as u64);
        let mut sample_count = 0u64;
        let mut next_sample_time = Instant::now();

        loop {
            let data = self.read_all()?;
            sample_count += 1;

            if callback(data) == StreamControl::Break {
                break;
            }

            next_sample_time += interval;
            let now = Instant::now();
            if next_sample_time > now {
                std::thread::sleep(next_sample_time - now);
            }
        }

        Ok(sample_count)
    }

    /// Stream sensor data for a specified duration
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

    // ========================================================================
    // FIFO operations
    // ========================================================================

    /// Enable FIFO mode (enters measurement mode)
    ///
    /// The ADXL355 FIFO is always active in measurement mode.
    /// This method configures the ODR and enters measurement mode.
    pub fn enable_fifo(&mut self, odr: OutputDataRate) -> Result<()> {
        // Enter standby first
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        // Configure ODR
        let current_filter = self.read_register(REG_FILTER)?;
        let new_filter = (current_filter & 0xF0) | (odr as u8);
        self.write_register(REG_FILTER, new_filter)?;

        // Enter measurement mode
        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(10));

        self.fifo_enabled = true;

        // Drain any startup data
        let _ = self.read_fifo_batch();

        Ok(())
    }

    /// Disable FIFO mode (enters standby)
    pub fn disable_fifo(&mut self) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        self.fifo_enabled = false;
        Ok(())
    }

    /// Get the number of valid samples in the FIFO
    pub fn get_fifo_entries(&mut self) -> Result<u8> {
        self.read_register(REG_FIFO_ENTRIES)
    }

    /// Read all available samples from the FIFO
    ///
    /// Reads FIFO_ENTRIES to determine count, then burst reads exact bytes needed.
    /// Uses X-axis marker bit (bit 0) for alignment (datasheet Rev. D, p31).
    pub fn read_fifo_batch(&mut self) -> Result<Vec<SensorData>> {
        let entries = self.get_fifo_entries()? as usize;

        if entries < 3 {
            return Ok(Vec::new());
        }

        // FIFO_ENTRIES reports axis locations. Round down to complete XYZ sets.
        let num_samples = entries / 3;
        let bytes_to_read = num_samples * 9;

        let fifo_data = self.read_registers(REG_FIFO_DATA, bytes_to_read)?;

        // Parse with X-axis marker alignment (bit 0 of low byte = X marker)
        let mut samples = Vec::with_capacity(num_samples);
        let mut i = 0;

        // Find first X-axis entry for alignment
        while i + 2 < fifo_data.len() {
            if fifo_data[i + 2] & 0x02 != 0 {
                return Ok(samples); // Empty
            }
            if fifo_data[i + 2] & 0x01 != 0 {
                break; // X-axis found
            }
            i += 3; // Skip misaligned entry
        }

        // Parse aligned XYZ triplets
        while i + 8 < fifo_data.len() {
            if fifo_data[i + 2] & 0x02 != 0 {
                break;
            }

            let accel_x = parse_20bit(fifo_data[i], fifo_data[i + 1], fifo_data[i + 2]);
            let accel_y = parse_20bit(fifo_data[i + 3], fifo_data[i + 4], fifo_data[i + 5]);
            let accel_z = parse_20bit(fifo_data[i + 6], fifo_data[i + 7], fifo_data[i + 8]);

            samples.push(SensorData {
                accel_x,
                accel_y,
                accel_z,
                temperature: 0,
            });

            i += 9;
        }

        Ok(samples)
    }

    /// Stream FIFO data with periodic batch reads
    pub fn stream_fifo<F>(&mut self, batch_interval_ms: u64, mut callback: F) -> Result<u64>
    where
        F: FnMut(&[SensorData]) -> StreamControl,
    {
        if !self.fifo_enabled {
            return Err(Adxl355Error::InvalidParameter(
                "FIFO not enabled. Call enable_fifo() first.".to_string()
            ));
        }

        if batch_interval_ms < 10 || batch_interval_ms > 1000 {
            return Err(Adxl355Error::InvalidParameter(format!(
                "Batch interval must be 10-1000 ms, got {}",
                batch_interval_ms
            )));
        }

        let interval = Duration::from_millis(batch_interval_ms);
        let mut total_samples = 0u64;
        let mut next_read_time = Instant::now();

        loop {
            let batch = self.read_fifo_batch()?;

            if !batch.is_empty() {
                total_samples += batch.len() as u64;

                if callback(&batch) == StreamControl::Break {
                    break;
                }
            }

            next_read_time += interval;
            let now = Instant::now();
            if next_read_time > now {
                std::thread::sleep(next_read_time - now);
            }
        }

        Ok(total_samples)
    }

    /// Collect samples using FIFO mode
    pub fn collect_samples_fifo(&mut self, odr: OutputDataRate, num_samples: usize) -> Result<Vec<SensorData>> {
        if !self.fifo_enabled {
            self.enable_fifo(odr)?;
        }

        let mut samples = Vec::with_capacity(num_samples);

        let batch_interval_ms = 50u64;

        self.stream_fifo(batch_interval_ms, |batch| {
            samples.extend_from_slice(batch);

            if samples.len() >= num_samples {
                StreamControl::Break
            } else {
                StreamControl::Continue
            }
        })?;

        samples.truncate(num_samples);

        Ok(samples)
    }
}

impl Drop for Adxl355 {
    fn drop(&mut self) {
        // Enter standby
        let _ = self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY);

        unsafe {
            I2C_CloseChannel(self.handle);
        }
    }
}
