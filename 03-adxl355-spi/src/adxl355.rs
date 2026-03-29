//! ADXL355 sensor driver using FTDI MPSSE SPI interface

use crate::error::{Adxl355Error, Result};
use crate::ffi::*;
use std::ptr;
use std::time::{Duration, Instant};

// Device identification registers
const REG_DEVID_AD: u8 = 0x00;
const REG_PARTID: u8 = 0x02;

// Status and FIFO
const REG_FIFO_ENTRIES: u8 = 0x05;

// Temperature data (12-bit: TEMP2[3:0] = bits[11:8], TEMP1[7:0] = bits[7:0])
const REG_TEMP2: u8 = 0x06;

// Accelerometer data
const REG_XDATA3: u8 = 0x08;

// FIFO data register
const REG_FIFO_DATA: u8 = 0x11;

// Configuration registers
const REG_FILTER: u8 = 0x28;
const REG_RANGE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_RESET: u8 = 0x2F;

// Expected device ID values
const DEVID_AD_VALUE: u8 = 0xAD;
const PARTID_VALUE: u8 = 0xED;

// Power control bits
const POWER_CTL_STANDBY: u8 = 0x01;

// Reset command
const RESET_CODE: u8 = 0x52;

/// Measurement range
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Range {
    G2 = 0x01,
    G4 = 0x02,
    G8 = 0x03,
}

impl Range {
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
    Odr4000 = 0x00,
    Odr2000 = 0x01,
    Odr1000 = 0x02,
    Odr500 = 0x03,
    Odr250 = 0x04,
    Odr125 = 0x05,
    Odr62_5 = 0x06,
    Odr31_25 = 0x07,
    Odr15_625 = 0x08,
    Odr7_813 = 0x09,
    Odr3_906 = 0x0A,
}

impl OutputDataRate {
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
    Continue,
    Break,
}

/// Sensor data structure
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub accel_x: i32,
    pub accel_y: i32,
    pub accel_z: i32,
    pub temperature: u16,
}

impl SensorData {
    pub fn accel_to_g(&self, range: Range) -> (f32, f32, f32) {
        let scale = range.scale_factor();
        (
            self.accel_x as f32 / scale,
            self.accel_y as f32 / scale,
            self.accel_z as f32 / scale,
        )
    }

    pub fn temperature_c(&self) -> f32 {
        ((self.temperature as f32 - 1885.0) / -9.05) + 25.0
    }
}

/// Parse 20-bit two's complement value from 3 bytes
fn parse_20bit(high: u8, mid: u8, low: u8) -> i32 {
    let raw = ((high as u32) << 12) | ((mid as u32) << 4) | ((low as u32) >> 4);
    if raw & 0x80000 != 0 {
        (raw | 0xFFF00000) as i32
    } else {
        raw as i32
    }
}

/// ADXL355 sensor interface (SPI)
pub struct Adxl355 {
    handle: FT_HANDLE,
    range: Range,
    fifo_enabled: bool,
}

impl Adxl355 {
    /// Open SPI channel without sensor init (for diagnostics)
    pub fn new_uninitialized(channel_index: u32) -> Result<Self> {
        let handle = Self::open_channel(channel_index)?;
        Ok(Adxl355 {
            handle,
            range: Range::G2,
            fifo_enabled: false,
        })
    }

    /// Read a raw register value (public, for diagnostics)
    pub fn read_reg(&mut self, reg: u8) -> Result<u8> {
        self.read_register(reg)
    }

    /// Diagnostic: raw SPI read showing all bytes including dummy
    pub fn read_reg_debug(&mut self, reg: u8) -> Result<(Vec<u8>, Vec<u8>)> {
        use crate::ffi::*;
        // Send 3 bytes to see what comes back at each position
        let mut out_buf = [(reg << 1) | 0x01, 0x00, 0x00];
        let mut in_buf = [0u8; 3];
        let mut transferred: DWORD = 0;

        let options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

        let status = unsafe {
            SPI_ReadWrite(
                self.handle,
                in_buf.as_mut_ptr(),
                out_buf.as_mut_ptr(),
                3,
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        Ok((out_buf.to_vec(), in_buf.to_vec()))
    }

    /// Create a new ADXL355 instance over SPI
    pub fn new(channel_index: u32) -> Result<Self> {
        let handle = Self::open_channel(channel_index)?;

        let mut sensor = Adxl355 {
            handle,
            range: Range::G2,
            fifo_enabled: false,
        };

        sensor.init()?;

        Ok(sensor)
    }

    /// Open and configure an SPI channel
    fn open_channel(channel_index: u32) -> Result<FT_HANDLE> {
        let mut num_channels: DWORD = 0;
        let status = unsafe { SPI_GetNumChannels(&mut num_channels) };
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
        let status = unsafe { SPI_OpenChannel(channel_index, &mut handle) };
        if status != FT_OK {
            return Err(status.into());
        }

        let mut config = ChannelConfig {
            ClockRate: 1_000_000, // 1 MHz (start conservative, increase after verification)
            LatencyTimer: 1,
            configOptions: SPI_CONFIG_OPTION_MODE0
                | SPI_CONFIG_OPTION_CS_DBUS3
                | SPI_CONFIG_OPTION_CS_ACTIVELOW,
            Pin: 0,
            currentPinState: 0,
        };

        let status = unsafe { SPI_InitChannel(handle, &mut config) };
        if status != FT_OK {
            unsafe { SPI_CloseChannel(handle) };
            return Err(status.into());
        }

        Ok(handle)
    }

    /// Initialize the ADXL355 sensor
    fn init(&mut self) -> Result<()> {
        // Debug: raw SPI_ReadWrite matching the exact working diagnostic pattern
        {
            let mut out = [0x80u8, 0x00, 0x00];
            let mut inp = [0u8; 3];
            let mut xfer: DWORD = 0;
            let opts = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
                | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE
                | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;
            let st = unsafe {
                SPI_ReadWrite(self.handle, inp.as_mut_ptr(), out.as_mut_ptr(), 3, &mut xfer, opts)
            };
            eprintln!("DEBUG raw SPI_ReadWrite: status={} RX={:02X} {:02X} {:02X}",
                st, inp[0], inp[1], inp[2]);
        }

        let devid_ad = self.read_register(REG_DEVID_AD)?;
        eprintln!("DEBUG read_register(0x00) = 0x{:02X}", devid_ad);
        if devid_ad != DEVID_AD_VALUE {
            return Err(Adxl355Error::InvalidDeviceId(devid_ad));
        }

        let partid = self.read_register(REG_PARTID)?;
        if partid != PARTID_VALUE {
            return Err(Adxl355Error::InvalidPartId(partid));
        }

        self.write_register(REG_RANGE, Range::G2 as u8)?;
        self.write_register(REG_FILTER, OutputDataRate::Odr1000 as u8)?;
        self.write_register(REG_POWER_CTL, 0x00)?;

        std::thread::sleep(Duration::from_millis(10));

        Ok(())
    }

    // ========================================================================
    // SPI register operations
    // ========================================================================

    /// Write a single byte to a register
    /// SPI write: CS low → [addr & 0x7F, value] → CS high
    fn write_register(&mut self, reg: u8, value: u8) -> Result<()> {
        let mut buffer = [reg & 0x7F, value];
        let mut transferred: DWORD = 0;

        let options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

        let status = unsafe {
            SPI_Write(
                self.handle,
                buffer.as_mut_ptr(),
                2,
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
    /// SPI read: CS low → send [addr | 0x80] → receive [data] simultaneously → CS high
    fn read_register(&mut self, reg: u8) -> Result<u8> {
        // ADXL355 responds during the command byte — data at in_buf[0]
        // Send 2 bytes to get 1 byte of register data (response starts at byte 0)
        let mut out_buf = [reg | 0x80, 0x00];
        let mut in_buf = [0u8; 2];
        let mut transferred: DWORD = 0;

        let options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

        let status = unsafe {
            SPI_ReadWrite(
                self.handle,
                in_buf.as_mut_ptr(),
                out_buf.as_mut_ptr(),
                2,
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        Ok(in_buf[0])
    }

    /// Read multiple bytes from consecutive registers (or FIFO)
    /// SPI read: CS low → send [addr | 0x80, 0x00 × count] → response starts at in_buf[0]
    fn read_registers(&mut self, reg: u8, count: usize) -> Result<Vec<u8>> {
        // ADXL355 responds starting at byte 0 (overlapping command byte)
        // Send 1 command + count bytes, read count+1 bytes, data starts at in_buf[0]
        let total = 1 + count;
        let mut out_buf = vec![0u8; total];
        out_buf[0] = reg | 0x80;

        let mut in_buf = vec![0u8; total];
        let mut transferred: DWORD = 0;

        let options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE
            | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

        let status = unsafe {
            SPI_ReadWrite(
                self.handle,
                in_buf.as_mut_ptr(),
                out_buf.as_mut_ptr(),
                total as DWORD,
                &mut transferred,
                options,
            )
        };

        if status != FT_OK {
            return Err(status.into());
        }

        // Data starts at byte 0, take first `count` bytes
        Ok(in_buf[..count].to_vec())
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    pub fn set_range(&mut self, range: Range) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        let current = self.read_register(REG_RANGE)?;
        let new_val = (current & 0xFC) | (range as u8);
        self.write_register(REG_RANGE, new_val)?;

        self.range = range;

        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    pub fn set_odr(&mut self, odr: OutputDataRate) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        let current = self.read_register(REG_FILTER)?;
        let new_val = (current & 0xF0) | (odr as u8);
        self.write_register(REG_FILTER, new_val)?;

        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    pub fn get_range(&self) -> Range {
        self.range
    }

    // ========================================================================
    // Data reading
    // ========================================================================

    pub fn read_temperature(&mut self) -> Result<u16> {
        let data = self.read_registers(REG_TEMP2, 2)?;
        let temp = ((data[0] as u16 & 0x0F) << 8) | (data[1] as u16);
        Ok(temp)
    }

    pub fn read_accel(&mut self) -> Result<(i32, i32, i32)> {
        let data = self.read_registers(REG_XDATA3, 9)?;
        let x = parse_20bit(data[0], data[1], data[2]);
        let y = parse_20bit(data[3], data[4], data[5]);
        let z = parse_20bit(data[6], data[7], data[8]);
        Ok((x, y, z))
    }

    pub fn read_all(&mut self) -> Result<SensorData> {
        let data = self.read_registers(REG_TEMP2, 11)?;

        let temperature = ((data[0] as u16 & 0x0F) << 8) | (data[1] as u16);
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

    pub fn enable_fifo(&mut self, odr: OutputDataRate) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        std::thread::sleep(Duration::from_millis(5));

        let current_filter = self.read_register(REG_FILTER)?;
        let new_filter = (current_filter & 0xF0) | (odr as u8);
        self.write_register(REG_FILTER, new_filter)?;

        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(10));

        self.fifo_enabled = true;

        let _ = self.read_fifo_batch();

        Ok(())
    }

    pub fn disable_fifo(&mut self) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        self.fifo_enabled = false;
        Ok(())
    }

    pub fn get_fifo_entries(&mut self) -> Result<u8> {
        self.read_register(REG_FIFO_ENTRIES)
    }

    /// Read all available samples from the FIFO (single SPI transaction for data)
    pub fn read_fifo_batch(&mut self) -> Result<Vec<SensorData>> {
        let entries = self.get_fifo_entries()? as usize;

        if entries < 3 {
            return Ok(Vec::new());
        }

        let num_samples = entries / 3;
        let bytes_to_read = num_samples * 9;

        let fifo_data = self.read_registers(REG_FIFO_DATA, bytes_to_read)?;

        // Parse with X-axis marker alignment (bit 0 of low byte = X marker)
        let mut samples = Vec::with_capacity(num_samples);
        let mut i = 0;

        // Find first X-axis entry for alignment
        while i + 2 < fifo_data.len() {
            if fifo_data[i + 2] & 0x02 != 0 {
                return Ok(samples);
            }
            if fifo_data[i + 2] & 0x01 != 0 {
                break;
            }
            i += 3;
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

    pub fn collect_samples_fifo(&mut self, odr: OutputDataRate, num_samples: usize) -> Result<Vec<SensorData>> {
        if !self.fifo_enabled {
            self.enable_fifo(odr)?;
        }

        let mut samples = Vec::with_capacity(num_samples);

        self.stream_fifo(50, |batch| {
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
        let _ = self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY);

        unsafe {
            SPI_CloseChannel(self.handle);
        }
    }
}
