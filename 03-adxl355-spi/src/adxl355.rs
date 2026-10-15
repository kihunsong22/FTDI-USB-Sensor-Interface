//! ADXL355 sensor driver using FTDI MPSSE SPI interface

use crate::error::{Adxl355Error, Result};
use crate::ffi::*;
use std::ptr;
use std::time::{Duration, Instant};

// Device identification registers
const REG_DEVID_AD: u8 = 0x00;
const REG_PARTID: u8 = 0x02;

// Status and FIFO
const REG_STATUS: u8 = 0x04;
const REG_FIFO_ENTRIES: u8 = 0x05;

// STATUS.FIFO_OVR: latched when a sample is dropped because the FIFO was full,
// cleared when STATUS is read
const STATUS_FIFO_OVR: u8 = 0x04;

// FIFO holds 96 entries = 32 XYZ samples
const FIFO_CAPACITY_SAMPLES: usize = 32;
const FIFO_X_MARKER: u8 = 0x01;
const FIFO_EMPTY_MARKER: u8 = 0x02;

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

/// Result of a FIFO batch read, including overflow detection
#[derive(Debug, Clone)]
pub struct FifoBatchResult {
    pub samples: Vec<SensorData>,
    pub overflow_detected: bool,
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

/// Parse FIFO_DATA bytes into XYZ samples. Each entry is 3 bytes; in the last
/// byte, bit 0 marks an X-axis entry and bit 1 marks a read of an empty FIFO.
fn parse_fifo(data: &[u8]) -> Vec<SensorData> {
    let mut samples = Vec::with_capacity(data.len() / 9);
    let mut i = 0;

    // Align to the first X-axis entry
    while i + 2 < data.len() && data[i + 2] & FIFO_X_MARKER == 0 {
        if data[i + 2] & FIFO_EMPTY_MARKER != 0 {
            return samples;
        }
        i += 3;
    }

    // Parse aligned XYZ triplets
    while i + 8 < data.len() && data[i + 2] & FIFO_EMPTY_MARKER == 0 {
        samples.push(SensorData {
            accel_x: parse_20bit(data[i], data[i + 1], data[i + 2]),
            accel_y: parse_20bit(data[i + 3], data[i + 4], data[i + 5]),
            accel_z: parse_20bit(data[i + 6], data[i + 7], data[i + 8]),
            temperature: 0,
        });
        i += 9;
    }

    samples
}

/// Turn an SPI_Write/SPI_Read status and byte count into a Result
fn check_transfer(status: FT_STATUS, expected: DWORD, actual: DWORD) -> Result<()> {
    if status != FT_OK {
        return Err(status.into());
    }
    if actual != expected {
        return Err(Adxl355Error::TransferError { expected, actual });
    }
    Ok(())
}

/// ADXL355 sensor interface (SPI)
pub struct Adxl355 {
    handle: FT_HANDLE,
    range: Range,
    odr: OutputDataRate,
    fifo_enabled: bool,
}

impl Adxl355 {
    /// Open SPI channel without sensor init (for diagnostics)
    pub fn new_uninitialized(channel_index: u32) -> Result<Self> {
        let handle = Self::open_channel(channel_index)?;
        Ok(Adxl355 {
            handle,
            range: Range::G2,
            odr: OutputDataRate::Odr4000, // FILTER register reset value
            fifo_enabled: false,
        })
    }

    /// Read a raw register value (public, for diagnostics)
    pub fn read_reg(&mut self, reg: u8) -> Result<u8> {
        self.read_register(reg)
    }

    /// Create a new ADXL355 instance over SPI
    pub fn new(channel_index: u32) -> Result<Self> {
        let handle = Self::open_channel(channel_index)?;

        let mut sensor = Adxl355 {
            handle,
            range: Range::G2,
            odr: OutputDataRate::Odr1000, // set by init()
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
        let devid_ad = self.read_register(REG_DEVID_AD)?;
        if devid_ad != DEVID_AD_VALUE {
            return Err(Adxl355Error::InvalidDeviceId(devid_ad));
        }

        let partid = self.read_register(REG_PARTID)?;
        if partid != PARTID_VALUE {
            return Err(Adxl355Error::InvalidPartId(partid));
        }

        // Read-modify-write to preserve upper bits (I2C_HS, INT_POL) — same as set_range()
        let current_range = self.read_register(REG_RANGE)?;
        self.write_register(REG_RANGE, (current_range & 0xFC) | (Range::G2 as u8))?;
        self.write_register(REG_FILTER, OutputDataRate::Odr1000 as u8)?;
        self.write_register(REG_POWER_CTL, 0x00)?;

        std::thread::sleep(Duration::from_millis(10));

        Ok(())
    }

    // ========================================================================
    // SPI register operations
    //
    // ADXL355 SPI protocol:
    //   Command byte = (register_address << 1) | RNW
    //     RNW = 1 for read, 0 for write
    //
    // FT232H MPSSE split Write+Read transport:
    //   The MPSSE SPI_Write sends the command byte (MOSI only).
    //   SPI_Read clocks in the response (MISO only), exactly `count` bytes.
    //
    //   Do not call SPI_ReadWrite: this libMPSSE build answers it with the
    //   MPSSE bad-command reply (0xFA 0x00), leaving 2 stray bytes in the RX
    //   queue that shift every later read. Reading extra bytes to skip them
    //   also clocks extra bytes out of FIFO_DATA, dropping ~1 sample per read.
    // ========================================================================

    /// Write a single byte to a register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<()> {
        let cmd = (reg << 1) | 0x00; // write command
        let mut buffer = [cmd, value];
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

        check_transfer(status, 2, transferred)
    }

    /// Read a single byte from a register
    fn read_register(&mut self, reg: u8) -> Result<u8> {
        Ok(self.read_registers(reg, 1)?[0])
    }

    /// Read multiple bytes from consecutive registers
    fn read_registers(&mut self, reg: u8, count: usize) -> Result<Vec<u8>> {
        let cmd = (reg << 1) | 0x01; // read command
        let mut cmd_buf = [cmd];
        let mut transferred: DWORD = 0;

        // Write command byte (CS asserted, stays low)
        let status = unsafe {
            SPI_Write(
                self.handle,
                cmd_buf.as_mut_ptr(),
                1,
                &mut transferred,
                SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
                    | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE,
            )
        };
        check_transfer(status, 1, transferred)?;

        // Read exactly count bytes, then CS deasserted
        let mut data = vec![0u8; count];
        transferred = 0;
        let status = unsafe {
            SPI_Read(
                self.handle,
                data.as_mut_ptr(),
                count as DWORD,
                &mut transferred,
                SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES
                    | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE,
            )
        };
        check_transfer(status, count as DWORD, transferred)?;

        Ok(data)
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
        self.odr = odr;

        self.write_register(REG_POWER_CTL, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    pub fn get_range(&self) -> Range {
        self.range
    }

    pub fn get_odr(&self) -> OutputDataRate {
        self.odr
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
        self.set_odr(odr)?;
        std::thread::sleep(Duration::from_millis(5));

        self.fifo_enabled = true;

        // Drop samples from the switch-over and clear STATUS.FIFO_OVR. At high
        // ODR the FIFO overflows during the sleeps above, and keeps overflowing
        // while the first drain reads it, so repeat until the flag stays clear.
        for _ in 0..4 {
            if !self.read_fifo_batch_checked()?.overflow_detected {
                break;
            }
        }

        Ok(())
    }

    /// Stop measurement (standby). The ADXL355 FIFO cannot be disabled on its
    /// own; it always fills while the sensor is measuring.
    pub fn standby(&mut self) -> Result<()> {
        self.write_register(REG_POWER_CTL, POWER_CTL_STANDBY)?;
        self.fifo_enabled = false;
        Ok(())
    }

    pub fn get_fifo_entries(&mut self) -> Result<u8> {
        // Bit 7 is reserved; only bits [6:0] hold the entry count (0–96)
        Ok(self.read_register(REG_FIFO_ENTRIES)? & 0x7F)
    }

    /// Read all available samples from the FIFO
    pub fn read_fifo_batch(&mut self) -> Result<Vec<SensorData>> {
        Ok(self.read_fifo_batch_checked()?.samples)
    }

    /// Read all available samples from the FIFO, with overflow detection.
    ///
    /// STATUS and FIFO_ENTRIES are adjacent, so one transaction returns both;
    /// STATUS.FIFO_OVR reports whether samples were dropped since the last read.
    pub fn read_fifo_batch_checked(&mut self) -> Result<FifoBatchResult> {
        let regs = self.read_registers(REG_STATUS, 2)?;
        let overflow_detected = regs[0] & STATUS_FIFO_OVR != 0;
        // Bit 7 of FIFO_ENTRIES is reserved; bits [6:0] hold the entry count (0–96)
        let num_samples = (regs[1] & 0x7F) as usize / 3;

        let samples = if num_samples == 0 {
            Vec::new()
        } else {
            parse_fifo(&self.read_registers(REG_FIFO_DATA, num_samples * 9)?)
        };

        Ok(FifoBatchResult { samples, overflow_detected })
    }

    /// Longest FIFO poll interval at the current ODR, leaving 4x headroom before
    /// the 32-sample FIFO fills (Windows sleeps can overshoot by a few ms)
    pub fn max_fifo_poll_interval(&self) -> Duration {
        Duration::from_secs_f64(FIFO_CAPACITY_SAMPLES as f64 / 4.0 / self.odr.as_hz())
    }

    /// Poll the FIFO every `batch_interval_ms` and pass each batch to `callback`.
    /// Samples already in the FIFO are discarded; fails with `FifoOverflow` if
    /// samples are dropped while streaming.
    pub fn stream_fifo<F>(&mut self, batch_interval_ms: u64, mut callback: F) -> Result<u64>
    where
        F: FnMut(&[SensorData]) -> StreamControl,
    {
        if !self.fifo_enabled {
            return Err(Adxl355Error::InvalidParameter(
                "FIFO not enabled. Call enable_fifo() first.".to_string()
            ));
        }

        let max_ms = (self.max_fifo_poll_interval().as_millis() as u64).max(1);
        if batch_interval_ms == 0 || batch_interval_ms > max_ms {
            return Err(Adxl355Error::InvalidParameter(format!(
                "Batch interval must be 1-{} ms at {} Hz ODR (FIFO holds {} samples), got {}",
                max_ms, self.odr.as_hz(), FIFO_CAPACITY_SAMPLES, batch_interval_ms
            )));
        }

        self.read_fifo_batch_checked()?;

        let interval = Duration::from_millis(batch_interval_ms);
        let mut total_samples = 0u64;
        let mut next_read_time = Instant::now();

        loop {
            let result = self.read_fifo_batch_checked()?;
            if result.overflow_detected {
                return Err(Adxl355Error::FifoOverflow);
            }
            let batch = result.samples;

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
        if !self.fifo_enabled || self.odr != odr {
            self.enable_fifo(odr)?;
        }

        let mut samples = Vec::with_capacity(num_samples);
        let interval_ms = (self.max_fifo_poll_interval().as_millis() as u64).max(1);

        self.stream_fifo(interval_ms, |batch| {
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a 20-bit value as a FIFO/data-register entry with marker bits
    fn entry(value: i32, markers: u8) -> [u8; 3] {
        let raw = (value as u32) & 0xFFFFF;
        [(raw >> 12) as u8, (raw >> 4) as u8, ((raw << 4) as u8) | markers]
    }

    #[test]
    fn parse_20bit_sign_extends() {
        for v in [0, 1, -1, 256_000, -256_000, 524_287, -524_288] {
            let e = entry(v, 0);
            assert_eq!(parse_20bit(e[0], e[1], e[2]), v);
        }
    }

    #[test]
    fn parse_fifo_aligns_to_x_and_stops_at_empty() {
        let mut data = Vec::new();
        data.extend(entry(5, 0)); // trailing Z entry before the first X
        data.extend(entry(1, FIFO_X_MARKER));
        data.extend(entry(-2, 0));
        data.extend(entry(3, 0));
        data.extend(entry(0, FIFO_EMPTY_MARKER));
        data.extend(entry(0, 0));
        data.extend(entry(0, 0));
        let samples = parse_fifo(&data);
        assert_eq!(samples.len(), 1);
        assert_eq!((samples[0].accel_x, samples[0].accel_y, samples[0].accel_z), (1, -2, 3));
    }
}
