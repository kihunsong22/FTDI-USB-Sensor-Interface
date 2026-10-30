//! LSM6DSOX driver over the FT232H I2C bus (libMPSSE)
//!
//! Register numbers and bit fields follow the LSM6DSOX datasheet (DS12814 Rev 4)
//! in References/LSM6DSOX.

use crate::ffi::*;
use std::fmt;
use std::ptr;
use std::thread::sleep;
use std::time::{Duration, Instant};

/// Slave addresses for SDO/SA0 low and high.
pub const ADDRESSES: [u8; 2] = [0x6A, 0x6B];
const WHO_AM_I_VALUE: u8 = 0x6C;

// Registers
const FIFO_CTRL3: u8 = 0x09;
const FIFO_CTRL4: u8 = 0x0A;
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const CTRL5_C: u8 = 0x14;
const CTRL9_XL: u8 = 0x18;
const STATUS_REG: u8 = 0x1E;
const OUT_TEMP_L: u8 = 0x20;
const FIFO_STATUS1: u8 = 0x3A;
const INTERNAL_FREQ_FINE: u8 = 0x63;
const FIFO_DATA_OUT_TAG: u8 = 0x78;

// CTRL3_C
const BDU: u8 = 0x40;
const IF_INC: u8 = 0x04;
const SW_RESET: u8 = 0x01;
// CTRL9_XL: reset value (DEN_X/Y/Z) plus I3C_disable, recommended for I2C-only use
const CTRL9_XL_I2C_ONLY: u8 = 0xE0 | 0x02;
// FIFO_CTRL4 FIFO_MODE
const FIFO_BYPASS: u8 = 0x00;
const FIFO_CONTINUOUS: u8 = 0x06;

// STATUS_REG
pub const XLDA: u8 = 0x01;
pub const GDA: u8 = 0x02;
// FIFO_STATUS2
pub const FIFO_OVR_LATCHED: u8 = 0x08;

// FIFO tags (TAG_SENSOR)
pub const TAG_GYRO: u8 = 0x01;
pub const TAG_ACCEL: u8 = 0x02;
/// Bytes per FIFO word: tag + X/Y/Z
pub const FIFO_WORD: usize = 7;

#[derive(Debug)]
pub enum Error {
    Ftdi { call: &'static str, status: FT_STATUS },
    NoChannels,
    NotFound,
    WrongId(u8),
    Nack(u8),
    Timeout(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::Ftdi { call, status } => write!(f, "{call} failed: {}", status_to_string(*status)),
            Error::NoChannels => write!(f, "no FT232H found (USB cable, D2XX driver)"),
            Error::NotFound => write!(
                f,
                "nothing answers at 0x6A or 0x6B (run `scan`; on the Adafruit FT232H the I2C switch must be on)"
            ),
            Error::WrongId(id) => write!(f, "WHO_AM_I is 0x{id:02X}, expected 0x{WHO_AM_I_VALUE:02X}"),
            Error::Nack(a) => write!(f, "device 0x{a:02X} did not acknowledge"),
            Error::Timeout(what) => write!(f, "timed out waiting for {what}"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

fn check(call: &'static str, status: FT_STATUS) -> Result<()> {
    if status == FT_OK {
        Ok(())
    } else {
        Err(Error::Ftdi { call, status })
    }
}

pub struct ChannelInfo {
    pub description: String,
    pub serial: String,
    /// Open in another process (another tool, or Motlinelab VMS)
    pub opened: bool,
}

pub fn channels() -> Result<Vec<ChannelInfo>> {
    let mut n: DWORD = 0;
    check("I2C_GetNumChannels", unsafe { I2C_GetNumChannels(&mut n) })?;
    (0..n)
        .map(|i| {
            let mut node: FT_DEVICE_LIST_INFO_NODE = unsafe { std::mem::zeroed() };
            check("I2C_GetChannelInfo", unsafe { I2C_GetChannelInfo(i, &mut node) })?;
            Ok(ChannelInfo {
                description: c_text(&node.Description),
                serial: c_text(&node.SerialNumber),
                opened: node.Flags & FT_FLAGS_OPENED != 0,
            })
        })
        .collect()
}

fn c_text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

pub struct I2cBus {
    handle: FT_HANDLE,
}

impl I2cBus {
    pub fn open(index: u32, clock_hz: u32) -> Result<Self> {
        let mut handle: FT_HANDLE = ptr::null_mut();
        check("I2C_OpenChannel", unsafe { I2C_OpenChannel(index, &mut handle) })?;
        let bus = I2cBus { handle };
        let mut config = ChannelConfig {
            ClockRate: clock_hz,
            LatencyTimer: 1,
            Options: 0,
            Pin: 0,
            currentPinState: 0,
        };
        check("I2C_InitChannel", unsafe { I2C_InitChannel(bus.handle, &mut config) })?;
        Ok(bus)
    }

    /// Addresses `addr` with a one-byte read. False when nothing acknowledges.
    pub fn probe(&mut self, addr: u8) -> Result<bool> {
        let mut byte = 0u8;
        let mut n: DWORD = 0;
        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE;
        match unsafe { I2C_DeviceRead(self.handle, addr, 1, &mut byte, &mut n, options) } {
            FT_OK => Ok(true),
            FT_DEVICE_NOT_FOUND => Ok(false),
            status => Err(Error::Ftdi { call: "I2C_DeviceRead", status }),
        }
    }

    /// Single register write. Slow mode, so every acknowledge is checked.
    pub fn write_reg(&mut self, addr: u8, reg: u8, value: u8) -> Result<()> {
        let mut buf = [reg, value];
        let mut n: DWORD = 0;
        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_BREAK_ON_NACK;
        match unsafe { I2C_DeviceWrite(self.handle, addr, 2, buf.as_mut_ptr(), &mut n, options) } {
            FT_OK if n == 2 => Ok(()),
            FT_OK | FT_DEVICE_NOT_FOUND | FT_FAILED_TO_WRITE_DEVICE => Err(Error::Nack(addr)),
            status => Err(Error::Ftdi { call: "I2C_DeviceWrite", status }),
        }
    }

    /// Burst read starting at `reg` (register write, repeated start, read).
    /// Fast transfer mode skips the acknowledge checks, so a missing device reads as 0xFF.
    pub fn read_regs(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<()> {
        let mut reg_buf = [reg];
        let mut n: DWORD = 0;
        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_BREAK_ON_NACK
            | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;
        check("I2C_DeviceWrite", unsafe {
            I2C_DeviceWrite(self.handle, addr, 1, reg_buf.as_mut_ptr(), &mut n, options)
        })?;
        let options = I2C_TRANSFER_OPTIONS_START_BIT
            | I2C_TRANSFER_OPTIONS_STOP_BIT
            | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE
            | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;
        check("I2C_DeviceRead", unsafe {
            I2C_DeviceRead(self.handle, addr, buf.len() as DWORD, buf.as_mut_ptr(), &mut n, options)
        })
    }

    /// Pin-level wiring check, ADBUS bytes:
    /// - D2 read with D1 driven low, high, low. D2 following D1 means D1 and D2 are
    ///   joined (Adafruit FT232H I2C switch on).
    /// - D0..D2 driven low, released, read 20 ms later. A pin back at 1 has an external
    ///   pull-up, which a powered sensor breakout provides.
    /// The next libMPSSE transfer sets the pins up again.
    pub fn pin_check(&mut self) -> Result<([u8; 3], u8)> {
        // 0x80 value dir: set ADBUS, 0x81: read ADBUS, 0x87: send immediate
        let follow = [0x80, 0x00, 0x02, 0x81, 0x80, 0x02, 0x02, 0x81, 0x80, 0x00, 0x02, 0x81, 0x87];
        let mut d1_d2 = [0u8; 3];
        check("FT_Purge", unsafe { FT_Purge(self.handle, FT_PURGE_RX | FT_PURGE_TX) })?;
        self.mpsse(&follow, &mut d1_d2)?;
        self.mpsse(&[0x80, 0x00, 0x07, 0x80, 0x00, 0x00, 0x87], &mut [])?;
        sleep(Duration::from_millis(20));
        let mut released = [0u8];
        self.mpsse(&[0x81, 0x87], &mut released)?;
        Ok((d1_d2, released[0]))
    }

    fn mpsse(&mut self, cmd: &[u8], reply: &mut [u8]) -> Result<()> {
        let mut n: DWORD = 0;
        check("FT_Write", unsafe { FT_Write(self.handle, cmd.as_ptr().cast(), cmd.len() as DWORD, &mut n) })?;
        if !reply.is_empty() {
            check("FT_Read", unsafe { FT_Read(self.handle, reply.as_mut_ptr().cast(), reply.len() as DWORD, &mut n) })?;
            if n as usize != reply.len() {
                return Err(Error::Timeout("MPSSE reply"));
            }
        }
        Ok(())
    }
}

impl Drop for I2cBus {
    fn drop(&mut self) {
        unsafe { I2C_CloseChannel(self.handle) };
    }
}

/// Output data rate, shared by the accelerometer and gyroscope in high-performance mode.
#[derive(Clone, Copy, Debug)]
pub struct Odr {
    pub nominal: f64,
    /// ODR_XL/ODR_G and BDR_XL/BDR_GY field value
    code: u8,
    /// ODRcoeff (AN5272 Table 38)
    coeff: u32,
}

const ODRS: [(f64, u8, u32); 10] = [
    (12.5, 1, 512),
    (26.0, 2, 256),
    (52.0, 3, 128),
    (104.0, 4, 64),
    (208.0, 5, 32),
    (417.0, 6, 16),
    (833.0, 7, 8),
    (1667.0, 8, 4),
    (3333.0, 9, 2),
    (6667.0, 10, 1),
];

impl Odr {
    pub fn from_hz(hz: f64) -> Option<Self> {
        ODRS.iter()
            .find(|(nominal, _, _)| (hz - nominal).abs() <= nominal * 0.02)
            .map(|&(nominal, code, coeff)| Odr { nominal, code, coeff })
    }

    pub fn all() -> impl Iterator<Item = Odr> {
        ODRS.iter().map(|&(nominal, code, coeff)| Odr { nominal, code, coeff })
    }

    /// Trimmed rate from INTERNAL_FREQ_FINE (AN5272 6.3).
    pub fn actual(self, freq_fine: i8) -> f64 {
        6667.0 * (1.0 + 0.0015 * freq_fine as f64) / self.coeff as f64
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub odr: Odr,
    pub accel_g: u16,
    pub gyro_dps: u16,
    pub gyro_on: bool,
}

impl Settings {
    pub fn new(odr: Odr, accel_g: u16, gyro_dps: u16) -> std::result::Result<Self, String> {
        let s = Settings { odr, accel_g, gyro_dps, gyro_on: true };
        if s.accel().is_none() {
            return Err(format!("accelerometer full scale {accel_g} g: use 2, 4, 8 or 16"));
        }
        if s.gyro().is_none() {
            return Err(format!("gyroscope full scale {gyro_dps} dps: use 125, 250, 500, 1000 or 2000"));
        }
        Ok(s)
    }

    /// CTRL1_XL FS bits and g/LSB
    fn accel(&self) -> Option<(u8, f64)> {
        Some(match self.accel_g {
            2 => (0x00, 0.061e-3),
            4 => (0x08, 0.122e-3),
            8 => (0x0C, 0.244e-3),
            16 => (0x04, 0.488e-3),
            _ => return None,
        })
    }

    /// CTRL2_G FS and FS_125 bits and dps/LSB
    fn gyro(&self) -> Option<(u8, f64)> {
        Some(match self.gyro_dps {
            125 => (0x02, 4.375e-3),
            250 => (0x00, 8.75e-3),
            500 => (0x04, 17.5e-3),
            1000 => (0x08, 35.0e-3),
            2000 => (0x0C, 70.0e-3),
            _ => return None,
        })
    }

    pub fn g_per_lsb(&self) -> f64 {
        self.accel().unwrap().1
    }

    pub fn dps_per_lsb(&self) -> f64 {
        self.gyro().unwrap().1
    }
}

/// One read of the output registers, raw counts.
#[derive(Clone, Copy, Debug, Default)]
pub struct Raw {
    pub temp: i16,
    pub gyro: [i16; 3],
    pub accel: [i16; 3],
}

impl Raw {
    pub fn temp_c(&self) -> f64 {
        25.0 + self.temp as f64 / 256.0
    }
}

pub fn xyz(b: &[u8]) -> [i16; 3] {
    [
        i16::from_le_bytes([b[0], b[1]]),
        i16::from_le_bytes([b[2], b[3]]),
        i16::from_le_bytes([b[4], b[5]]),
    ]
}

pub struct Lsm6dsox {
    bus: I2cBus,
    pub address: u8,
}

impl Lsm6dsox {
    /// Finds the sensor at 0x6A or 0x6B, checks WHO_AM_I and resets it.
    pub fn open(mut bus: I2cBus) -> Result<Self> {
        let mut address = None;
        for a in ADDRESSES {
            if bus.probe(a)? {
                address = Some(a);
                break;
            }
        }
        let mut s = Lsm6dsox { bus, address: address.ok_or(Error::NotFound)? };
        let id = s.read(WHO_AM_I)?;
        if id != WHO_AM_I_VALUE {
            return Err(Error::WrongId(id));
        }
        s.reset()?;
        Ok(s)
    }

    fn read(&mut self, reg: u8) -> Result<u8> {
        let mut b = [0u8];
        self.bus.read_regs(self.address, reg, &mut b)?;
        Ok(b[0])
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<()> {
        self.bus.write_reg(self.address, reg, value)
    }

    /// Control registers back to reset values, then block data update and I2C only.
    pub fn reset(&mut self) -> Result<()> {
        self.write(CTRL3_C, SW_RESET)?;
        let t = Instant::now();
        while self.read(CTRL3_C)? & SW_RESET != 0 {
            if t.elapsed() > Duration::from_millis(50) {
                return Err(Error::Timeout("software reset"));
            }
        }
        self.write(CTRL3_C, BDU | IF_INC)?;
        self.write(CTRL9_XL, CTRL9_XL_I2C_ONLY)
    }

    pub fn freq_fine(&mut self) -> Result<i8> {
        Ok(self.read(INTERNAL_FREQ_FINE)? as i8)
    }

    /// CTRL1_XL (10h) to CTRL10_C (19h)
    pub fn control_registers(&mut self) -> Result<[u8; 10]> {
        let mut regs = [0u8; 10];
        self.bus.read_regs(self.address, CTRL1_XL, &mut regs)?;
        Ok(regs)
    }

    /// Starts both sensors (or the accelerometer alone) at one rate in high-performance mode.
    pub fn start(&mut self, s: &Settings) -> Result<()> {
        self.write(CTRL1_XL, s.odr.code << 4 | s.accel().unwrap().0)?;
        let ctrl2_g = if s.gyro_on { s.odr.code << 4 | s.gyro().unwrap().0 } else { 0 };
        self.write(CTRL2_G, ctrl2_g)
    }

    pub fn power_down(&mut self) -> Result<()> {
        self.write(FIFO_CTRL4, FIFO_BYPASS)?;
        self.write(CTRL1_XL, 0)?;
        self.write(CTRL2_G, 0)
    }

    pub fn status(&mut self) -> Result<u8> {
        self.read(STATUS_REG)
    }

    /// Temperature, gyroscope and accelerometer outputs in one burst (20h to 2Dh).
    pub fn outputs(&mut self) -> Result<Raw> {
        let mut b = [0u8; 14];
        self.bus.read_regs(self.address, OUT_TEMP_L, &mut b)?;
        Ok(Raw {
            temp: i16::from_le_bytes([b[0], b[1]]),
            gyro: xyz(&b[2..8]),
            accel: xyz(&b[8..14]),
        })
    }

    fn wait(&mut self, ready: u8) -> Result<()> {
        let t = Instant::now();
        while self.status()? & ready != ready {
            if t.elapsed() > Duration::from_millis(200) {
                return Err(Error::Timeout("data ready"));
            }
        }
        Ok(())
    }

    /// Batches the running sensors at the ODR into the FIFO, continuous mode.
    pub fn fifo_start(&mut self, s: &Settings) -> Result<()> {
        self.write(FIFO_CTRL4, FIFO_BYPASS)?; // empties the FIFO
        let bdr_gy = if s.gyro_on { s.odr.code } else { 0 };
        self.write(FIFO_CTRL3, bdr_gy << 4 | s.odr.code)?;
        self.write(FIFO_CTRL4, FIFO_CONTINUOUS)
    }

    /// Unread words and the FIFO_STATUS2 flags. Reading clears FIFO_OVR_LATCHED.
    pub fn fifo_status(&mut self) -> Result<(usize, u8)> {
        let mut b = [0u8; 2];
        self.bus.read_regs(self.address, FIFO_STATUS1, &mut b)?;
        Ok((b[0] as usize | (b[1] as usize & 0x03) << 8, b[1]))
    }

    /// Reads `buf.len() / 7` words in one burst. The address wraps from 7Eh back to 78h.
    pub fn fifo_read(&mut self, buf: &mut [u8]) -> Result<()> {
        self.bus.read_regs(self.address, FIFO_DATA_OUT_TAG, buf)
    }

    /// Self-test output change per axis, accelerometer in mg and gyroscope in dps
    /// (datasheet procedure; limits 50..1700 mg and 150..700 dps). Keep the sensor still.
    pub fn self_test(&mut self) -> Result<([f64; 3], [f64; 3])> {
        self.reset()?;

        self.write(CTRL1_XL, 0x38)?; // 52 Hz, 4 g
        let off = self.settled_mean(XLDA, |r| r.accel)?;
        self.write(CTRL5_C, 0x01)?; // ST_XL positive
        let on = self.settled_mean(XLDA, |r| r.accel)?;
        self.write(CTRL5_C, 0)?;
        self.write(CTRL1_XL, 0)?;
        let accel = [0, 1, 2].map(|i| (on[i] - off[i]).abs() * 0.122);

        self.write(CTRL2_G, 0x5C)?; // 208 Hz, 2000 dps
        let off = self.settled_mean(GDA, |r| r.gyro)?;
        self.write(CTRL5_C, 0x04)?; // ST_G positive
        let on = self.settled_mean(GDA, |r| r.gyro)?;
        self.write(CTRL5_C, 0)?;
        self.write(CTRL2_G, 0)?;
        let gyro = [0, 1, 2].map(|i| (on[i] - off[i]).abs() * 0.070);

        Ok((accel, gyro))
    }

    /// Waits 100 ms, drops the first sample, then averages five.
    fn settled_mean(&mut self, ready: u8, pick: fn(&Raw) -> [i16; 3]) -> Result<[f64; 3]> {
        sleep(Duration::from_millis(100));
        self.wait(ready)?;
        self.outputs()?;
        let mut sum = [0.0; 3];
        for _ in 0..5 {
            self.wait(ready)?;
            let v = pick(&self.outputs()?);
            for i in 0..3 {
                sum[i] += v[i] as f64 / 5.0;
            }
        }
        Ok(sum)
    }
}

impl Drop for Lsm6dsox {
    fn drop(&mut self) {
        let _ = self.power_down();
    }
}
