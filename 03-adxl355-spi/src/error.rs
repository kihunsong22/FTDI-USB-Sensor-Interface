//! Error types for ADXL355 sensor interface

use thiserror::Error;

use crate::ffi::{status_to_string, FT_STATUS, FT_OK};

/// Error type for ADXL355 operations
#[derive(Error, Debug)]
pub enum Adxl355Error {
    /// FTDI driver error
    #[error("FTDI error: {status} ({description})")]
    FtdiError {
        status: FT_STATUS,
        description: String,
    },

    /// No I2C channels found
    #[error("No I2C channels found")]
    NoChannelsFound,

    /// Invalid channel index
    #[error("Invalid channel index: {0}")]
    InvalidChannel(u32),

    /// Device communication error
    #[error("Device communication error: {0}")]
    CommunicationError(String),

    /// Invalid DEVID_AD response
    #[error("Invalid device ID: expected DEVID_AD=0xAD, got 0x{0:02X}")]
    InvalidDeviceId(u8),

    /// Invalid PARTID response
    #[error("Invalid part ID: expected PARTID=0xED (ADXL355), got 0x{0:02X}")]
    InvalidPartId(u8),

    /// Data transfer error
    #[error("Data transfer error: expected {expected} bytes, transferred {actual}")]
    TransferError { expected: u32, actual: u32 },

    /// Invalid parameter
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    /// FIFO overflow error
    #[allow(dead_code)]
    #[error("FIFO overflow: data loss occurred")]
    FifoOverflow,

    /// Device busy (NVM operation in progress)
    #[allow(dead_code)]
    #[error("Device busy (NVM operation in progress)")]
    DeviceBusy,
}

impl From<FT_STATUS> for Adxl355Error {
    fn from(status: FT_STATUS) -> Self {
        if status == FT_OK {
            panic!("Cannot convert FT_OK to error");
        }
        Adxl355Error::FtdiError {
            status,
            description: status_to_string(status).to_string(),
        }
    }
}

/// Result type for ADXL355 operations
pub type Result<T> = std::result::Result<T, Adxl355Error>;
