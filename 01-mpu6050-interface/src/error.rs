//! Error types for MPU6050 sensor interface

use thiserror::Error;

use crate::ffi::{status_to_string, FT_STATUS, FT_OK};

/// Error type for MPU6050 operations
#[derive(Error, Debug)]
pub enum Mpu6050Error {
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

    /// Invalid WHO_AM_I response
    #[error("Invalid WHO_AM_I response: expected 0x68, got 0x{0:02X}")]
    InvalidDeviceId(u8),

    /// Data transfer error
    #[error("Data transfer error: expected {expected} bytes, transferred {actual}")]
    TransferError { expected: u32, actual: u32 },

    /// Invalid parameter
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),
}

impl From<FT_STATUS> for Mpu6050Error {
    fn from(status: FT_STATUS) -> Self {
        if status == FT_OK {
            panic!("Cannot convert FT_OK to error");
        }
        Mpu6050Error::FtdiError {
            status,
            description: status_to_string(status).to_string(),
        }
    }
}

/// Result type for MPU6050 operations
pub type Result<T> = std::result::Result<T, Mpu6050Error>;
