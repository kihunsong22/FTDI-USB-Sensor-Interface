//! FT232H-based sensor interface library for ADXL355 (SPI)
//!
//! This library provides a high-level interface to the ADXL355 3-axis digital
//! accelerometer using the FTDI FT232H USB-to-SPI bridge via the libMPSSE library.

pub mod error;
mod ffi;
pub mod adxl355;
pub mod hdf5_format;
pub mod common;

pub use error::{Adxl355Error, Result};
pub use adxl355::{Adxl355, SensorData, StreamControl, Range, OutputDataRate};
pub use hdf5_format::{Hdf5Reader, Hdf5Writer, Metadata, TimestampedSample};
pub use common::{TimeKeeper, create_bar};
