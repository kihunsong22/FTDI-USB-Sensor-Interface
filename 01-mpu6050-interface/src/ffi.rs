//! FFI bindings for FTDI libMPSSE I2C library
//!
//! This module provides low-level bindings to the libMPSSE I2C DLL.
//! Based on libmpsse_i2c.h from FTDI's libMPSSE library.

#![allow(non_camel_case_types)]
#![allow(dead_code)]

use std::ffi::c_void;

// Windows types (matching WinTypes.h and ftd2xx.h)
pub type DWORD = u32;
pub type UCHAR = u8;
pub type USHORT = u16;
pub type LPDWORD = *mut DWORD;
pub type BOOL = i32;

// FTDI types
pub type FT_STATUS = DWORD;
pub type FT_HANDLE = *mut c_void;

// FT_STATUS return codes (from ftd2xx.h)
pub const FT_OK: FT_STATUS = 0;
pub const FT_INVALID_HANDLE: FT_STATUS = 1;
pub const FT_DEVICE_NOT_FOUND: FT_STATUS = 2;
pub const FT_DEVICE_NOT_OPENED: FT_STATUS = 3;
pub const FT_IO_ERROR: FT_STATUS = 4;
pub const FT_INSUFFICIENT_RESOURCES: FT_STATUS = 5;
pub const FT_INVALID_PARAMETER: FT_STATUS = 6;
pub const FT_INVALID_BAUD_RATE: FT_STATUS = 7;
pub const FT_DEVICE_NOT_OPENED_FOR_ERASE: FT_STATUS = 8;
pub const FT_DEVICE_NOT_OPENED_FOR_WRITE: FT_STATUS = 9;
pub const FT_FAILED_TO_WRITE_DEVICE: FT_STATUS = 10;
pub const FT_EEPROM_READ_FAILED: FT_STATUS = 11;
pub const FT_EEPROM_WRITE_FAILED: FT_STATUS = 12;
pub const FT_EEPROM_ERASE_FAILED: FT_STATUS = 13;
pub const FT_EEPROM_NOT_PRESENT: FT_STATUS = 14;
pub const FT_EEPROM_NOT_PROGRAMMED: FT_STATUS = 15;
pub const FT_INVALID_ARGS: FT_STATUS = 16;
pub const FT_NOT_SUPPORTED: FT_STATUS = 17;
pub const FT_OTHER_ERROR: FT_STATUS = 18;

// I2C Transfer Options (from libmpsse_i2c.h)
pub const I2C_TRANSFER_OPTIONS_START_BIT: DWORD = 0x00000001;
pub const I2C_TRANSFER_OPTIONS_STOP_BIT: DWORD = 0x00000002;
pub const I2C_TRANSFER_OPTIONS_BREAK_ON_NACK: DWORD = 0x00000004;
pub const I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE: DWORD = 0x00000008;
pub const I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES: DWORD = 0x00000010;
pub const I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BITS: DWORD = 0x00000020;
pub const I2C_TRANSFER_OPTIONS_NO_ADDRESS: DWORD = 0x00000040;

// I2C Clock Rates (from libmpsse_i2c.h)
pub const I2C_CLOCK_STANDARD_MODE: DWORD = 100000;    // 100 kHz
pub const I2C_CLOCK_FAST_MODE: DWORD = 400000;         // 400 kHz
pub const I2C_CLOCK_FAST_MODE_PLUS: DWORD = 1000000;   // 1 MHz
pub const I2C_CLOCK_HIGH_SPEED_MODE: DWORD = 3400000;  // 3.4 MHz

// FT_DEVICE_LIST_INFO_NODE structure (from ftd2xx.h)
#[repr(C)]
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct FT_DEVICE_LIST_INFO_NODE {
    pub Flags: DWORD,
    pub Type: DWORD,
    pub ID: DWORD,
    pub LocId: DWORD,
    pub SerialNumber: [u8; 16],
    pub Description: [u8; 64],
    pub ftHandle: FT_HANDLE,
}

// ChannelConfig structure (from libmpsse_i2c.h)
#[repr(C)]
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct ChannelConfig {
    pub ClockRate: DWORD,
    pub LatencyTimer: UCHAR,
    pub Options: DWORD,
    pub Pin: DWORD,
    pub currentPinState: USHORT,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            ClockRate: I2C_CLOCK_FAST_MODE,
            LatencyTimer: 8,
            Options: 0,
            Pin: 0,
            currentPinState: 0,
        }
    }
}

// External function declarations from libmpsse.dll
#[link(name = "libmpsse")]
extern "C" {
    /// Initialize libMPSSE library
    pub fn Init_libMPSSE();

    /// Cleanup libMPSSE library
    pub fn Cleanup_libMPSSE();

    /// Get the number of I2C channels available
    pub fn I2C_GetNumChannels(numChannels: *mut DWORD) -> FT_STATUS;

    /// Get information about a specific channel
    pub fn I2C_GetChannelInfo(
        index: DWORD,
        chanInfo: *mut FT_DEVICE_LIST_INFO_NODE,
    ) -> FT_STATUS;

    /// Open an I2C channel
    pub fn I2C_OpenChannel(index: DWORD, handle: *mut FT_HANDLE) -> FT_STATUS;

    /// Initialize an I2C channel with configuration
    pub fn I2C_InitChannel(handle: FT_HANDLE, config: *mut ChannelConfig) -> FT_STATUS;

    /// Close an I2C channel
    pub fn I2C_CloseChannel(handle: FT_HANDLE) -> FT_STATUS;

    /// Read data from an I2C device
    pub fn I2C_DeviceRead(
        handle: FT_HANDLE,
        deviceAddress: UCHAR,
        sizeToTransfer: DWORD,
        buffer: *mut UCHAR,
        sizeTransfered: LPDWORD,
        options: DWORD,
    ) -> FT_STATUS;

    /// Write data to an I2C device
    pub fn I2C_DeviceWrite(
        handle: FT_HANDLE,
        deviceAddress: UCHAR,
        sizeToTransfer: DWORD,
        buffer: *const UCHAR,
        sizeTransfered: LPDWORD,
        options: DWORD,
    ) -> FT_STATUS;
}

/// Helper function to convert FT_STATUS to a string description
pub fn status_to_string(status: FT_STATUS) -> &'static str {
    match status {
        FT_OK => "FT_OK",
        FT_INVALID_HANDLE => "FT_INVALID_HANDLE",
        FT_DEVICE_NOT_FOUND => "FT_DEVICE_NOT_FOUND",
        FT_DEVICE_NOT_OPENED => "FT_DEVICE_NOT_OPENED",
        FT_IO_ERROR => "FT_IO_ERROR",
        FT_INSUFFICIENT_RESOURCES => "FT_INSUFFICIENT_RESOURCES",
        FT_INVALID_PARAMETER => "FT_INVALID_PARAMETER",
        FT_INVALID_BAUD_RATE => "FT_INVALID_BAUD_RATE",
        FT_DEVICE_NOT_OPENED_FOR_ERASE => "FT_DEVICE_NOT_OPENED_FOR_ERASE",
        FT_DEVICE_NOT_OPENED_FOR_WRITE => "FT_DEVICE_NOT_OPENED_FOR_WRITE",
        FT_FAILED_TO_WRITE_DEVICE => "FT_FAILED_TO_WRITE_DEVICE",
        FT_EEPROM_READ_FAILED => "FT_EEPROM_READ_FAILED",
        FT_EEPROM_WRITE_FAILED => "FT_EEPROM_WRITE_FAILED",
        FT_EEPROM_ERASE_FAILED => "FT_EEPROM_ERASE_FAILED",
        FT_EEPROM_NOT_PRESENT => "FT_EEPROM_NOT_PRESENT",
        FT_EEPROM_NOT_PROGRAMMED => "FT_EEPROM_NOT_PROGRAMMED",
        FT_INVALID_ARGS => "FT_INVALID_ARGS",
        FT_NOT_SUPPORTED => "FT_NOT_SUPPORTED",
        FT_OTHER_ERROR => "FT_OTHER_ERROR",
        _ => "UNKNOWN_ERROR",
    }
}
