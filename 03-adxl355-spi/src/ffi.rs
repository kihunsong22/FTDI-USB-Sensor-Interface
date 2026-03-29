//! FFI bindings for FTDI libMPSSE SPI library
//!
//! This module provides low-level bindings to the libMPSSE SPI DLL.
//! Based on libmpsse_spi.h from FTDI's libMPSSE library.

#![allow(non_camel_case_types)]
#![allow(dead_code)]

use std::ffi::c_void;

// Windows types
pub type DWORD = u32;
pub type UCHAR = u8;
pub type USHORT = u16;
pub type LPDWORD = *mut DWORD;
pub type BOOL = i32;

// FTDI types
pub type FT_STATUS = DWORD;
pub type FT_HANDLE = *mut c_void;

// FT_STATUS return codes
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

// SPI Transfer Options
pub const SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES: DWORD = 0x00000000;
pub const SPI_TRANSFER_OPTIONS_SIZE_IN_BITS: DWORD = 0x00000001;
pub const SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE: DWORD = 0x00000002;
pub const SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE: DWORD = 0x00000004;

// SPI Config Options
pub const SPI_CONFIG_OPTION_MODE0: DWORD = 0x00000000; // CPOL=0, CPHA=0
pub const SPI_CONFIG_OPTION_CS_DBUS3: DWORD = 0x00000000;
pub const SPI_CONFIG_OPTION_CS_ACTIVELOW: DWORD = 0x00000020;

// FT_DEVICE_LIST_INFO_NODE structure
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

// SPI ChannelConfig structure
#[repr(C)]
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct ChannelConfig {
    pub ClockRate: DWORD,
    pub LatencyTimer: UCHAR,
    pub configOptions: DWORD,
    pub Pin: DWORD,
    pub currentPinState: USHORT,
}

// External function declarations from libmpsse.dll (SPI)
#[link(name = "libmpsse")]
extern "C" {
    pub fn Init_libMPSSE();
    pub fn Cleanup_libMPSSE();
    pub fn SPI_GetNumChannels(numChannels: *mut DWORD) -> FT_STATUS;
    pub fn SPI_OpenChannel(index: DWORD, handle: *mut FT_HANDLE) -> FT_STATUS;
    pub fn SPI_InitChannel(handle: FT_HANDLE, config: *mut ChannelConfig) -> FT_STATUS;
    pub fn SPI_CloseChannel(handle: FT_HANDLE) -> FT_STATUS;

    pub fn SPI_ReadWrite(
        handle: FT_HANDLE,
        inBuffer: *mut UCHAR,
        outBuffer: *mut UCHAR,
        sizeToTransfer: DWORD,
        sizeTransferred: LPDWORD,
        transferOptions: DWORD,
    ) -> FT_STATUS;

    pub fn SPI_Write(
        handle: FT_HANDLE,
        buffer: *mut UCHAR,
        sizeToTransfer: DWORD,
        sizeTransfered: LPDWORD,
        options: DWORD,
    ) -> FT_STATUS;

    pub fn SPI_Read(
        handle: FT_HANDLE,
        buffer: *mut UCHAR,
        sizeToTransfer: DWORD,
        sizeTransferred: LPDWORD,
        options: DWORD,
    ) -> FT_STATUS;
}

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
