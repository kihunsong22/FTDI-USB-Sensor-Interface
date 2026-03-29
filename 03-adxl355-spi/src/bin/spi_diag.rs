//! SPI diagnostic - test split write+read vs full-duplex ReadWrite

use std::ptr;

#[allow(non_camel_case_types)]
type DWORD = u32;
type UCHAR = u8;
type USHORT = u16;
type LPDWORD = *mut DWORD;
#[allow(non_camel_case_types)]
type FT_HANDLE = *mut std::ffi::c_void;

#[repr(C)]
struct ChannelConfig {
    clock_rate: DWORD,
    latency_timer: UCHAR,
    config_options: DWORD,
    pin: DWORD,
    current_pin_state: USHORT,
}

const CS_EN: DWORD = 0x02;
const CS_DIS: DWORD = 0x04;
const BYTES: DWORD = 0x00;

#[link(name = "libmpsse")]
extern "C" {
    fn SPI_GetNumChannels(n: *mut DWORD) -> DWORD;
    fn SPI_OpenChannel(index: DWORD, handle: *mut FT_HANDLE) -> DWORD;
    fn SPI_InitChannel(handle: FT_HANDLE, config: *mut ChannelConfig) -> DWORD;
    fn SPI_CloseChannel(handle: FT_HANDLE) -> DWORD;
    fn SPI_Write(handle: FT_HANDLE, buf: *mut UCHAR, size: DWORD, xfer: LPDWORD, opts: DWORD) -> DWORD;
    fn SPI_Read(handle: FT_HANDLE, buf: *mut UCHAR, size: DWORD, xfer: LPDWORD, opts: DWORD) -> DWORD;
    fn SPI_ReadWrite(handle: FT_HANDLE, in_buf: *mut UCHAR, out_buf: *mut UCHAR, size: DWORD, xfer: LPDWORD, opts: DWORD) -> DWORD;
}

fn open_spi() -> FT_HANDLE {
    let mut num: DWORD = 0;
    unsafe { SPI_GetNumChannels(&mut num) };
    let mut handle: FT_HANDLE = ptr::null_mut();
    unsafe { SPI_OpenChannel(0, &mut handle) };
    let mut config = ChannelConfig {
        clock_rate: 1_000_000,
        latency_timer: 1,
        config_options: 0x00 | 0x00 | 0x20, // MODE0, CS_DBUS3, CS_ACTIVELOW
        pin: 0,
        current_pin_state: 0,
    };
    unsafe { SPI_InitChannel(handle, &mut config) };
    handle
}

fn main() {
    println!("ADXL355 SPI Transport Diagnostic");
    println!("=================================\n");

    let handle = open_spi();

    // Test both command formats × both transport methods
    let formats: [(&str, fn(u8) -> u8); 2] = [
        ("(addr<<1)|0x01", |a| (a << 1) | 0x01),
        ("addr|0x80",      |a| a | 0x80),
    ];

    let reg = 0x00u8; // DEVID_AD, expect 0xAD

    for (fmt_name, make_cmd) in &formats {
        let cmd = make_cmd(reg);

        // Method 1: SPI_ReadWrite (full duplex, single transaction)
        {
            let mut out = [cmd, 0x00, 0x00];
            let mut inp = [0u8; 3];
            let mut xfer: DWORD = 0;
            unsafe { SPI_ReadWrite(handle, inp.as_mut_ptr(), out.as_mut_ptr(), 3, &mut xfer, BYTES | CS_EN | CS_DIS) };
            println!("ReadWrite  cmd=0x{:02X} ({}):  RX = {:02X} {:02X} {:02X}",
                cmd, fmt_name, inp[0], inp[1], inp[2]);
        }

        // Method 2: SPI_Write cmd (CS enable, no disable) + SPI_Read data (no enable, CS disable)
        {
            let mut cmd_buf = [cmd];
            let mut xfer: DWORD = 0;
            unsafe { SPI_Write(handle, cmd_buf.as_mut_ptr(), 1, &mut xfer, BYTES | CS_EN) };

            let mut data = [0u8; 2];
            xfer = 0;
            unsafe { SPI_Read(handle, data.as_mut_ptr(), 2, &mut xfer, BYTES | CS_DIS) };
            println!("Write+Read cmd=0x{:02X} ({}):  RX = {:02X} {:02X}",
                cmd, fmt_name, data[0], data[1]);
        }

        println!();
    }

    // Method 3: Write+Read with 1-byte cmd then 1-byte read (most conservative)
    for (name, addr) in [("DEVID_AD", 0x00u8), ("DEVID_MST", 0x01), ("PARTID", 0x02), ("RANGE", 0x2C)] {
        for (fmt_name, make_cmd) in &formats {
            let cmd = make_cmd(addr);
            let mut cmd_buf = [cmd];
            let mut xfer: DWORD = 0;
            unsafe { SPI_Write(handle, cmd_buf.as_mut_ptr(), 1, &mut xfer, BYTES | CS_EN) };

            let mut data = [0u8; 1];
            xfer = 0;
            unsafe { SPI_Read(handle, data.as_mut_ptr(), 1, &mut xfer, BYTES | CS_DIS) };
            let mark = if data[0] == expected(addr) { " ✓" } else { "" };
            println!("{:<12} cmd=0x{:02X} ({:<14}): 0x{:02X}{}", name, cmd, fmt_name, data[0], mark);
        }
    }

    println!("\nExpected: DEVID_AD=0xAD, DEVID_MST=0x1D, PARTID=0xED, RANGE=0x81");

    unsafe { SPI_CloseChannel(handle) };
}

fn expected(addr: u8) -> u8 {
    match addr { 0x00 => 0xAD, 0x01 => 0x1D, 0x02 => 0xED, 0x2C => 0x81, _ => 0x00 }
}
