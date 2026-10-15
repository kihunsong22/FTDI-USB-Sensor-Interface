//! Minimal raw SPI test - no library dependency, just FFI.
//! Reads DEVID_AD (expect 0xAD): command (addr << 1) | 1, then exactly 1 byte.

use std::ptr;

type DWORD = u32;
type UCHAR = u8;
type USHORT = u16;
type LPDWORD = *mut DWORD;
type Handle = *mut std::ffi::c_void;

#[repr(C)]
struct Cfg { clk: DWORD, lat: UCHAR, opt: DWORD, pin: DWORD, ps: USHORT }

#[link(name = "libmpsse")]
extern "C" {
    fn SPI_GetNumChannels(n: *mut DWORD) -> DWORD;
    fn SPI_OpenChannel(i: DWORD, h: *mut Handle) -> DWORD;
    fn SPI_InitChannel(h: Handle, c: *mut Cfg) -> DWORD;
    fn SPI_CloseChannel(h: Handle) -> DWORD;
    fn SPI_Write(h: Handle, b: *mut UCHAR, s: DWORD, x: LPDWORD, opt: DWORD) -> DWORD;
    fn SPI_Read(h: Handle, b: *mut UCHAR, s: DWORD, x: LPDWORD, opt: DWORD) -> DWORD;
}

fn main() {
    let mut n: DWORD = 0;
    unsafe { SPI_GetNumChannels(&mut n) };
    println!("Channels: {}", n);

    let mut h: Handle = ptr::null_mut();
    let st = unsafe { SPI_OpenChannel(0, &mut h) };
    println!("Open: {}", st);

    let mut cfg = Cfg { clk: 1_000_000, lat: 1, opt: 0x20, pin: 0, ps: 0 };
    let st = unsafe { SPI_InitChannel(h, &mut cfg) };
    println!("Init: {}", st);

    // Command byte with CS held low (0x02), then 1 byte with CS released (0x04)
    let mut cmd = [0x01u8];
    let mut rx = [0u8; 1];
    let mut xfer: DWORD = 0;
    let st_w = unsafe { SPI_Write(h, cmd.as_mut_ptr(), 1, &mut xfer, 0x02) };
    let st_r = unsafe { SPI_Read(h, rx.as_mut_ptr(), 1, &mut xfer, 0x04) };
    println!("Write status={} Read status={} xfer={}", st_w, st_r, xfer);

    if rx[0] == 0xAD { println!("DEVID_AD = 0xAD ✓") }
    else { println!("DEVID_AD = 0x{:02X} ✗ (expected 0xAD)", rx[0]) }

    unsafe { SPI_CloseChannel(h) };
}
