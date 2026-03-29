//! Minimal raw SPI test - no library dependency, just FFI

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
    fn SPI_ReadWrite(h: Handle, i: *mut UCHAR, o: *mut UCHAR, s: DWORD, x: LPDWORD, opt: DWORD) -> DWORD;
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

    // Read DEVID_AD with cmd=0x80
    let mut out = [0x80u8, 0x00, 0x00];
    let mut inp = [0u8; 3];
    let mut xfer: DWORD = 0;
    let st = unsafe { SPI_ReadWrite(h, inp.as_mut_ptr(), out.as_mut_ptr(), 3, &mut xfer, 0x06) };
    println!("ReadWrite status={} xfer={}", st, xfer);
    println!("RX: {:02X} {:02X} {:02X}", inp[0], inp[1], inp[2]);

    if inp[0] == 0xAD { println!("DEVID_AD = 0xAD ✓") }
    else { println!("DEVID_AD = 0x{:02X} ✗ (expected 0xAD)", inp[0]) }

    unsafe { SPI_CloseChannel(h) };
}
