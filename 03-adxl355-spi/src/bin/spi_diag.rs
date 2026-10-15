//! SPI transport diagnostic: raw libMPSSE split Write+Read with exact lengths,
//! checking register values and that no stray bytes stay in the FTDI RX queue.
//!
//! Never uses SPI_ReadWrite: this libMPSSE build answers it with the MPSSE
//! bad-command reply (0xFA 0x00), which shifts every later read by 2 bytes.

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
}

#[link(name = "ftd2xx")]
extern "system" {
    fn FT_GetQueueStatus(handle: FT_HANDLE, rx_bytes: *mut DWORD) -> DWORD;
}

fn rx_queue(handle: FT_HANDLE) -> DWORD {
    let mut n: DWORD = 0;
    unsafe { FT_GetQueueStatus(handle, &mut n) };
    n
}

/// Split read: command byte (CS stays low), then exactly `n` bytes (CS released)
fn read(handle: FT_HANDLE, reg: u8, n: usize) -> Vec<u8> {
    let mut cmd = [(reg << 1) | 0x01];
    let mut data = vec![0u8; n];
    let mut xfer: DWORD = 0;
    unsafe {
        SPI_Write(handle, cmd.as_mut_ptr(), 1, &mut xfer, BYTES | CS_EN);
        SPI_Read(handle, data.as_mut_ptr(), n as DWORD, &mut xfer, BYTES | CS_DIS);
    }
    data
}

fn main() {
    println!("ADXL355 SPI Transport Diagnostic");
    println!("=================================\n");

    let mut num: DWORD = 0;
    unsafe { SPI_GetNumChannels(&mut num) };
    println!("Channels: {}", num);
    if num == 0 {
        eprintln!("No FT232H found (or another program has it open)");
        std::process::exit(1);
    }

    let mut handle: FT_HANDLE = ptr::null_mut();
    let mut config = ChannelConfig {
        clock_rate: 1_000_000,
        latency_timer: 1,
        config_options: 0x00 | 0x00 | 0x20, // MODE0, CS_DBUS3, CS_ACTIVELOW
        pin: 0,
        current_pin_state: 0,
    };
    let open = unsafe { SPI_OpenChannel(0, &mut handle) };
    let init = unsafe { SPI_InitChannel(handle, &mut config) };
    println!("Open status: {}, Init status: {}", open, init);
    let mut ok = open == 0 && init == 0;

    let queue = rx_queue(handle);
    println!("RX queue after init: {} bytes (expect 0)\n", queue);
    ok &= queue == 0;

    for (name, addr, expected) in [
        ("DEVID_AD", 0x00u8, 0xADu8),
        ("DEVID_MST", 0x01, 0x1D),
        ("PARTID", 0x02, 0xED),
        ("RANGE", 0x2C, 0x81),
    ] {
        let value = read(handle, addr, 1)[0];
        let queue = rx_queue(handle);
        let pass = value == expected && queue == 0;
        ok &= pass;
        println!("{:<10} cmd=0x{:02X}: 0x{:02X} (expect 0x{:02X})  RX queue {}  {}",
            name, (addr << 1) | 1, value, expected, queue, if pass { "✓" } else { "✗" });
    }

    let burst = read(handle, 0x00, 3);
    let queue = rx_queue(handle);
    let pass = burst == [0xAD, 0x1D, 0xED] && queue == 0;
    ok &= pass;
    println!("Burst 0x00-0x02: {:02X?} (expect [AD, 1D, ED])  RX queue {}  {}",
        burst, queue, if pass { "✓" } else { "✗" });

    println!("\nRANGE reads 0x81 only with the power-on range (±2g).");
    println!("{}", if ok { "PASS" } else { "FAIL" });

    unsafe { SPI_CloseChannel(handle) };
}
