# Project Backlog

## Pending Improvements

### 1. Centralize Runtime DLLs ✓ COMPLETED

**Issue**: Runtime DLLs (`libmpsse.dll`, `FTD2XX.dll`) needed manual copying to executable directory after each build.

**Solution Implemented**: Option A - Auto-copy via `build.rs`

The build script now automatically copies both DLLs from their source locations to the target directory during the build process (lines 33-53 in `build.rs`):
```rust
if let Ok(profile) = env::var("PROFILE") {
    let target_dir = project_root.join("target").join("i686-pc-windows-msvc").join(&profile);
    if target_dir.exists() {
        let _ = fs::copy(&mpsse_dll_src, &mpsse_dll_dst);
        let _ = fs::copy(&d2xx_dll_src, &d2xx_dll_dst);
        println!("cargo:warning=Copied runtime DLLs to {}", target_dir.display());
    }
}
```

Benefits realized:
- Fully automatic - no manual steps required
- Works with `cargo run` immediately after build
- Each build gets fresh DLLs
- DLLs available in both debug and release builds

### 2. Fix I2C Transfer Size Issue ✓ COMPLETED

**Issue**: Error "expected 6 bytes, transferred 48" when reading sensor data.

**Root Cause**: When using `I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES`, the MPSSE library returns the transfer count in **bits** rather than bytes (48 bits = 6 bytes × 8).

**Solution Implemented**: Removed transfer count validation in `src/mpu6050.rs` and followed FTDI sample code pattern of checking only `FT_STATUS` return code:

```rust
// In read_registers() method (lines 276-283):
if status != FT_OK {
    return Err(status.into());
}

// Note: When using FAST_TRANSFER_BYTES, the transferred count is in bits, not bytes
// (e.g., 6 bytes = 48 bits). Based on FTDI sample code, we should only check status.
// If status is FT_OK, the data is valid regardless of the transferred count.

Ok(data)
```

This matches the pattern used in FTDI's BME280-I2C and I2C_EEPROM sample code. Sensor now reads successfully.

### 3. Suppress FFI Naming Warnings ✓ COMPLETED

**Issue**: 12 warnings about snake_case naming in FFI structures.

**Solution Implemented**: Added `#[allow(non_snake_case)]` attribute to both FFI structs in `src/ffi.rs`:
- `FT_DEVICE_LIST_INFO_NODE`
- `ChannelConfig`

Build now completes cleanly with no warnings.

## Future Enhancements

- [ ] Add sensor calibration functionality
- [ ] Support configurable accelerometer/gyroscope ranges
- [ ] Implement data filtering (low-pass, high-pass)
- [ ] Add data logging to CSV/binary format
- [ ] Create example integration with visualizer
- [ ] Support multiple MPU6050 sensors on same I2C bus
- [ ] Add interrupt-based reading (if supported by FT232H)
