# Installation Guide

## Prerequisites

- Rust toolchain (edition 2021+)
- FTDI D2XX drivers
- HDF5 C Library 1.14.x (**NOT 2.0** - Rust bindings don't support it yet)
- FT232H hardware + MPU6050 sensor

---

## HDF5 C Library Installation

**Required** for all three programs (collector, visualizer, analyzer).

### Windows

1. Download from https://www.hdfgroup.org/downloads/hdf5/
2. Install to: `C:\Program Files\HDF_Group\HDF5\1.14.6\`
3. The project's `.cargo/config.toml` already sets `HDF5_DIR` for builds.

4. Add to system PATH (for h5dump CLI tool):
```
C:\Program Files\HDF_Group\HDF5\1.14.6\bin
```

Verify:
```powershell
h5dump --version
```

### Linux

**Debian/Ubuntu**:
```bash
sudo apt install libhdf5-dev pkg-config
```

**Fedora/RHEL**:
```bash
sudo dnf install hdf5-devel pkgconfig
```

### macOS

```bash
brew install hdf5
```

If needed:
```bash
export HDF5_DIR=$(brew --prefix hdf5)
```

---

## FTDI Drivers

### Windows
- Download from https://ftdichip.com/drivers/d2xx-drivers/
- Install setup executable
- DLLs included in project repository

### Linux
```bash
# Debian/Ubuntu
sudo apt install libftdi1-dev

# Add udev rules for non-root access
sudo tee /etc/udev/rules.d/99-ftdi.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="0666"
EOF

sudo udevadm control --reload-rules
```

### macOS
```bash
brew install libftdi
```

---

## Build

### All Programs
```bash
cd 01-mpu6050-interface
cargo build --release --features analysis
```

Creates:
- `target/release/collector[.exe]`
- `target/release/visualizer[.exe]`
- `target/release/analyzer[.exe]`

### Individual Programs
```bash
cargo build --release --bin collector
cargo build --release --bin visualizer
cargo build --release --features analysis --bin analyzer
```

---

## Verification

### Check HDF5
```bash
echo $HDF5_DIR              # Linux/macOS
echo %HDF5_DIR%             # Windows
h5dump --version
```

### Test Hardware
```bash
cargo run --release --bin mpu6050-reader
```

Should show:
```
Initializing FT232H I2C interface...
Sensor initialized successfully!
```

### Test HDF5 Programs
```bash
# Quick 5-second test
cargo run --release --bin collector -- --output test.h5 --mode polling --rate 10 --duration 5
cargo run --release --bin visualizer -- --input test.h5 --replay
cargo run --release --features analysis --bin analyzer -- --input test.h5 --statistics
```

---

## Troubleshooting

### HDF5 Not Found

**Windows**:
The project's `.cargo/config.toml` already sets `HDF5_DIR`. If you need to override:
```powershell
$env:HDF5_DIR = "C:\Program Files\HDF_Group\HDF5\1.14.6"
cargo build --release
```

**Linux**:
```bash
# If pkg-config fails
export HDF5_DIR=/usr/local
```

### FTDI Access Issues

**Windows**: Check Device Manager for "USB Serial Converter"

**Linux**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

---

## Quick Reference

### Build Commands
```bash
# Full build
cargo build --release --features analysis

# Library only (no HDF5)
cargo build --release --lib
```

### Environment Variables

**Windows**:
```
HDF5_DIR = C:\Program Files\HDF_Group\HDF5\1.14.6  (set in .cargo/config.toml)
PATH += C:\Program Files\HDF_Group\HDF5\1.14.6\bin  (optional, for h5dump CLI)
```

**Linux/macOS**:
- Usually auto-detected via pkg-config
- If needed: `export HDF5_DIR=/usr/local`
