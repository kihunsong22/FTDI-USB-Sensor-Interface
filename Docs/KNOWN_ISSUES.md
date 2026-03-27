# Known Integration Issues

## HDF5 2.0 is not supported

*Last verified: 2026-03-23*

The Rust `hdf5` crate does not support HDF5 2.0. Use **1.14.x only**.

The crates.io release (0.8.1, Nov 2021) doesn't support 1.14.x either — that support only exists in unreleased commits on master. This is why `Cargo.toml` uses a git dependency:

```toml
hdf5 = { git = "https://github.com/aldanor/hdf5-rust.git" }
```

A fork ([hdf5-metno](https://github.com/metno/hdf5-rust)) publishes newer versions to crates.io as `hdf5-metno` (v0.12.3), which may be an alternative if the git dependency becomes problematic.

If you see linker errors or runtime crashes after upgrading HDF5, check that you haven't installed 2.0.
