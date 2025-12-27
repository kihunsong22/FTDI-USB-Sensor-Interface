use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Get the current directory (project root for this crate)
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let project_root = PathBuf::from(&manifest_dir);

    // Navigate to parent directory where shared FTDI libraries are located
    let shared_root = project_root.parent().unwrap();

    // Path to MPSSE DLL (Win32/32-bit)
    let mpsse_lib_path = shared_root.join("FTDI MPSSE").join("build").join("Win32").join("DLL");

    // Path to D2XX DLL
    let d2xx_lib_path = shared_root.join("FTDI-D2XX-Drivers-Win-2.12.36.20U").join("x86");

    // Tell cargo where to find the libraries
    println!("cargo:rustc-link-search=native={}", mpsse_lib_path.display());
    println!("cargo:rustc-link-search=native={}", d2xx_lib_path.display());

    // Link against the libraries
    // Note: libmpsse.dll depends on FTD2XX.dll, which will be loaded at runtime
    println!("cargo:rustc-link-lib=dylib=libmpsse");

    // Rerun if the DLL paths change
    println!("cargo:rerun-if-changed=../FTDI MPSSE/build/Win32/DLL/libmpsse.dll");
    println!("cargo:rerun-if-changed=../FTDI-D2XX-Drivers-Win-2.12.36.20U/x86/FTD2XX.dll");

    // Auto-copy runtime DLLs to output directory
    // This ensures the DLLs are available when running the executable
    if let Ok(profile) = env::var("PROFILE") {
        let target_dir = project_root
            .join("target")
            .join("i686-pc-windows-msvc")
            .join(&profile);

        // Only copy if target directory exists (i.e., during actual build, not just script check)
        if target_dir.exists() {
            let mpsse_dll_src = mpsse_lib_path.join("libmpsse.dll");
            let d2xx_dll_src = d2xx_lib_path.join("FTD2XX.dll");

            let mpsse_dll_dst = target_dir.join("libmpsse.dll");
            let d2xx_dll_dst = target_dir.join("FTD2XX.dll");

            // Copy DLLs, ignore errors if files don't exist yet
            let _ = fs::copy(&mpsse_dll_src, &mpsse_dll_dst);
            let _ = fs::copy(&d2xx_dll_src, &d2xx_dll_dst);

            println!("cargo:warning=Copied runtime DLLs to {}", target_dir.display());
        }
    }
}
