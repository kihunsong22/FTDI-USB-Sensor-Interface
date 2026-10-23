//! Common utilities shared across programs

use std::time::Instant;

/// Tracks elapsed time since creation
pub struct TimeKeeper {
    start: Instant,
}

impl TimeKeeper {
    /// Create a new TimeKeeper starting now
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }

    /// Get elapsed time in seconds
    pub fn elapsed_secs(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }
}

impl Default for TimeKeeper {
    fn default() -> Self {
        Self::new()
    }
}

/// Keep FIFO polling on time on laptops running on battery: opts the process out
/// of Windows power throttling (EcoQoS) and raises the calling thread's priority.
/// Call it from the thread that polls the sensor. Failures are ignored.
#[cfg(windows)]
pub fn prioritize_polling() {
    use std::ffi::c_void;

    #[repr(C)]
    struct PowerThrottlingState {
        version: u32,
        control_mask: u32,
        state_mask: u32,
    }

    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentProcess() -> *mut c_void;
        fn GetCurrentThread() -> *mut c_void;
        fn SetProcessInformation(process: *mut c_void, class: i32, info: *const c_void, size: u32) -> i32;
        fn SetThreadPriority(thread: *mut c_void, priority: i32) -> i32;
    }

    const PROCESS_POWER_THROTTLING: i32 = 4;
    const EXECUTION_SPEED: u32 = 0x1;
    const IGNORE_TIMER_RESOLUTION: u32 = 0x4;
    const THREAD_PRIORITY_HIGHEST: i32 = 2;

    // Control bits set with state bits clear = throttling explicitly off
    let state = PowerThrottlingState {
        version: 1,
        control_mask: EXECUTION_SPEED | IGNORE_TIMER_RESOLUTION,
        state_mask: 0,
    };
    unsafe {
        SetProcessInformation(
            GetCurrentProcess(),
            PROCESS_POWER_THROTTLING,
            &state as *const PowerThrottlingState as *const c_void,
            std::mem::size_of::<PowerThrottlingState>() as u32,
        );
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    }
}

#[cfg(not(windows))]
pub fn prioritize_polling() {}

/// Create a horizontal bar graph for a value
pub fn create_bar(value: f32, max_value: f32, width: usize) -> String {
    let normalized = (value / max_value).clamp(-1.0, 1.0);
    let center = width / 2;
    let bar_length = ((normalized.abs() * center as f32) as usize).min(center);

    let mut bar = String::new();

    if normalized < 0.0 {
        bar.push_str(&" ".repeat(center - bar_length));
        bar.push_str(&"█".repeat(bar_length));
        bar.push('|');
        bar.push_str(&" ".repeat(center));
    } else {
        bar.push_str(&" ".repeat(center));
        bar.push('|');
        bar.push_str(&"█".repeat(bar_length));
        bar.push_str(&" ".repeat(center - bar_length));
    }

    bar
}
