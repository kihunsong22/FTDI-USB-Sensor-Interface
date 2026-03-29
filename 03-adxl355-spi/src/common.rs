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
