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
///
/// # Arguments
/// * `value` - The value to display
/// * `max_value` - Maximum absolute value (defines scale)
/// * `width` - Total width of the bar in characters
///
/// # Example
/// ```
/// use ft232_sensor_interface::create_bar;
///
/// // Display +1.5g on a ±2g scale with 40-char width
/// let bar = create_bar(1.5, 2.0, 40);
/// println!("[{}]", bar);
/// ```
pub fn create_bar(value: f32, max_value: f32, width: usize) -> String {
    let normalized = (value / max_value).clamp(-1.0, 1.0);
    let center = width / 2;
    let bar_length = ((normalized.abs() * center as f32) as usize).min(center);

    let mut bar = String::new();

    if normalized < 0.0 {
        // Negative value: bar extends left from center
        bar.push_str(&" ".repeat(center - bar_length));
        bar.push_str(&"█".repeat(bar_length));
        bar.push('|');
        bar.push_str(&" ".repeat(center));
    } else {
        // Positive value: bar extends right from center
        bar.push_str(&" ".repeat(center));
        bar.push('|');
        bar.push_str(&"█".repeat(bar_length));
        bar.push_str(&" ".repeat(center - bar_length));
    }

    bar
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_bar_zero() {
        let bar = create_bar(0.0, 2.0, 40);
        assert_eq!(bar.chars().count(), 41); // 40 chars + 1 center marker
        assert!(bar.contains('|'));
    }

    #[test]
    fn test_create_bar_positive() {
        let bar = create_bar(1.0, 2.0, 40);
        assert_eq!(bar.chars().count(), 41);
        assert!(bar.contains('█'));
    }

    #[test]
    fn test_create_bar_negative() {
        let bar = create_bar(-1.0, 2.0, 40);
        assert_eq!(bar.chars().count(), 41);
        assert!(bar.contains('█'));
    }

    #[test]
    fn test_timekeeper() {
        let keeper = TimeKeeper::new();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let elapsed = keeper.elapsed_secs();
        assert!(elapsed >= 0.01); // At least 10ms
        assert!(elapsed < 0.1);   // Less than 100ms
    }
}
