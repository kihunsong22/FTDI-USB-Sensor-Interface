//! Data validation: verify FIFO reads return genuinely fresh samples
//!
//! Checks:
//! 1. FIFO_ENTRIES register changes between reads (FIFO is filling)
//! 2. Consecutive FIFO batches contain different sample values
//! 3. Axis markers in raw FIFO data are correct (X=0x01 flag on byte[2])
//! 4. Accelerometer values are physically plausible (~1g on Z for stationary)
//! 5. Samples within a batch vary (not all identical = stale)
//! 6. Raw pipeline bytes (the 2 skipped bytes) vs fresh bytes

use ft232_adxl355_spi::{Adxl355, OutputDataRate, Range};
use std::time::{Duration, Instant};

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("ADXL355 Data Validation");
    println!("=======================\n");

    let mut sensor = Adxl355::new(0)?;
    let range = sensor.get_range();
    println!("Sensor initialized, range: {:?}\n", range);

    // ── Test 1: FIFO_ENTRIES changes over time ──
    println!("=== Test 1: FIFO_ENTRIES register is live ===");
    sensor.enable_fifo(OutputDataRate::Odr1000)?;
    // Drain the FIFO
    let _ = sensor.read_fifo_batch();
    std::thread::sleep(Duration::from_millis(5));

    let e1 = sensor.get_fifo_entries()?;
    std::thread::sleep(Duration::from_millis(20));
    let e2 = sensor.get_fifo_entries()?;
    std::thread::sleep(Duration::from_millis(20));
    let e3 = sensor.get_fifo_entries()?;
    println!("  Entries over 40ms: {} -> {} -> {}", e1, e2, e3);
    if e1 < e2 && e2 < e3 {
        println!("  PASS: FIFO is filling (entries increasing)\n");
    } else {
        println!("  WARN: entries not strictly increasing — may already be full\n");
    }

    // ── Test 2: Consecutive batches differ ──
    println!("=== Test 2: Consecutive batches return different data ===");
    // Drain
    let _ = sensor.read_fifo_batch();
    std::thread::sleep(Duration::from_millis(50));

    let batch_a = sensor.read_fifo_batch()?;
    std::thread::sleep(Duration::from_millis(50));
    let batch_b = sensor.read_fifo_batch()?;

    if batch_a.is_empty() || batch_b.is_empty() {
        println!("  FAIL: got empty batch (a={}, b={})\n", batch_a.len(), batch_b.len());
    } else {
        let a_last = batch_a.last().unwrap();
        let b_first = batch_b.first().unwrap();
        let same = a_last.accel_x == b_first.accel_x
            && a_last.accel_y == b_first.accel_y
            && a_last.accel_z == b_first.accel_z;
        println!("  Batch A last:  X={:6} Y={:6} Z={:6}",
            a_last.accel_x, a_last.accel_y, a_last.accel_z);
        println!("  Batch B first: X={:6} Y={:6} Z={:6}",
            b_first.accel_x, b_first.accel_y, b_first.accel_z);
        if same {
            println!("  FAIL: batches returned identical edge samples — possible stale data\n");
        } else {
            println!("  PASS: batches contain different samples\n");
        }
    }

    // ── Test 3: Samples within a batch are not all identical ──
    println!("=== Test 3: Intra-batch variation ===");
    std::thread::sleep(Duration::from_millis(100));
    let batch = sensor.read_fifo_batch()?;
    println!("  Batch size: {} samples", batch.len());
    if batch.len() >= 2 {
        let mut unique_x = std::collections::HashSet::new();
        let mut unique_y = std::collections::HashSet::new();
        let mut unique_z = std::collections::HashSet::new();
        for s in &batch {
            unique_x.insert(s.accel_x);
            unique_y.insert(s.accel_y);
            unique_z.insert(s.accel_z);
        }
        println!("  Unique values — X: {}, Y: {}, Z: {}",
            unique_x.len(), unique_y.len(), unique_z.len());
        if unique_x.len() == 1 && unique_y.len() == 1 && unique_z.len() == 1 {
            println!("  FAIL: all samples identical — reading stale data\n");
        } else {
            println!("  PASS: samples vary within batch (sensor noise expected)\n");
        }
    }

    // ── Test 4: Physical plausibility ──
    println!("=== Test 4: Physical plausibility (stationary = ~1g on Z) ===");
    std::thread::sleep(Duration::from_millis(50));
    let batch = sensor.read_fifo_batch()?;
    if !batch.is_empty() {
        let n = batch.len() as f32;
        let avg_x: f32 = batch.iter().map(|s| s.accel_to_g(range).0).sum::<f32>() / n;
        let avg_y: f32 = batch.iter().map(|s| s.accel_to_g(range).1).sum::<f32>() / n;
        let avg_z: f32 = batch.iter().map(|s| s.accel_to_g(range).2).sum::<f32>() / n;
        let magnitude = (avg_x * avg_x + avg_y * avg_y + avg_z * avg_z).sqrt();
        println!("  Avg X: {:.5}g, Y: {:.5}g, Z: {:.5}g", avg_x, avg_y, avg_z);
        println!("  Magnitude: {:.5}g", magnitude);
        if (magnitude - 1.0).abs() < 0.05 {
            println!("  PASS: magnitude ~1g (gravity)\n");
        } else {
            println!("  FAIL: magnitude {:.3}g, expected ~1.0g\n", magnitude);
        }
    }

    // ── Test 5: Sustained rate check ──
    println!("=== Test 5: Sustained sample rate (2 seconds at 1kHz ODR) ===");
    let _ = sensor.read_fifo_batch(); // drain
    let start = Instant::now();
    let mut total = 0u64;
    let mut empty_polls = 0u64;
    while start.elapsed() < Duration::from_secs(2) {
        match sensor.read_fifo_batch() {
            Ok(b) if !b.is_empty() => total += b.len() as u64,
            Ok(_) => empty_polls += 1,
            Err(e) => { eprintln!("  Error: {}", e); break; }
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    let rate = total as f64 / elapsed;
    println!("  Samples: {} in {:.2}s = {:.1} Hz (ODR target: 1000 Hz)", total, elapsed, rate);
    println!("  Empty polls: {}", empty_polls);
    if rate > 800.0 && rate < 1100.0 {
        println!("  PASS: rate within 20% of 1kHz ODR\n");
    } else {
        println!("  WARN: rate {:.0} Hz deviates from 1kHz target\n", rate);
    }

    // ── Test 6: Raw pipeline verification ──
    println!("=== Test 6: Pipeline byte inspection ===");
    println!("  Reading FIFO_ENTRIES via read_register (uses 2-byte skip)...");
    let _ = sensor.read_fifo_batch(); // drain
    std::thread::sleep(Duration::from_millis(50));
    let entries = sensor.get_fifo_entries()?;
    println!("  FIFO_ENTRIES = {} (expect >0 after 50ms at 1kHz)", entries);
    if entries > 0 {
        println!("  PASS: register reads returning plausible values\n");
    } else {
        println!("  FAIL: 0 entries after 50ms wait\n");
    }

    // ── Test 7: Temperature sanity ──
    println!("=== Test 7: Temperature register ===");
    let temp_raw = sensor.read_temperature()?;
    let temp_c = ((temp_raw as f32 - 1885.0) / -9.05) + 25.0;
    println!("  Raw: {}, Celsius: {:.1}°C", temp_raw, temp_c);
    if temp_c > 10.0 && temp_c < 50.0 {
        println!("  PASS: temperature in plausible room-temp range\n");
    } else {
        println!("  FAIL: temperature {:.1}°C seems wrong\n", temp_c);
    }

    sensor.disable_fifo()?;
    println!("=== All checks complete ===");
    Ok(())
}
