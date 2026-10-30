//! LSM6DSOX bring-up on an FT232H I2C bus: bus scan, identity, self-test,
//! polled reads and FIFO streaming with a lost-sample check.

mod ffi;
mod lsm6dsox;

use clap::{Parser, Subcommand};
use lsm6dsox::*;
use std::time::Instant;

type AnyResult<T = ()> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Parser)]
#[command(about = "LSM6DSOX on an FT232H I2C bus (libMPSSE)")]
struct Cli {
    /// FT232H channel index
    #[arg(long, global = true, default_value_t = 0)]
    channel: u32,
    /// I2C clock in kHz: 100, 400 or 1000
    #[arg(long, global = true, default_value_t = 400)]
    clock: u32,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// List FT232H channels and the I2C addresses that answer
    Scan,
    /// Pin-level wiring check: are D1 and D2 joined (I2C switch), idle levels of D0..D2
    Pins,
    /// WHO_AM_I, trimmed output data rates, control registers
    Info,
    /// Built-in accelerometer and gyroscope self-test (keep the sensor still)
    Selftest,
    /// Poll the output registers and print values
    Read {
        #[command(flatten)]
        sensor: SensorArgs,
        #[arg(long, default_value_t = 5.0)]
        secs: f64,
    },
    /// Stream accel + gyro through the FIFO and check for lost samples
    Fifo {
        #[command(flatten)]
        sensor: SensorArgs,
        #[arg(long, default_value_t = 3.0)]
        secs: f64,
        /// Accelerometer only (gyroscope powered down, not batched)
        #[arg(long)]
        no_gyro: bool,
    },
}

#[derive(clap::Args)]
struct SensorArgs {
    /// Output data rate in Hz for both sensors, 12.5 to 6667 (default 104 for read, 833 for fifo)
    #[arg(long)]
    odr: Option<f64>,
    /// Accelerometer full scale in g: 2, 4, 8, 16
    #[arg(long, default_value_t = 4)]
    accel_fs: u16,
    /// Gyroscope full scale in dps: 125, 250, 500, 1000, 2000
    #[arg(long, default_value_t = 500)]
    gyro_fs: u16,
}

impl SensorArgs {
    fn settings(&self, default_odr: f64) -> AnyResult<Settings> {
        let hz = self.odr.unwrap_or(default_odr);
        let odr = Odr::from_hz(hz).ok_or_else(|| {
            let list: Vec<String> = Odr::all().map(|o| o.nominal.to_string()).collect();
            format!("--odr {hz}: use one of {}", list.join(", "))
        })?;
        Ok(Settings::new(odr, self.accel_fs, self.gyro_fs)?)
    }
}

fn main() {
    let cli = Cli::parse();
    if let Err(e) = run(&cli) {
        eprintln!("error: {e}");
        if let Some(Error::Ftdi { call: "I2C_OpenChannel", .. }) = e.downcast_ref::<Error>() {
            eprintln!("the FT232H may be open in another program (Motlinelab VMS, another collector)");
        }
        std::process::exit(1);
    }
}

fn run(cli: &Cli) -> AnyResult {
    let clock = match cli.clock {
        100 | 400 | 1000 => cli.clock * 1000,
        c => return Err(format!("--clock {c}: use 100, 400 or 1000").into()),
    };
    let open = || -> AnyResult<Lsm6dsox> {
        let s = Lsm6dsox::open(I2cBus::open(cli.channel, clock)?)?;
        println!("LSM6DSOX at 0x{:02X}, I2C {} kHz", s.address, clock / 1000);
        Ok(s)
    };
    match &cli.cmd {
        Cmd::Scan => scan(cli.channel, clock),
        Cmd::Pins => pins(cli.channel, clock),
        Cmd::Info => info(open()?),
        Cmd::Selftest => selftest(open()?),
        Cmd::Read { sensor, secs } => {
            let settings = sensor.settings(104.0)?;
            read(open()?, settings, *secs)
        }
        Cmd::Fifo { sensor, secs, no_gyro } => {
            let mut settings = sensor.settings(833.0)?;
            settings.gyro_on = !no_gyro;
            fifo(open()?, settings, *secs)
        }
    }
}

fn scan(channel: u32, clock: u32) -> AnyResult {
    let list = channels()?;
    if list.is_empty() {
        return Err(Error::NoChannels.into());
    }
    for (i, c) in list.iter().enumerate() {
        let busy = if c.opened { "  (open in another program)" } else { "" };
        println!("channel {i}: {} serial {}{busy}", c.description, c.serial);
    }
    let mut bus = I2cBus::open(channel, clock)?;
    let mut found = 0;
    for addr in 0x08..=0x77u8 {
        if bus.probe(addr)? {
            found += 1;
            let name = match addr {
                0x6A | 0x6B => "  LSM6DSOX (or other LSM6DS)",
                0x1C | 0x1E => "  LIS3MDL magnetometer",
                0x68 | 0x69 => "  MPU6050",
                _ => "",
            };
            println!("0x{addr:02X} answers{name}");
        }
    }
    if found == 0 {
        println!("no I2C device answered on channel {channel}");
        println!("Adafruit FT232H: is the I2C switch on (joins D1 and D2 as SDA)? STEMMA QT cable seated?");
    }
    Ok(())
}

fn pins(channel: u32, clock: u32) -> AnyResult {
    let (d1_d2, released) = I2cBus::open(channel, clock)?.pin_check()?;
    let bit = |b: u8, pin: u8| (b >> pin) & 1;
    let d2 = d1_d2.map(|b| bit(b, 2));
    println!("D1 driven low, high, low: D2 reads {} {} {}", d2[0], d2[1], d2[2]);
    if d2 == [0, 1, 0] {
        println!("  D1 and D2 are joined (I2C switch on)");
    } else {
        println!("  D2 does not follow D1: not joined, turn the I2C Mode switch on");
    }
    let pulled = [0, 1, 2].map(|pin| bit(released, pin));
    println!("driven low, released 20 ms: D0 {}  D1 {}  D2 {}", pulled[0], pulled[1], pulled[2]);
    if pulled[0] == 1 && (pulled[1] == 1 || pulled[2] == 1) {
        println!("  SCL and SDA have pull-ups (sensor board powered and connected)");
    } else {
        println!("  no pull-up on {}: sensor board not connected or not powered (STEMMA QT cable, Vin)",
            if pulled[0] == 0 { "SCL (D0)" } else { "SDA (D1/D2)" });
    }
    Ok(())
}

fn info(mut s: Lsm6dsox) -> AnyResult {
    println!("WHO_AM_I 0x6C");
    let fine = s.freq_fine()?;
    println!("INTERNAL_FREQ_FINE {fine} ({:+.2} % from nominal)", fine as f64 * 0.15);
    for odr in Odr::all().filter(|o| [104.0, 833.0, 1667.0, 3333.0, 6667.0].contains(&o.nominal)) {
        println!("  ODR {:>6} Hz is {:.3} Hz on this part", odr.nominal, odr.actual(fine));
    }
    let regs = s.control_registers()?;
    let text: Vec<String> = regs.iter().enumerate().map(|(i, r)| format!("CTRL{} {r:02X}", i + 1)).collect();
    println!("after reset: {}", text.join("  "));
    Ok(())
}

fn selftest(mut s: Lsm6dsox) -> AnyResult {
    let (accel, gyro) = s.self_test()?;
    let accel_ok = accel.iter().all(|v| (50.0..=1700.0).contains(v));
    let gyro_ok = gyro.iter().all(|v| (150.0..=700.0).contains(v));
    let verdict = |ok| if ok { "pass" } else { "FAIL" };
    println!(
        "accel self-test change  X {:6.0}  Y {:6.0}  Z {:6.0} mg   limits 50..1700   {}",
        accel[0], accel[1], accel[2], verdict(accel_ok)
    );
    println!(
        "gyro  self-test change  X {:6.0}  Y {:6.0}  Z {:6.0} dps  limits 150..700   {}",
        gyro[0], gyro[1], gyro[2], verdict(gyro_ok)
    );
    if accel_ok && gyro_ok {
        Ok(())
    } else {
        Err("self-test out of limits (keep the sensor still and retry)".into())
    }
}

/// Running mean and standard deviation per axis (Welford).
#[derive(Default)]
struct AxisStats {
    n: u64,
    mean: [f64; 3],
    m2: [f64; 3],
}

impl AxisStats {
    fn add(&mut self, v: [f64; 3]) {
        self.n += 1;
        for i in 0..3 {
            let d = v[i] - self.mean[i];
            self.mean[i] += d / self.n as f64;
            self.m2[i] += d * (v[i] - self.mean[i]);
        }
    }

    fn std(&self) -> [f64; 3] {
        let n = (self.n.max(2) - 1) as f64;
        self.m2.map(|m| (m / n).sqrt())
    }

    fn print(&self, name: &str, unit: &str, noise_unit: &str, noise_scale: f64) {
        let [x, y, z] = self.mean;
        let [sx, sy, sz] = self.std().map(|s| s * noise_scale);
        println!("{name} mean  {x:+.4} {y:+.4} {z:+.4} {unit}   noise (std)  {sx:.2} {sy:.2} {sz:.2} {noise_unit}");
    }
}

fn scaled(v: [i16; 3], per_lsb: f64) -> [f64; 3] {
    v.map(|c| c as f64 * per_lsb)
}

fn magnitude(v: [f64; 3]) -> f64 {
    v.iter().map(|c| c * c).sum::<f64>().sqrt()
}

fn read(mut s: Lsm6dsox, set: Settings, secs: f64) -> AnyResult {
    let fine = s.freq_fine()?;
    s.start(&set)?;
    println!("ODR {} Hz, +-{} g, +-{} dps, polling for {secs} s", set.odr.nominal, set.accel_g, set.gyro_dps);
    let mut accel = AxisStats::default();
    let mut gyro = AxisStats::default();
    let mut temp = 0.0;
    let mut polls = 0u64;
    let (mut settling, mut settling_max) = (0u32, 0.0f64);
    let t0 = Instant::now();
    let mut next_print = 0.5;
    while t0.elapsed().as_secs_f64() < secs {
        polls += 1;
        if s.status()? & (XLDA | GDA) != XLDA | GDA {
            continue;
        }
        let raw = s.outputs()?;
        let a = scaled(raw.accel, set.g_per_lsb());
        let g = scaled(raw.gyro, set.dps_per_lsb());
        let t = t0.elapsed().as_secs_f64();
        if t < SETTLE_SECS {
            settling += 1;
            settling_max = g.iter().fold(settling_max, |m, v| m.max(v.abs()));
            continue;
        }
        accel.add(a);
        gyro.add(g);
        temp = raw.temp_c();
        if t >= next_print {
            next_print += 0.5;
            println!(
                "{t:5.1} s  accel {:+.4} {:+.4} {:+.4} g  |a| {:.4}   gyro {:+7.2} {:+7.2} {:+7.2} dps   {temp:.1} C",
                a[0], a[1], a[2], magnitude(a), g[0], g[1], g[2]
            );
        }
    }
    let el = t0.elapsed().as_secs_f64();
    println!(
        "{} samples in {el:.2} s = {:.1} Hz read, sensor ODR {:.2} Hz, {:.0} status polls/s",
        accel.n,
        accel.n as f64 / el,
        set.odr.actual(fine),
        polls as f64 / el
    );
    println!(
        "left out {settling} samples from the first {:.0} ms (turn-on), largest gyro value {settling_max:.1} dps",
        SETTLE_SECS * 1e3
    );
    accel.print("accel", "g", "mg", 1e3);
    gyro.print("gyro ", "dps", "dps", 1.0);
    println!("|accel mean| {:.4} g, temperature {temp:.1} C", magnitude(accel.mean));
    Ok(())
}

/// Turn-on time is 35 ms (datasheet); samples before this are left out of the statistics.
const SETTLE_SECS: f64 = 0.1;

/// Checks the FIFO time slots: with accel and gyro at one rate, every slot holds one accel
/// word and `gyro_per_slot` gyro words, and TAG_CNT steps by one (mod 4) from slot to slot.
#[derive(Default)]
struct SlotCheck {
    gyro_per_slot: u32,
    current: Option<u8>,
    accel: u32,
    gyro: u32,
    slots: u64,
    errors: u64,
}

impl SlotCheck {
    fn word(&mut self, cnt: u8, is_accel: bool) {
        match self.current {
            Some(c) if c == cnt => {}
            Some(c) => {
                // the first slot may be partial, so it is not judged
                let complete = self.accel == 1 && self.gyro == self.gyro_per_slot;
                if cnt != (c + 1) & 3 || (self.slots > 0 && !complete) {
                    self.errors += 1;
                }
                self.slots += 1;
                self.accel = 0;
                self.gyro = 0;
                self.current = Some(cnt);
            }
            None => self.current = Some(cnt),
        }
        if is_accel {
            self.accel += 1;
        } else {
            self.gyro += 1;
        }
    }
}

/// Least-squares slope of y against x.
#[derive(Default)]
struct LineFit {
    n: f64,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
}

impl LineFit {
    fn add(&mut self, x: f64, y: f64) {
        self.n += 1.0;
        self.sx += x;
        self.sy += y;
        self.sxx += x * x;
        self.sxy += x * y;
    }

    fn slope(&self) -> f64 {
        (self.n * self.sxy - self.sx * self.sy) / (self.n * self.sxx - self.sx * self.sx)
    }
}

const MAX_WORDS: usize = 512;

fn fifo(mut s: Lsm6dsox, set: Settings, secs: f64) -> AnyResult {
    let fine = s.freq_fine()?;
    let rate = set.odr.actual(fine);
    let gyro_text = if set.gyro_on { format!("+-{} dps", set.gyro_dps) } else { "gyro off".into() };
    println!(
        "ODR {} Hz (this part {rate:.2} Hz), +-{} g, {gyro_text}, FIFO continuous for {secs} s",
        set.odr.nominal, set.accel_g
    );
    s.start(&set)?;
    s.fifo_start(&set)?;
    let t0 = Instant::now();

    let words_per_slot = 1 + set.gyro_on as usize;
    let mut buf = vec![0u8; MAX_WORDS * FIFO_WORD];
    let mut accel = AxisStats::default();
    let mut gyro = AxisStats::default();
    let mut slots = SlotCheck { gyro_per_slot: set.gyro_on as u32, ..Default::default() };
    let (mut other, mut overruns, mut reads, mut bytes, mut max_backlog) = (0u64, 0u64, 0u64, 0usize, 0usize);
    let mut rate_fit = LineFit::default();
    let mut last: f64;
    let mut next_print = 1.0;
    loop {
        let (words, flags) = s.fifo_status()?;
        last = t0.elapsed().as_secs_f64();
        if flags & FIFO_OVR_LATCHED != 0 {
            overruns += 1;
        }
        max_backlog = max_backlog.max(words);
        // samples per sensor made so far: read words plus the backlog, one word per sensor per slot
        if last >= 0.5 {
            rate_fit.add(last, (bytes / FIFO_WORD + words) as f64 / words_per_slot as f64);
        }
        if words > 0 {
            let n = words.min(MAX_WORDS) * FIFO_WORD;
            s.fifo_read(&mut buf[..n])?;
            reads += 1;
            bytes += n;
            for w in buf[..n].chunks_exact(FIFO_WORD) {
                let cnt = (w[0] >> 1) & 3;
                match w[0] >> 3 {
                    TAG_ACCEL => {
                        accel.add(scaled(xyz(&w[1..]), set.g_per_lsb()));
                        slots.word(cnt, true);
                    }
                    TAG_GYRO => {
                        gyro.add(scaled(xyz(&w[1..]), set.dps_per_lsb()));
                        slots.word(cnt, false);
                    }
                    _ => other += 1,
                }
            }
        }
        if last >= secs {
            break;
        }
        if last >= next_print {
            next_print += 1.0;
            println!("{last:4.1} s  accel {}  gyro {}  backlog {words} words  overruns {overruns}", accel.n, gyro.n);
        }
    }
    s.power_down()?;

    let ratio = |n: u64| 100.0 * n as f64 / (rate * last);
    println!("accel {} samples = {:.2} Hz ({:.2} % of ODR)", accel.n, accel.n as f64 / last, ratio(accel.n));
    if set.gyro_on {
        println!("gyro  {} samples = {:.2} Hz ({:.2} % of ODR)", gyro.n, gyro.n as f64 / last, ratio(gyro.n));
    }
    let fitted = rate_fit.slope();
    println!(
        "rate from count vs host time (after 0.5 s): {fitted:.2} Hz, {:+.0} ppm against the trimmed ODR",
        (fitted / rate - 1.0) * 1e6
    );
    println!(
        "time slots {}, slot errors {}, FIFO overruns {overruns}, other tags {other}",
        slots.slots, slots.errors
    );
    println!(
        "{reads} burst reads, {:.1} kB/s from the FIFO, largest backlog {max_backlog} words",
        bytes as f64 / last / 1e3
    );
    accel.print("accel", "g", "mg", 1e3);
    if set.gyro_on {
        gyro.print("gyro ", "dps", "dps", 1.0);
    }
    println!("|accel mean| {:.4} g", magnitude(accel.mean));
    Ok(())
}
