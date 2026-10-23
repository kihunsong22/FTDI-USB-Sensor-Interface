# Engine vibration metrics (metrics version 1)

Definitions behind `engine-metrics` and `metrics-compare`. Every value is a fixed
computation on the recorded samples: the same version on the same file gives the
same numbers, and any change to a definition or default bumps `METRICS_VERSION`
(the `metrics_version` CSV column).

Scope is what one ADXL355 on the engine measures well: 3 axes, DC to about
1 kHz, +/-8 g. Not covered: content above 1 kHz (knock, bearing defects), shocks
above 8 g, and seat-based human vibration (ISO 2631 needs the sensor on the seat).

## Engine speed from vibration

A 4-stroke engine with `c` cylinders fires `c/2` times per revolution, so the
firing order (2nd order for 4 cylinders) appears at `rpm/60 x c/2` Hz.

- Frames of 0.5 s every 0.05 s, Hann window, mean removed, zero padded x8.
  Power is summed over the three axes.
- The peak is searched between 500 and 7000 rpm, weighted by its second harmonic
  only to break near-ties, and refined by parabolic interpolation.
- A frame is valid when the 20-200 Hz level exceeds `--gate` (0.02 g RMS, 3-axis)
  and the peak is at least 10x the median of the search band.

Accuracy on a synthetic signal: within 2 rpm. The sample rate is measured from
the timestamps, so the sensor clock offset does not bias the rpm.

## Segments

| Kind | Rule | Default |
|---|---|---|
| steady | Consecutive valid frames within +/-2% of the running mean rpm | at least 10 s (`--steady-min`) |
| decel | rpm falls through `--decel-hi`, then `--decel-lo`, on valid frames | 2500 to 1500 rpm, within 10 s |

A decel segment is dropped if neighbouring frames differ by more than 10%
(tracking jump), if it spans fewer than 3 frames, or if the linear fit has
R^2 < 0.9 (gear shifts, uneven drops).

## Steady-segment metrics

Spectra are Welch PSDs (Hann, 2 s segments, 50% overlap, mean removed), summed
over the three axes. "3-axis" means the square root of the summed powers.

| Column | Unit | Definition | Typical use |
|---|---|---|---|
| `rpm_mean`, `rpm_sd` | rpm | Mean and SD of the frame rpm | Speed, idle stability |
| `ord_<k>_g` (k = 0.5, 1, 1.5, 2, 3, 4, 6) | g RMS, 3-axis | Power within +/-max(1 Hz, 2.5%) of order k | Order analysis |
| `ord_0_5_re_fire_db` | dB | 0.5 order relative to the firing order | Cylinder-to-cylinder combustion uniformity |
| `band_2_20_g` | g RMS, 3-axis | Band power 2-20 Hz | Rigid-body shake on the mounts |
| `band_20_200_g` | g RMS, 3-axis | Band power 20-200 Hz | Engine orders |
| `band_200_800_g` | g RMS, 3-axis | Band power 200-800 Hz (below the sensor filter edge) | Mechanical noise |
| `accel_rms_ms2`, `accel_db` | m/s^2, dB re 1e-6 m/s^2 | Overall 10-1000 Hz, 3-axis | Overall level, NVH convention |
| `velocity_rms_mms` | mm/s | 10-1000 Hz, highest axis | Machine vibration convention (ISO 20816 style) |
| `crest_factor` | - | Peak / RMS of 10-1000 Hz acceleration, highest axis | Impulsiveness |
| `kurtosis` | - | 4th moment / variance^2 of 10-1000 Hz acceleration, highest axis (3 = Gaussian) | Impulsiveness |
| `dc_x_g`, `dc_y_g`, `dc_z_g` | g | Mean acceleration per axis | Sensor attitude: a change means the mount moved |
| `temperature_c` | C | Mean sensor temperature (blank if not recorded) | Condition record |

Meter-style values (`accel_rms_ms2`, `velocity_rms_mms`, `crest_factor`) use the
same filters as `analyzer --vibration` and skip the first 1 s of the segment.

## Deceleration metrics

| Column | Unit | Definition |
|---|---|---|
| `duration_s` | s | Time between the two rpm crossings (interpolated between frames) |
| `decel_rate_rpm_s` | rpm/s | Least-squares slope of rpm against time between the crossings (negative) |
| `decel_fit_r2` | - | R^2 of that fit |

With the throttle released in neutral and fuel cut, the deceleration is the
engine's motoring torque (friction, pumping, accessories) divided by its
rotating inertia.

## Time columns

- `time_origin`: wall-clock time of `timestamps` = 0 (HDF5 attribute
  `metadata/timestamps_origin`). Files recorded before that attribute existed
  fall back to `metadata/start_time`, which is up to about 0.1 s earlier;
  `time_origin_source` says which one was used.
- `abs_start`: `time_origin` + `t_start_s`. Alignment with other loggers is
  limited by the PC clock; a shared event (e.g. engine start) is the safest sync.

## Comparison (`metrics-compare`)

Each CSV is one session (e.g. one day). Rows of the chosen kind and rpm range
are averaged per session; sessions are the unit of comparison.

- `change`: after mean minus before mean, in % of the before mean (dB columns: in dB).
- `MDC` (minimum detectable change): 2.8 x pooled session SD x sqrt(1/n_before + 1/n_after),
  for 5% significance and 80% power. Needs at least 2 sessions per group.
- `beyond`: |change| > MDC.
- `<column>@rpm`: amplitude scaled to the mean rpm of all selected rows,
  amplitude x (rpm_ref / rpm)^exp (`--rpm-exp`, default 2: inertia forces grow
  with rpm^2). Use it when sessions ran at slightly different rpm.
