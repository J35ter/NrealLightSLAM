//! Startup stationary gyro-bias calibration.
//!
//! The Nreal Light has no magnetometer, so yaw has no absolute reference and
//! any residual gyro bias integrates into linear yaw drift (measured on real
//! glasses 2026-08-04: ~15°/60 s ≈ 0.25°/s constant — classic turn-on bias
//! left over after `ar-drivers`' static device calibration). The standard
//! fix: while the device is detected as stationary, the mean gyro reading
//! IS the bias — subtract it from every sample before the filter.

use crate::jsonl::ImuSample;

/// Gyro norm (rad/s) below which a sample counts as "still" for calibration.
const STILL_GYRO: f64 = 0.15;
/// Accelerometer norm band (fraction of g) for a sample to count as "still".
const STILL_ACCEL_MIN: f64 = 0.8;
const STILL_ACCEL_MAX: f64 = 1.2;
const G: f64 = 9.81;

/// Result of a finished calibration.
#[derive(Debug, Clone, Copy)]
pub struct CalibResult {
    /// Per-axis mean gyro bias to subtract, rad/s.
    pub bias: [f64; 3],
    /// Number of still samples used.
    pub samples: u64,
    /// Total wall-clock time spent collecting (s).
    pub elapsed: f64,
    /// Per-axis standard deviation of the still samples (rad/s) — a motion
    /// quality indicator.
    pub std: [f64; 3],
    /// True when the full window of still samples was collected; false when
    /// we hit the max wait with too little valid data (device was moving).
    pub complete: bool,
}

/// Collects gyro samples while the device is stationary and estimates bias.
///
/// All timing is derived from the sample timestamps (`t`, monotonic seconds
/// since stream start) — deterministic in tests and robust in production.
pub struct GyroCalibrator {
    /// Required accumulated time (s) of *still* samples.
    window: f64,
    /// Hard cap on elapsed stream time (s).
    max_wait: f64,
    /// Stream time of the first pushed sample.
    start_t: Option<f64>,
    prev_t: Option<f64>,
    valid_duration: f64,
    sum: [f64; 3],
    sum_sq: [f64; 3],
    count: u64,
    done: bool,
}

impl GyroCalibrator {
    pub fn new(window_seconds: f64) -> Self {
        let window = window_seconds.max(0.1);
        GyroCalibrator {
            window,
            max_wait: window * 5.0,
            start_t: None,
            prev_t: None,
            valid_duration: 0.0,
            sum: [0.0; 3],
            sum_sq: [0.0; 3],
            count: 0,
            done: false,
        }
    }

    /// Feed one sample. Returns `Some(result)` when calibration has finished,
    /// `None` while still collecting. `result.bias` is the mean gyro over the
    /// still window — subtract it from subsequent gyro samples.
    pub fn push(&mut self, s: &ImuSample) -> Option<CalibResult> {
        if self.done {
            return None;
        }
        let t0 = self.start_t.get_or_insert(s.t);
        let dt = match self.prev_t {
            Some(pt) => (s.t - pt).clamp(0.0, 0.1),
            None => 0.0,
        };
        self.prev_t = Some(s.t);

        let acc_norm = (s.ax * s.ax + s.ay * s.ay + s.az * s.az).sqrt();
        let gyro_norm = (s.gx * s.gx + s.gy * s.gy + s.gz * s.gz).sqrt();
        let still = acc_norm > STILL_ACCEL_MIN * G
            && acc_norm < STILL_ACCEL_MAX * G
            && gyro_norm < STILL_GYRO;
        if still {
            self.sum[0] += s.gx;
            self.sum[1] += s.gy;
            self.sum[2] += s.gz;
            self.sum_sq[0] += s.gx * s.gx;
            self.sum_sq[1] += s.gy * s.gy;
            self.sum_sq[2] += s.gz * s.gz;
            self.count += 1;
            self.valid_duration += dt;
        }

        let elapsed = s.t - *t0;
        // Small epsilon: accumulated dt sums can land a hair under the window
        // (binary floating point), which would otherwise delay completion.
        let complete = self.valid_duration >= self.window - 1e-6;
        let gave_up = elapsed >= self.max_wait;
        if !complete && !gave_up {
            return None;
        }
        self.done = true;

        let bias = if self.count > 0 {
            [
                self.sum[0] / self.count as f64,
                self.sum[1] / self.count as f64,
                self.sum[2] / self.count as f64,
            ]
        } else {
            [0.0; 3]
        };
        let std = |i: usize| -> f64 {
            if self.count > 1 {
                let mean = bias[i];
                let ms = self.sum_sq[i] / self.count as f64;
                (ms - mean * mean).max(0.0).sqrt()
            } else {
                0.0
            }
        };
        Some(CalibResult {
            bias,
            samples: self.count,
            elapsed,
            std: [std(0), std(1), std(2)],
            complete,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jsonl::ImuSample;

    fn sample(t: f64, gx: f64, gy: f64, gz: f64, accel: [f64; 3]) -> ImuSample {
        ImuSample {
            t,
            ax: accel[0],
            ay: accel[1],
            az: accel[2],
            gx,
            gy,
            gz,
        }
    }

    /// 2 s @ 200 Hz of constant bias + white-ish noise → bias recovered.
    #[test]
    fn recovers_constant_bias() {
        let mut c = GyroCalibrator::new(2.0);
        let bias = 0.00436; // ≈ 0.25°/s
        let mut result = None;
        // 2 s @ 200 Hz + margin: the first sample carries dt = 0, so the
        // accumulated still-duration needs one extra sample past 400.
        for i in 0..450 {
            let t = i as f64 * 0.005;
            // Deterministic pseudo-noise in ±0.005 rad/s.
            let noise = (i as f64 * 12.9898).sin() * 0.005;
            let s = sample(t, noise, bias + noise, -noise, [0.0, G, 0.0]);
            if let Some(r) = c.push(&s) {
                result = Some(r);
                break;
            }
        }
        let r = result.expect("calibration should finish");
        assert!(r.complete);
        assert!(r.samples >= 300, "samples {}", r.samples);
        assert!((r.bias[1] - bias).abs() < 0.002, "bias {:?}", r.bias);
        assert!(r.bias[0].abs() < 0.002 && r.bias[2].abs() < 0.002);
    }

    /// Moving samples do not count; calibration waits for stillness.
    #[test]
    fn ignores_motion_until_still() {
        let mut c = GyroCalibrator::new(1.0);
        let bias = 0.01;
        // 1 s of active motion (gyro far above threshold).
        for i in 0..200 {
            let s = sample(i as f64 * 0.005, 2.0, 0.5, -1.0, [0.0, G, 0.0]);
            assert!(c.push(&s).is_none());
        }
        // 1.2 s still → completes with the correct bias.
        let mut result = None;
        for i in 200..440 {
            let s = sample(i as f64 * 0.005, bias, 0.0, 0.0, [0.0, G, 0.0]);
            if let Some(r) = c.push(&s) {
                result = Some(r);
                break;
            }
        }
        let r = result.expect("should finish after stillness");
        assert!(r.complete);
        assert!((r.bias[0] - bias).abs() < 0.002, "bias {:?}", r.bias);
    }

    /// Never still → gives up at max_wait with zero samples (caller warns).
    #[test]
    fn gives_up_when_never_still() {
        let mut c = GyroCalibrator::new(0.2); // max_wait = 1.0 s
        let mut result = None;
        for i in 0..500 {
            let s = sample(i as f64 * 0.005, 3.0, -2.0, 1.0, [0.0, G, 0.0]);
            if let Some(r) = c.push(&s) {
                result = Some(r);
                break;
            }
        }
        let r = result.expect("must give up eventually");
        assert!(!r.complete);
        assert_eq!(r.samples, 0);
        assert_eq!(r.bias, [0.0; 3]);
    }
}
