//! `neuromancer-ahrs` — a dependency-free attitude and heading reference system.
//!
//! Implements the **Mahony complementary filter** for 3-DoF orientation from
//! accelerometer + gyroscope, plus quaternion math and YXZ Tait-Bryan
//! yaw/pitch/roll extraction.
//!
//! # Frames & conventions
//!
//! - **Body frame: RUB** — +X right, +Y up, +Z back (same as the Android
//!   sensor frame and `ar-drivers`). Gravity at rest reads `(0, +9.81, 0)` m/s².
//! - **World frame:** y-up, aligned with the body frame at the identity quaternion.
//! - **Quaternion** `q` is a unit quaternion rotating **world → body**
//!   (Hamilton convention, `w` scalar first).
//! - **Euler angles:** YXZ Tait-Bryan — yaw about world Y, then pitch about the
//!   rotated X, then roll about the rotated Z (intrinsic Y-X-Z). Returned in
//!   radians by [`quat_to_ypr`]; the tracker converts to degrees for HUD.
//!
//! There is no magnetometer input, so **yaw** is a pure gyro integration with
//! no absolute reference and drifts slowly; pitch/roll are corrected by
//! accelerometer gravity. That is expected and by design (see spec §2.4).

/// Unit quaternion rotating world → body. Hamilton convention, scalar-first.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    /// The identity quaternion (no rotation).
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quat {
        Quat { w, x, y, z }
    }

    /// Construct a unit quaternion from an axis-angle rotation
    /// (`axis` is normalized, `angle` in radians, right-handed).
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Quat {
        let [ax, ay, az] = axis;
        let half = angle * 0.5;
        let s = half.sin();
        Quat::new(half.cos(), ax * s, ay * s, az * s)
    }

    /// Conjugate (inverse for a unit quaternion).
    pub fn conjugate(self) -> Quat {
        Quat::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn norm(self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize to a unit quaternion (returns identity for a zero quaternion).
    pub fn normalize(self) -> Quat {
        let n = self.norm();
        if n < 1e-12 {
            return Quat::IDENTITY;
        }
        Quat::new(self.w / n, self.x / n, self.y / n, self.z / n)
    }

    /// Rotate a vector by this (unit) quaternion: `v' = q ⊗ v ⊗ q̄`.
    pub fn rotate_vector(self, v: [f64; 3]) -> [f64; 3] {
        let q = self.normalize();
        let p = Quat::new(0.0, v[0], v[1], v[2]);
        let r = q * p * q.conjugate();
        [r.x, r.y, r.z]
    }
}

impl std::ops::Mul for Quat {
    type Output = Quat;

    /// Hamilton quaternion product: `self ⊗ rhs`.
    fn mul(self, rhs: Quat) -> Quat {
        Quat::new(
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        )
    }
}

/// Normalize a quaternion to unit length.
pub fn quat_normalize(q: Quat) -> Quat {
    q.normalize()
}

/// Extract YXZ Tait-Bryan angles (yaw, pitch, roll) in **radians**.
///
/// Convention: yaw about world Y, pitch about the rotated X, roll about the
/// rotated Z. Equivalent to `R = Rz(roll) · Rx(pitch) · Ry(yaw)` mapping
/// world → body.
///
/// Returns `[yaw, pitch, roll]`. Pitch is `asin`-clamped to the valid range;
/// yaw/roll are well-defined for all orientations except exact gimbal-lock
/// configurations (pitch = ±90°).
pub fn quat_to_ypr(q: Quat) -> [f64; 3] {
    let q = q.normalize();
    let (w, x, y, z) = (q.w, q.x, q.y, q.z);

    let yaw = (2.0 * (w * y - x * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let sinp = (2.0 * (y * z + w * x)).clamp(-1.0, 1.0);
    let pitch = sinp.asin();
    let roll = (2.0 * (w * z - x * y)).atan2(1.0 - 2.0 * (x * x + z * z));
    [yaw, pitch, roll]
}

/// Build a world → body quaternion from YXZ Tait-Bryan angles in radians.
/// Inverse of [`quat_to_ypr`] (same convention).
pub fn quat_from_ypr(yaw: f64, pitch: f64, roll: f64) -> Quat {
    let qy = Quat::from_axis_angle([0.0, 1.0, 0.0], yaw);
    let qx = Quat::from_axis_angle([1.0, 0.0, 0.0], pitch);
    let qz = Quat::from_axis_angle([0.0, 0.0, 1.0], roll);
    // R = Rz(roll) · Rx(pitch) · Ry(yaw) ⇒ q = qz ⊗ qx ⊗ qy
    (qz * qx * qy).normalize()
}

/// Mahony complementary filter state.
///
/// Consumes every accelerometer/gyroscope sample and produces the fused
/// orientation as a world → body quaternion.
///
/// - `kp` proportional gain: how strongly the accelerometer pulls the
///   estimated gravity vector back toward the measured one.
/// - `ki` integral gain: eliminates steady-state tilt error.
///
/// Defaults per spec §2.4: `kp = 1.0`, `ki = 0.005`.
#[derive(Debug, Clone)]
pub struct Mahony {
    pub kp: f64,
    pub ki: f64,
    q: Quat,
    integral: [f64; 3],
}

/// Cap on the integral-error magnitude, to prevent wind-up during sustained
/// motion (spec: no magnetometer; integral only corrects slow tilt bias).
const INTEGRAL_LIMIT: f64 = 2.0;

impl Mahony {
    /// Create a filter with the spec defaults (`kp = 1.0`, `ki = 0.005`).
    pub fn new() -> Self {
        Self::with_gains(1.0, 0.005)
    }

    pub fn with_gains(kp: f64, ki: f64) -> Self {
        Mahony {
            kp,
            ki,
            q: Quat::IDENTITY,
            integral: [0.0; 3],
        }
    }

    /// Reset orientation to identity and clear the integral term.
    pub fn reset(&mut self) {
        self.q = Quat::IDENTITY;
        self.integral = [0.0; 3];
    }

    /// Current orientation as a unit quaternion (world → body).
    pub fn quaternion(&self) -> Quat {
        self.q
    }

    /// Current orientation as `[yaw, pitch, roll]` in radians.
    pub fn ypr(&self) -> [f64; 3] {
        quat_to_ypr(self.q)
    }

    /// One filter step.
    ///
    /// - `accel`: `[ax, ay, az]` — specific force in m/s² (gravity + linear
    ///   acceleration); normalized internally, any scale works.
    /// - `gyro`: `[gx, gy, gz]` — body-frame angular velocity in rad/s.
    /// - `dt`: elapsed time since the previous sample, in seconds. Pass `0.0`
    ///   for the first sample (gyro-only orientation correction still applies).
    ///
    /// Returns the updated orientation quaternion.
    pub fn update(&mut self, accel: [f64; 3], gyro: [f64; 3], dt: f64) -> Quat {
        // Estimated direction of gravity in the body frame (world +Y rotated
        // into the body frame by the current orientation).
        let v = self.q.rotate_vector([0.0, 1.0, 0.0]);

        // Normalized measured specific force. If the accelerometer reads ~0
        // (free fall / bad sample), skip the correction — pure gyro mode.
        let a_norm = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
        let (mut ex, mut ey, mut ez) = (0.0, 0.0, 0.0);
        if a_norm > 1e-6 {
            let a = [accel[0] / a_norm, accel[1] / a_norm, accel[2] / a_norm];
            // err = v × a rotates v toward a (see crate docs / unit tests).
            ex = v[1] * a[2] - v[2] * a[1];
            ey = v[2] * a[0] - v[0] * a[2];
            ez = v[0] * a[1] - v[1] * a[0];
        }

        if self.ki > 0.0 {
            self.integral[0] += self.ki * ex * dt;
            self.integral[1] += self.ki * ey * dt;
            self.integral[2] += self.ki * ez * dt;
            // Clamp integral magnitude to prevent wind-up.
            let mag = (self.integral[0] * self.integral[0]
                + self.integral[1] * self.integral[1]
                + self.integral[2] * self.integral[2])
                .sqrt();
            if mag > INTEGRAL_LIMIT {
                let s = INTEGRAL_LIMIT / mag;
                self.integral[0] *= s;
                self.integral[1] *= s;
                self.integral[2] *= s;
            }
        }

        // Corrected body angular velocity.
        let wx = gyro[0] + self.kp * ex + self.integral[0];
        let wy = gyro[1] + self.kp * ey + self.integral[1];
        let wz = gyro[2] + self.kp * ez + self.integral[2];

        // Quaternion kinematics (world → body, body-frame rates): q̇ = ½ q ⊗ ω.
        let qdot = (Quat::new(0.0, wx, wy, wz) * self.q).scale(0.5);

        // Integrate and normalize.
        let q_new = Quat::new(
            self.q.w + qdot.w * dt,
            self.q.x + qdot.x * dt,
            self.q.y + qdot.y * dt,
            self.q.z + qdot.z * dt,
        );
        self.q = q_new.normalize();
        self.q
    }
}

impl Default for Mahony {
    fn default() -> Self {
        Self::new()
    }
}

// Private helpers.
impl Quat {
    fn scale(self, s: f64) -> Quat {
        Quat::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEG: f64 = std::f64::consts::PI / 180.0;
    const G: f64 = 9.81;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!(
            (a - b).abs() <= eps,
            "|{a} - {b}| = {} > {eps}",
            (a - b).abs()
        );
    }

    fn assert_quat_close(a: Quat, b: Quat, eps: f64) {
        // Compare up to sign (q and -q are the same rotation).
        let d1 = (a.w - b.w).abs() + (a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs();
        let d2 = (a.w + b.w).abs() + (a.x + b.x).abs() + (a.y + b.y).abs() + (a.z + b.z).abs();
        assert!(d1.min(d2) <= eps, "quats differ: {a:?} vs {b:?}");
    }

    /// Identity quaternion is a multiplicative identity.
    #[test]
    fn quat_mul_identity() {
        let q = Quat::from_axis_angle([1.0, 0.0, 0.0], 0.7);
        let r = q * Quat::IDENTITY;
        assert_quat_close(r, q, 1e-12);
        let r2 = Quat::IDENTITY * q;
        assert_quat_close(r2, q, 1e-12);
    }

    /// Normalization turns any quaternion into a unit one.
    #[test]
    fn quat_normalize_unit() {
        let q = Quat::new(2.0, 0.0, 0.0, 2.0).normalize();
        assert_close(q.norm(), 1.0, 1e-12);
        assert_close(q.w, 2.0f64 / 8.0f64.sqrt(), 1e-12);
    }

    /// Conjugate of a unit quaternion is its inverse.
    #[test]
    fn quat_conjugate_inverse() {
        let q = Quat::from_axis_angle([0.0, 1.0, 0.0], 0.9);
        let r = q * q.conjugate();
        assert_quat_close(r, Quat::IDENTITY, 1e-12);
    }

    /// Rotating a vector with a quaternion matches the axis-angle expectation.
    #[test]
    fn rotate_vector_pure_yaw() {
        // +90° about world +Y (right-handed): +X → −Z.
        let q = Quat::from_axis_angle([0.0, 1.0, 0.0], 90.0 * DEG);
        let v = q.rotate_vector([1.0, 0.0, 0.0]);
        assert_close(v[0], 0.0, 1e-9);
        assert_close(v[1], 0.0, 1e-9);
        assert_close(v[2], -1.0, 1e-9);
        // Y-up stays put under a yaw.
        let u = q.rotate_vector([0.0, 1.0, 0.0]);
        assert_close(u[1], 1.0, 1e-9);
    }

    /// YPR extraction: pure yaw.
    #[test]
    fn ypr_pure_yaw() {
        let q = Quat::from_axis_angle([0.0, 1.0, 0.0], 30.0 * DEG);
        let [yaw, pitch, roll] = quat_to_ypr(q);
        assert_close(yaw, 30.0 * DEG, 1e-9);
        assert_close(pitch, 0.0, 1e-9);
        assert_close(roll, 0.0, 1e-9);
    }

    /// YPR extraction: pure pitch.
    #[test]
    fn ypr_pure_pitch() {
        let q = Quat::from_axis_angle([1.0, 0.0, 0.0], -25.0 * DEG);
        let [yaw, pitch, roll] = quat_to_ypr(q);
        assert_close(yaw, 0.0, 1e-9);
        assert_close(pitch, -25.0 * DEG, 1e-9);
        assert_close(roll, 0.0, 1e-9);
    }

    /// YPR extraction: pure roll.
    #[test]
    fn ypr_pure_roll() {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], 15.0 * DEG);
        let [yaw, pitch, roll] = quat_to_ypr(q);
        assert_close(yaw, 0.0, 1e-9);
        assert_close(pitch, 0.0, 1e-9);
        assert_close(roll, 15.0 * DEG, 1e-9);
    }

    /// YPR round-trip: q → YPR → q reproduces the original rotation.
    #[test]
    fn ypr_roundtrip() {
        // Deterministic pseudo-random quaternion sequence (avoid exact
        // gimbal-lock: keep |pitch| < 80°).
        let mut seed = 0x9E3779B97F4A7C15u64;
        let mut next = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        };
        for _ in 0..50 {
            let yaw = next() * 180.0 * DEG;
            let pitch = next() * 60.0 * DEG;
            let roll = next() * 180.0 * DEG;
            let q = quat_from_ypr(yaw, pitch, roll);
            let [y2, p2, r2] = quat_to_ypr(q);
            let q2 = quat_from_ypr(y2, p2, r2);
            assert_quat_close(q2, q, 1e-9);
        }
    }

    /// Mahony converges to level (pitch/roll → 0) from a tilted start with
    /// no gyro input and constant gravity.
    ///
    /// Note: with no magnetometer, yaw has no absolute reference — during a
    /// *large* initial-tilt transient the accel-only correction can rotate
    /// through world-y (the correction axis `v×a` has a yaw component for
    /// combined pitch+roll errors) and settle at a stable, non-zero yaw.
    /// In normal near-level tracking (T3) the error stays ~0 and this leak
    /// is negligible, as `mahony_gyro_yaw_integration` shows. The contract
    /// tested here: pitch/roll converge, and once converged yaw stops
    /// drifting (stable equilibrium).
    #[test]
    fn mahony_converges_to_level() {
        let mut f = Mahony::new();
        // Start tilted: pitch +30°, roll +20°.
        f.q = quat_from_ypr(0.0, 30.0 * DEG, 20.0 * DEG);
        let accel = [0.0, G, 0.0];
        let dt = 0.005;
        for _ in 0..4000 {
            f.update(accel, [0.0, 0.0, 0.0], dt);
        }
        let [yaw, pitch, roll] = f.ypr();
        assert!(pitch.abs() < 0.5 * DEG, "pitch {pitch} not converged");
        assert!(roll.abs() < 0.5 * DEG, "roll {roll} not converged");
        // Yaw settles (bounded, stable) — no magnetometer, so no absolute
        // reference; the transient may shift it but it must stop moving.
        let yaw_at_convergence = yaw;
        assert!(yaw.abs() < 10.0 * DEG, "yaw {yaw} far outside expected range");
        for _ in 0..4000 {
            f.update(accel, [0.0, 0.0, 0.0], dt);
        }
        let [yaw2, pitch2, roll2] = f.ypr();
        assert!(
            (yaw2 - yaw_at_convergence).abs() < 0.2 * DEG,
            "yaw kept drifting: {yaw_at_convergence} → {yaw2}"
        );
        assert!(pitch2.abs() < 0.5 * DEG && roll2.abs() < 0.5 * DEG);
    }

    /// Mahony tracks a constant body rotation rate (gyro-only yaw advance).
    #[test]
    fn mahony_gyro_yaw_integration() {
        let mut f = Mahony::new();
        let omega = 30.0 * DEG; // 30°/s
        let dt = 0.005;
        let accel = [0.0, G, 0.0];
        let steps = 200; // 1.0 s total
        for _ in 0..steps {
            f.update(accel, [0.0, omega, 0.0], dt);
        }
        let [yaw, pitch, roll] = f.ypr();
        assert_close(yaw, omega * steps as f64 * dt, 2.0 * DEG);
        assert!(pitch.abs() < 1.0 * DEG, "pitch {pitch} disturbed");
        assert!(roll.abs() < 1.0 * DEG, "roll {roll} disturbed");
    }

    /// Accelerometer correction pulls the orientation to the actual tilt.
    #[test]
    fn mahony_converges_to_tilted_gravity() {
        let mut f = Mahony::new();
        // Actual tilt: pitch −40° about body X.
        let tilt = -40.0 * DEG;
        let accel = quat_from_ypr(0.0, tilt, 0.0).rotate_vector([0.0, G, 0.0]);
        let dt = 0.005;
        for _ in 0..6000 {
            f.update(accel, [0.0, 0.0, 0.0], dt);
        }
        let [_, pitch, roll] = f.ypr();
        assert_close(pitch, tilt, 0.5 * DEG);
        assert!(roll.abs() < 0.5 * DEG, "roll {roll} not converged");
    }

    /// The quaternion stays unit length over many updates.
    #[test]
    fn mahony_keeps_unit_quaternion() {
        let mut f = Mahony::new();
        let dt = 0.005;
        for i in 0..5000 {
            // Random-ish accel/gyro, moderate magnitudes.
            let t = i as f64 * dt;
            let accel = [0.0 + 0.3 * (t * 2.0).sin(), G, 0.2 * (t * 3.0).cos()];
            let gyro = [0.5 * (t * 1.7).sin(), 1.0 * (t * 0.9).cos(), 0.3 * (t * 2.3).sin()];
            f.update(accel, gyro, dt);
            assert_close(f.quaternion().norm(), 1.0, 1e-9);
        }
    }

    /// Zero accelerometer input must not produce NaN — pure gyro mode.
    #[test]
    fn mahony_handles_zero_accel() {
        let mut f = Mahony::new();
        let dt = 0.005;
        for _ in 0..100 {
            f.update([0.0, 0.0, 0.0], [0.2, 0.1, -0.3], dt);
        }
        let q = f.quaternion();
        assert!(q.w.is_finite() && q.x.is_finite() && q.y.is_finite() && q.z.is_finite());
        assert_close(q.norm(), 1.0, 1e-9);
    }
}
