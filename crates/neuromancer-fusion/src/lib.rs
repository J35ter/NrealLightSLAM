//! `neuromancer-fusion` — error-state Kalman filter (ESKF) fusing IMU
//! (gyro + accel) with visual odometry pose for 6-DoF tracking (P2b).
//!
//! The filter follows Solà's error-state Kalman filter formulation
//! ("Quaternion kinematics for the error-state Kalman filter", 2017): the
//! IMU drives a nominal-state prediction at IMU rate; VO pose corrections
//! are applied to a 15-dimensional error state (position, velocity,
//! orientation, gyro bias, accel bias) which is then injected into the
//! nominal state.
//!
//! # Frames & conventions (must match the rest of the stack)
//!
//! - **World frame:** y-up, gravity `g_w = (0, -9.81, 0)` m/s².
//! - **Body frame: RUB** — +X right, +Y up, +Z back (same as `ar-drivers`
//!   and `neuromancer-ahrs`). At rest the accelerometer reads
//!   `(0, +9.81, 0)` m/s² (specific force = −gravity).
//! - **Quaternion** `q`: unit quaternion rotating **world → body**
//!   (Hamilton, scalar-first `w`), identical convention to
//!   `neuromancer-ahrs::Quat`.
//! - **VO measurements** must be expressed in the **same world/body frames**
//!   as the filter state — i.e. already head-frame poses (the tracker
//!   pre-multiplies by `imu_to_camera_left`).
//!
//! # Nominal-state dynamics (IMU dead-reckoning)
//!
//! With `ω = ω_m − b_g` and `a = a_m − b_a` (both body frame):
//!
//! ```text
//! ṗ = v
//! v̇ = R(q)·a + g_w            (world frame; R rotates body→world)
//! q̇ = q ⊗ ½[0, ω]            (Hamilton product with the pure-quaternion ω)
//! ḃ_g = 0, ḃ_a = 0
//! ```
//!
//! # Error-state model (15 states)
//!
//! `δx = [δp, δv, δθ, δb_g, δb_a]` with `δθ` the small-angle orientation
//! error (body frame). Continuous-time dynamics:
//!
//! ```text
//! δṗ = δv
//! δv̇ = −R·[a]×·δθ − R·δb_a − R·n_a
//! δθ̇ = −[ω]×·δθ − δb_g − n_g
//! δḃ_g = n_bg, δḃ_a = n_ba
//! ```
//!
//! where `[·]×` is the skew-symmetric cross-product matrix and `n_*` are the
//! process noises (gyro, accel, gyro-bias random walk, accel-bias random
//! walk). The covariance is propagated with `F_d = I + F·dt`,
//! `Q_d = G·Q_c·Gᵀ·dt` (first-order discretization).
//!
//! # VO correction
//!
//! Measurement `z = {p_m, q_m}` (position + orientation of the body in the
//! world). Innovation:
//!
//! ```text
//! δz_p = p_m − p
//! δz_θ = Log(q_m ⊗ q⁻¹)        (body-frame rotation error, 3-vector)
//! ```
//!
//! Observation Jacobian `H` maps the 15-state error into `[δz_p; δz_θ]`:
//! `H = [[I₃, 0, 0, 0, 0], [0, 0, I₃, 0, 0]]`. Standard Kalman update on
//! the error state, then injection `p += δp, v += δv, q ← q ⊗ Exp(δθ),
//! b += δb` and reset of `δx` (with the "ESKF reset" covariance fix — here
//! the Joseph form update makes the reset exact to first order).

use nalgebra::{
    Isometry3, Matrix3, SMatrix, SVector, UnitQuaternion, Vector3,
};

/// World gravity (y-up world).
pub const GRAVITY: [f64; 3] = [0.0, -9.81, 0.0];

/// Filter configuration: noise magnitudes (all std-dev, SI units).
#[derive(Debug, Clone, Copy)]
pub struct EskfConfig {
    /// Gyro measurement noise std-dev (rad/s).
    pub gyro_noise: f64,
    /// Accelerometer measurement noise std-dev (m/s²).
    pub accel_noise: f64,
    /// Gyro-bias random-walk std-dev (rad/s², per √s).
    pub gyro_walk: f64,
    /// Accel-bias random-walk std-dev (m/s³, per √s).
    pub accel_walk: f64,
    /// VO position measurement noise std-dev (m).
    pub pos_noise: f64,
    /// VO orientation measurement noise std-dev (rad).
    pub ori_noise: f64,
}

impl Default for EskfConfig {
    fn default() -> Self {
        EskfConfig {
            // Matches the noise injected by the synthetic tests
            // (fusion_tracks_synthetic_trajectory): 0.002 rad/s gyro,
            // 0.05 m/s² accel, 0.02 m / 0.01 rad VO.
            gyro_noise: 0.002,
            accel_noise: 0.05,
            gyro_walk: 1e-5,
            accel_walk: 1e-4,
            pos_noise: 0.02,
            ori_noise: 0.01,
        }
    }
}

impl EskfConfig {
    /// Tuning for REAL scenes (the tracker, P2b live runs). The Nreal's VO
    /// pose on a still headset is far noisier than the synthetic model: M9
    /// measured inter-pose deltas up to ~0.17 m and yaw wandering ±12° over
    /// 20 s. Tight values (0.02 m / 0.01 rad) made the ESKF chase VO jitter
    /// and misattribute it to gyro bias (bg swung to 0.05 rad/s, fused yaw
    /// followed the VO wander instead of the smooth gyro).
    pub fn real_scene() -> Self {
        EskfConfig {
            gyro_noise: 0.002,   // ~0.1°/s
            accel_noise: 0.05,   // ~5 mg
            gyro_walk: 1e-5,     // slow bias drift
            accel_walk: 1e-4,
            pos_noise: 0.10,     // VO stereo depth accuracy on real scenes
            ori_noise: 0.05,     // VO orientation accuracy (~2.9°)
        }
    }
}

/// The ESKF state: nominal state plus the 15×15 error covariance.
#[derive(Debug, Clone)]
pub struct Eskf {
    p: Vector3<f64>,
    v: Vector3<f64>,
    q: UnitQuaternion<f64>,
    bg: Vector3<f64>,
    ba: Vector3<f64>,
    /// Error-state covariance P (15×15).
    p_cov: SMatrix<f64, 15, 15>,
    cfg: EskfConfig,
    /// Initial covariance (kept for `reset`).
    init_cov: SMatrix<f64, 15, 15>,
    /// Gravity in the **filter world frame** (m/s²). Defaults to
    /// `(0, -9.81, 0)` (world +Y up). The tracker sets this from the
    /// startup accelerometer reading when the VO world frame is
    /// camera-anchored (tilted relative to gravity).
    g_w: Vector3<f64>,
    /// Last known time (s) — used to guard against non-monotonic calls.
    t: f64,
}

/// Skew-symmetric cross-product matrix from a 3-vector.
fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

impl Eskf {
    /// Create a filter at the origin/identity, with `p0` the initial
    /// position and `init_std` the initial error-state std-dev per block:
    /// `[p, v, θ, bg, ba]` (5 values).
    pub fn new(cfg: EskfConfig, p0: Vector3<f64>, q0: UnitQuaternion<f64>, init_std: [f64; 5]) -> Self {
        let mut init_cov = SMatrix::<f64, 15, 15>::zeros();
        for (i, s) in init_std.iter().enumerate() {
            for k in 0..3 {
                let idx = i * 3 + k;
                init_cov[(idx, idx)] = s * s;
            }
        }
        Eskf {
            p: p0,
            v: Vector3::zeros(),
            q: q0,
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
            p_cov: init_cov,
            cfg,
            init_cov,
            g_w: Vector3::from(GRAVITY),
            t: 0.0,
        }
    }

    /// Set the gravity vector in the filter world frame. Call before the
    /// first `predict` when the world frame is not gravity-aligned (the
    /// tracker's VO world is anchored to the first camera frame, which sits
    /// ~15° off gravity on the headset).
    pub fn set_gravity(&mut self, g_w: Vector3<f64>) {
        self.g_w = g_w;
    }

    /// IMU prediction step: consume one gyro/accel sample pair (`dt` s
    /// after the previous call). Returns the updated fused pose.
    pub fn predict(&mut self, dt: f64, gyro: &Vector3<f64>, accel: &Vector3<f64>) -> Isometry3<f64> {
        if dt <= 0.0 {
            return self.pose();
        }
        self.t += dt;
        let w = gyro - self.bg;
        let a = accel - self.ba;

        // --- Nominal state ---
        // Orientation: q ⊗ Exp(ω·dt/2).
        let omega = w * (dt * 0.5);
        let angle = omega.norm();
        let dq = if angle > 1e-12 {
            let axis = omega / angle;
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle * 2.0)
        } else {
            UnitQuaternion::identity()
        };
        self.q *= dq;

        // Acceleration in world frame: a_w = R(q)ᵀ·a + g_w. R(q) maps
        // world→body, so the body→world rotation is its transpose/inverse.
        let a_w = self.q.inverse() * a + self.g_w;
        // Exact constant-accel kinematics: p += v·dt + ½·a·dt², v += a·dt.
        self.p += self.v * dt + a_w * (0.5 * dt * dt);
        self.v += a_w * dt;

        // --- Error-state covariance ---
        // F (15×15) continuous-time Jacobian:
        //   δṗ = δv
        //   δv̇ = −R[a]× δθ − R δb_a
        //   δθ̇ = −[ω]× δθ − δb_g
        // R rotates world→body; the error-state dynamics use the body→world
        // rotation Rᵀ (δv̇ is expressed in the world frame).
        let r = *self.q.to_rotation_matrix().transpose().matrix();
        let a_skew = skew(&a);
        let w_skew = skew(&w);
        let mut f = SMatrix::<f64, 15, 15>::zeros();
        // δṗ from δv (block 0,0 -> 1,0).
        for k in 0..3 {
            f[(k, 3 + k)] = 1.0;
        }
        // δv̇ from δθ, δb_a.
        let r_a_skew = r * a_skew;
        let r_neg = -&r;
        for i in 0..3 {
            for j in 0..3 {
                f[(3 + i, 6 + j)] = -r_a_skew[(i, j)];
                f[(3 + i, 12 + j)] = r_neg[(i, j)];
            }
        }
        // δθ̇ from δθ, δb_g.
        for i in 0..3 {
            for j in 0..3 {
                f[(6 + i, 6 + j)] = -w_skew[(i, j)];
                f[(6 + i, 9 + j)] = -((i == j) as i32 as f64);
            }
        }
        // First-order discretization: F_d = I + F·dt.
        let fd = SMatrix::<f64, 15, 15>::identity() + f * dt;

        // Q_c (12×12): gyro, accel, gyro-walk, accel-walk (each 3).
        let (sg2, sa2, sbg2, sba2) = (
            self.cfg.gyro_noise * self.cfg.gyro_noise,
            self.cfg.accel_noise * self.cfg.accel_noise,
            self.cfg.gyro_walk * self.cfg.gyro_walk,
            self.cfg.accel_walk * self.cfg.accel_walk,
        );
        // G (15×12): maps noises into error dynamics.
        let mut g = SMatrix::<f64, 15, 12>::zeros();
        // gyro noise -> δθ̇ (block row 6, col 0)
        for k in 0..3 {
            g[(6 + k, k)] = -1.0;
        }
        // accel noise -> δv̇ (row 3, col 3): −R·n_a
        for i in 0..3 {
            for j in 0..3 {
                g[(3 + i, 3 + j)] = -r[(i, j)];
            }
        }
        // gyro walk -> δḃ_g (row 9, col 6)
        for k in 0..3 {
            g[(9 + k, 6 + k)] = 1.0;
        }
        // accel walk -> δḃ_a (row 12, col 9)
        for k in 0..3 {
            g[(12 + k, 9 + k)] = 1.0;
        }
        let qc = SMatrix::<f64, 12, 12>::from_diagonal(&SVector::<f64, 12>::from([
            sg2, sg2, sg2, sa2, sa2, sa2, sbg2, sbg2, sbg2, sba2, sba2, sba2,
        ]));
        let g_qc = g * qc;
        let qd = g_qc * g.transpose() * dt;

        self.p_cov = fd * self.p_cov * fd.transpose() + qd;
        self.pose()
    }

    /// VO correction: fuse a 6-DoF pose measurement (`p_m`, `q_m`) of the
    /// body frame in the world frame. Returns the updated fused pose.
    pub fn correct(&mut self, p_m: &Vector3<f64>, q_m: &UnitQuaternion<f64>) -> Isometry3<f64> {
        // Innovation.
        let dz_p = p_m - self.p;
        let dz_theta = rotation_error(&self.q, q_m); // Log(q_m ⊗ q⁻¹) in body frame

        // H (6×15): position rows 0..3 -> δp; orientation rows 3..6 -> δθ.
        let mut h = SMatrix::<f64, 6, 15>::zeros();
        for k in 0..3 {
            h[(k, k)] = 1.0;
            h[(3 + k, 6 + k)] = 1.0;
        }
        // R (6×6).
        let (sp2, so2) = (self.cfg.pos_noise * self.cfg.pos_noise, self.cfg.ori_noise * self.cfg.ori_noise);
        let r_cov = SMatrix::<f64, 6, 6>::from_diagonal(&nalgebra::Vector6::from([sp2, sp2, sp2, so2, so2, so2]));

        // Kalman gain: K = P Hᵀ (H P Hᵀ + R)⁻¹
        let ph_t = self.p_cov * h.transpose();
        let s = h * ph_t + r_cov;
        let k = ph_t * s.try_inverse().unwrap_or_else(SMatrix::<f64, 6, 6>::zeros);

        // Error-state update.
        let dz = nalgebra::Vector6::from([dz_p.x, dz_p.y, dz_p.z, dz_theta.x, dz_theta.y, dz_theta.z]);
        let dx = k * dz;

        // Injection into the nominal state.
        self.p += dx.fixed_rows::<3>(0).into_owned();
        self.v += dx.fixed_rows::<3>(3).into_owned();
        let dtheta = dx.fixed_rows::<3>(6).into_owned();
        let angle = dtheta.norm();
        if angle > 1e-12 {
            let dq = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(dtheta / angle), angle);
            self.q *= dq;
        }
        self.bg += dx.fixed_rows::<3>(9).into_owned();
        self.ba += dx.fixed_rows::<3>(12).into_owned();

        // Joseph-form covariance update: P = (I − KH)P(I − KH)ᵀ + KRKᵀ.
        let i = SMatrix::<f64, 15, 15>::identity();
        let i_kh = i - k * h;
        self.p_cov = i_kh * self.p_cov * i_kh.transpose() + k * r_cov * k.transpose();

        self.pose()
    }

    /// Accelerometer gravity-direction correction (the Mahony-style level
    /// reference folded into the ESKF). The accelerometer at rest reads the
    /// specific force f = a − g in the body frame; when the device is not
    /// accelerating (‖f‖ ≈ g), the measured gravity direction in the body
    /// frame is −f̂. This pins roll/pitch at IMU rate so the filter does NOT
    /// rely on sparse, jittery VO orientation for leveling (which otherwise
    /// gets misattributed to gyro bias and runs away).
    ///
    /// Linearized measurement: `h(q) = R(q)·ĝ_w` (predicted gravity
    /// direction in the body frame), innovation `g_meas − h`, and with the
    /// right-error convention `q = q̂ ⊗ Exp(δθ)` the Jacobian is
    /// `H_θ = −R(q̂)·[ĝ_w]×` — a rank-2 projector (gravity cannot observe
    /// yaw, as it should).
    ///
    /// Call alongside `predict` for every IMU sample whose ‖f‖ is within
    /// `still_frac` of g. `still_frac = 0.2` is a sane default.
    pub fn correct_gravity(&mut self, accel: &Vector3<f64>, still_frac: f64) {
        let a = accel - self.ba;
        let norm = a.norm();
        if norm < 1e-6 {
            return;
        }
        let g_norm = self.g_w.norm();
        // Only trust the accel as a gravity reference when the specific-force
        // magnitude is close to g (device not accelerating).
        if (norm - g_norm).abs() > still_frac * g_norm {
            return;
        }
        let g_meas = -(a / norm); // measured gravity direction in the body frame
        let g_w_hat = self.g_w / g_norm;
        let g_hat = self.q * g_w_hat; // predicted gravity direction in body

        // Innovation and Jacobian (3×15: maps δθ only).
        let innovation = g_meas - g_hat;
        let r_mat = *self.q.to_rotation_matrix().matrix();
        let h_theta = -r_mat * skew(&g_w_hat); // −R(q̂)·[ĝ_w]×
        let mut h = SMatrix::<f64, 3, 15>::zeros();
        for i in 0..3 {
            for j in 0..3 {
                h[(i, 6 + j)] = h_theta[(i, j)];
            }
        }
        // R: accel noise projected onto the level directions (~2× accel
        // noise over g).
        let sigma = (2.0 * self.cfg.accel_noise / g_norm).max(1e-4);
        let r_cov = SMatrix::<f64, 3, 3>::identity() * (sigma * sigma);

        let ph_t = self.p_cov * h.transpose();
        let s = h * ph_t + r_cov;
        let k = ph_t * s.try_inverse().unwrap_or_else(SMatrix::<f64, 3, 3>::zeros);
        let dx = k * innovation;

        let dtheta = dx.fixed_rows::<3>(6).into_owned();
        let angle = dtheta.norm();
        if angle > 1e-12 {
            let dq = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(dtheta / angle), angle);
            self.q *= dq;
        }
        // Joseph-form update.
        let i = SMatrix::<f64, 15, 15>::identity();
        let i_kh = i - k * h;
        self.p_cov = i_kh * self.p_cov * i_kh.transpose() + k * r_cov * k.transpose();

        // Recompute the pose with the corrected orientation.
        self.pose();
    }

    /// Current fused pose: world → body (IMU/head frame).
    pub fn pose(&self) -> Isometry3<f64> {
        Isometry3::from_parts(
            nalgebra::Translation3::from(self.p),
            self.q,
        )
    }

    pub fn position(&self) -> &Vector3<f64> {
        &self.p
    }

    pub fn velocity(&self) -> &Vector3<f64> {
        &self.v
    }

    pub fn orientation(&self) -> &UnitQuaternion<f64> {
        &self.q
    }

    pub fn gyro_bias(&self) -> &Vector3<f64> {
        &self.bg
    }

    /// Set the gyro bias directly (e.g. from the startup stationary
    /// calibration). The ESKF will continue refining it via VO corrections.
    pub fn set_gyro_bias(&mut self, bg: Vector3<f64>) {
        self.bg = bg;
    }

    pub fn accel_bias(&self) -> &Vector3<f64> {
        &self.ba
    }

    /// Set the accel bias directly (e.g. from the startup stationary
    /// calibration). The ESKF will continue refining it via VO corrections.
    pub fn set_accel_bias(&mut self, ba: Vector3<f64>) {
        self.ba = ba;
    }

    pub fn covariance(&self) -> &SMatrix<f64, 15, 15> {
        &self.p_cov
    }

    /// Time in seconds since construction (advanced by `predict`).
    pub fn t(&self) -> f64 {
        self.t
    }

    /// Reset to identity/origin state with the configured initial covariance.
    pub fn reset(&mut self, p0: Vector3<f64>, q0: UnitQuaternion<f64>) {
        self.p = p0;
        self.v = Vector3::zeros();
        self.q = q0;
        self.bg = Vector3::zeros();
        self.ba = Vector3::zeros();
        self.p_cov = self.init_cov;
        self.t = 0.0;
    }
}

/// Body-frame rotation error `Log(q̂⁻¹ ⊗ q_m)` as a 3-vector (rad).
///
/// The error-state orientation dynamics are in the **body frame**
/// (`q = q̂ ⊗ Exp(δθ)`, right-error convention — `δθ̇ = −[ω]×δθ − δb_g`), so
/// the innovation must be the body-frame relative rotation. Using the
/// world-frame error `Log(q_m ⊗ q̂⁻¹)` here breaks the loop: the correction
/// axis is rotated by `q̂`, and once the state leaves identity (e.g. the
/// headset's ~15° resting pitch) the innovation and injection disagree and
/// the orientation diverges exponentially.
fn rotation_error(q: &UnitQuaternion<f64>, q_m: &UnitQuaternion<f64>) -> Vector3<f64> {
    let dq = q.conjugate() * q_m;
    let angle = dq.angle();
    if angle < 1e-12 {
        return Vector3::zeros();
    }
    let axis = dq.axis().map(|u| u.into_inner());
    match axis {
        Some(ax) => ax * angle,
        None => Vector3::zeros(),
    }
}

/// Convert a nalgebra `Isometry3` into the AHRS-style `[w, x, y, z]` quaternion
/// components (scalar-first), matching `neuromancer-ahrs::Quat`.
pub fn pose_to_quat_xyzw(pose: &Isometry3<f64>) -> [f64; 4] {
    let q = pose.rotation.quaternion();
    [q.w, q.i, q.j, q.k]
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn identity_cfg() -> EskfConfig {
        EskfConfig {
            gyro_noise: 1e-9,
            accel_noise: 1e-9,
            gyro_walk: 1e-12,
            accel_walk: 1e-12,
            pos_noise: 1e-6,
            ori_noise: 1e-6,
        }
    }

    #[test]
    fn at_rest_position_holds() {
        // Gravity compensation: with a_m = (0, +9.81, 0) (specific force at
        // rest) and zero gyro, the filter must stay put.
        let mut eskf = Eskf::new(identity_cfg(), Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        let g = Vector3::new(0.0, 9.81, 0.0);
        let z = Vector3::zeros();
        for _ in 0..100 {
            eskf.predict(0.01, &z, &g);
        }
        assert!(eskf.position().norm() < 1e-6, "pos {}", eskf.position());
        assert!(eskf.velocity().norm() < 1e-6, "v {}", eskf.velocity());
    }

    #[test]
    fn constant_accel_propagates_position() {
        let mut eskf = Eskf::new(identity_cfg(), Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        // Forward acceleration of 1 m/s² in body +X, no gravity (use a_m =
        // a_body + 9.81*y to cancel gravity). After 1 s: p = 0.5 m in x.
        let a_m = Vector3::new(1.0, 9.81, 0.0);
        let z = Vector3::zeros();
        for _ in 0..100 {
            eskf.predict(0.01, &z, &a_m);
        }
        assert!((eskf.position().x - 0.5).abs() < 1e-3, "x {}", eskf.position().x);
        assert!((eskf.position().y).abs() < 1e-3, "y {}", eskf.position().y);
        assert!((eskf.velocity().x - 1.0).abs() < 1e-3, "vx {}", eskf.velocity().x);
    }

    #[test]
    fn gyro_rotates_orientation() {
        let mut eskf = Eskf::new(identity_cfg(), Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        // Yaw rotation of 0.5 rad/s about world +Y (body +Y at identity).
        let gyro = Vector3::new(0.0, 0.5, 0.0);
        let g = Vector3::new(0.0, 9.81, 0.0);
        for _ in 0..200 {
            eskf.predict(0.01, &gyro, &g);
        }
        let [w, x, y, z] = pose_to_quat_xyzw(&eskf.pose());
        let q = neuromancer_ahrs::Quat::new(w, x, y, z);
        let [yaw, _, _] = neuromancer_ahrs::quat_to_ypr(q);
        assert!((yaw - 1.0).abs() < 1e-2, "yaw {} (expected ~1.0 rad)", yaw);
    }

    // --- Synthetic trajectory + IMU/VO measurement generation --------------
    //
    // Ground truth: yaw ψ(t) = 0.2·t about world +Y, position
    // p(t) = (0.15t, 0, 0.15·sin(0.6t)) — a slow turn with sinusoidal
    // acceleration, enough to exercise the full state.
    struct Gt {
        t: f64,
    }

    impl Gt {
        fn p(&self) -> Vector3<f64> {
            Vector3::new(0.15 * self.t, 0.0, 0.15 * (0.6 * self.t).sin())
        }
        fn a_world(&self) -> Vector3<f64> {
            Vector3::new(0.0, 0.0, -0.054 * (0.6 * self.t).sin())
        }
        fn q(&self) -> UnitQuaternion<f64> {
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(Vector3::y_axis().into_inner()), 0.2 * self.t)
        }
        /// Body-frame specific force = R(q)·(a_w − g_w) = q·a_w − q·g_w.
        fn specific_force(&self) -> Vector3<f64> {
            self.q() * self.a_world() - self.q() * Vector3::from(GRAVITY)
        }
        /// Body angular velocity: pure yaw about world +Y = body +Y here.
        fn omega_body(&self) -> Vector3<f64> {
            Vector3::new(0.0, 0.2, 0.0)
        }
    }

    /// Deterministic LCG for reproducible noise in tests.
    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((*seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0 // ~U(-1,1)
    }

    #[test]
    fn fusion_tracks_synthetic_trajectory() {
        // IMU at 200 Hz, VO at 30 Hz, 4 s run.
        let dt_imu = 0.005;
        let dt_vo = 1.0 / 30.0;
        let dur = 4.0;
        // Biases + realistic noise (default config).
        let cfg = EskfConfig::default();
        let b_g = Vector3::new(0.01, -0.005, 0.008);
        let b_a = Vector3::new(0.02, 0.01, -0.03);
        let mut eskf = Eskf::new(cfg, Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        let mut seed = 42u64;

        let mut t_imu = 0.0;
        let mut t_next_vo = 0.0;
        let mut last_vo_err_p = 0.0;
        while t_imu < dur {
            let gt = Gt { t: t_imu };
            // Synthetic IMU measurement with noise + bias.
            let w_m = gt.omega_body() + b_g + Vector3::new(0.002 * lcg(&mut seed), 0.002 * lcg(&mut seed), 0.002 * lcg(&mut seed));
            let a_m = gt.specific_force() + b_a + Vector3::new(0.05 * lcg(&mut seed), 0.05 * lcg(&mut seed), 0.05 * lcg(&mut seed));
            eskf.predict(dt_imu, &w_m, &a_m);
            t_imu += dt_imu;

            // VO correction at 30 Hz.
            if t_imu >= t_next_vo {
                t_next_vo = t_imu + dt_vo;
                let gt = Gt { t: t_imu };
                let p_m = gt.p() + Vector3::new(0.02 * lcg(&mut seed), 0.02 * lcg(&mut seed), 0.02 * lcg(&mut seed));
                let dq = UnitQuaternion::from_axis_angle(
                    &nalgebra::Unit::new_normalize(Vector3::new(1.0, 2.0, -1.0).normalize()),
                    0.01 * lcg(&mut seed),
                );
                let q_m = gt.q() * dq;
                eskf.correct(&p_m, &q_m);
                last_vo_err_p = (eskf.position() - gt.p()).norm();
            }
        }

        let gt_end = Gt { t: dur };
        let err_p = (eskf.position() - gt_end.p()).norm();
        let err_q = UnitQuaternion::new_unchecked(eskf.orientation().quaternion() * *gt_end.q().conjugate()).angle();
        assert!(err_p < 0.08, "final position err {err_p} m (GT {} vs est {})", gt_end.p(), eskf.position());
        assert!(err_q < 0.05, "final orientation err {err_q} rad");
        assert!(last_vo_err_p < 0.08, "last VO-gated position err {last_vo_err_p} m");
        // Biases should be pulled toward their true values by the VO updates.
        assert!((eskf.gyro_bias() - b_g).norm() < 0.01, "gyro bias est {} vs {}", eskf.gyro_bias(), b_g);
        assert!((eskf.accel_bias() - b_a).norm() < 0.08, "accel bias est {} vs {}", eskf.accel_bias(), b_a);
    }

    #[test]
    fn without_vo_corrections_position_diverges() {
        // Same IMU stream, no VO: pure dead-reckoning must drift (the accel
        // bias integrates into unbounded position error), proving the VO
        // correction is what keeps the fused pose accurate.
        let cfg = EskfConfig::default();
        let b_g = Vector3::new(0.01, -0.005, 0.008);
        let b_a = Vector3::new(0.02, 0.01, -0.03);
        let mut eskf = Eskf::new(cfg, Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        let mut seed = 7u64;
        let mut t = 0.0;
        while t < 4.0 {
            let gt = Gt { t };
            let w_m = gt.omega_body() + b_g + Vector3::new(0.002 * lcg(&mut seed), 0.002 * lcg(&mut seed), 0.002 * lcg(&mut seed));
            let a_m = gt.specific_force() + b_a + Vector3::new(0.05 * lcg(&mut seed), 0.05 * lcg(&mut seed), 0.05 * lcg(&mut seed));
            eskf.predict(0.005, &w_m, &a_m);
            t += 0.005;
        }
        let gt_end = Gt { t: 4.0 };
        let err_p = (eskf.position() - gt_end.p()).norm();
        // Dead-reckoned position error with 0.01 m/s²-ish bias over 4 s of
        // accelerated motion is far worse than the fused 0.08 m bound.
        assert!(err_p > 0.3, "expected divergence, got {err_p} m");
    }

    /// Regression: the orientation innovation must be the BODY-frame error
    /// (right-error convention). The world-frame innovation `Log(q_m ⊗ q̂⁻¹)`
    /// disagrees with the body-frame dynamics once the state leaves identity
    /// (e.g. the headset's ~15° resting pitch), and the orientation diverges
    /// exponentially. Pure-yaw tests can't catch it — yaw about the shared
    /// vertical axis is identical in both frames.
    #[test]
    fn orientation_correction_converges_from_resting_pitch() {
        let cfg = EskfConfig::default();
        // Start at the headset's resting attitude (15° pitch) with zero
        // motion; VO measures the SAME attitude every frame (still headset).
        let q_rest = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::x_axis().into_inner()),
            15f64.to_radians(),
        );
        // Correct the initial offset (start from identity, not the rest pose).
        let mut eskf = Eskf::new(cfg, Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        let mut seed = 123u64;
        let mut t = 0.0;
        let mut max_err_q = 0.0f64;
        let mut max_err_p = 0.0f64;
        // At rest with attitude q_rest, the accelerometer reads the specific
        // force in the BODY frame: q_rest · (0, +9.81, 0) (world gravity
        // rotated into body). Feeding a constant (0,9.81,0) is only correct
        // at identity — after the orientation converges the filter would see
        // a spurious ~g·sin(15°) accel and the position would drift.
        let g_body = q_rest * Vector3::new(0.0, 9.81, 0.0);
        let z = Vector3::zeros();
        let mut next_vo = 0.0;
        while t < 3.0 {
            eskf.predict(0.001, &z, &g_body); // still headset: no gyro, gravity only
            t += 0.001;
            if t >= next_vo {
                next_vo = t + 1.0 / 30.0;
                // VO measures the resting pose with small noise.
                let dq = UnitQuaternion::from_axis_angle(
                    &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
                    0.005 * lcg(&mut seed),
                );
                eskf.correct(&Vector3::zeros(), &(q_rest * dq));
                let err_q = UnitQuaternion::new_unchecked(eskf.orientation().quaternion() * *q_rest.conjugate()).angle();
                let err_p = eskf.position().norm();
                max_err_q = max_err_q.max(err_q);
                max_err_p = max_err_p.max(err_p);
            }
        }
        // The filter must converge to the resting pose and STAY there (the
        // body-frame innovation keeps the correction axis aligned with the
        // actual error; the world-frame version diverges to tens of degrees).
        assert!(max_err_q.to_degrees() < 2.0, "max orientation err {:.2}°", max_err_q.to_degrees());
        assert!(max_err_p < 0.05, "max position err {max_err_p} m");
    }

    /// The accelerometer gravity correction alone (no VO at all) must hold
    /// the level attitude: start tilted 15°, feed the still-specific force
    /// and zero gyro, and the filter must converge to and stay at the true
    /// attitude — this is the Mahony-style roll/pitch anchor that keeps the
    /// ESKF level when VO corrections are sparse or jittery.
    #[test]
    fn gravity_correction_holds_level_without_vo() {
        // Perfect gyro (no drift to fight), light accel noise.
        let cfg = EskfConfig {
            gyro_noise: 1e-9,
            accel_noise: 0.02,
            ..EskfConfig::default()
        };
        // World gravity-aligned (the default g_w) for this unit test.
        let q_true = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::x_axis().into_inner()),
            15f64.to_radians(),
        );
        // Start WRONG (identity) — the correction must pull it to q_true.
        let mut eskf = Eskf::new(cfg, Vector3::zeros(), UnitQuaternion::identity(), [0.1, 0.1, 0.1, 0.01, 0.01]);
        let z = Vector3::zeros();
        // Specific force in the body frame at rest with attitude q_true:
        // f = −g_body = −q_true·g_w.
        let f_rest = -(q_true * Vector3::from(GRAVITY));
        let mut t = 0.0;
        while t < 2.0 {
            eskf.predict(0.001, &z, &f_rest);
            eskf.correct_gravity(&f_rest, 0.2);
            t += 0.001;
        }
        let err_q = UnitQuaternion::new_unchecked(eskf.orientation().quaternion() * *q_true.conjugate()).angle();
        assert!(
            err_q.to_degrees() < 1.0,
            "gravity correction failed to converge: err {:.2}° (q {} vs {})",
            err_q.to_degrees(),
            eskf.orientation(),
            q_true
        );
        assert!(eskf.position().norm() < 0.05, "position err {}", eskf.position());
    }
}
