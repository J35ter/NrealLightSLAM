//! Camera calibration wiring (M8): build the VO `Rectifier` from the
//! glasses' on-device factory calibration, with a fallback to the constants
//! captured for this unit (`cam_probe --calib`, 2026-08-04) for replay mode
//! where the device is not reachable.
//!
//! The device calibration comes from `ar-drivers` `CameraDescriptor`
//! (`NrealLight::cameras()` reads the `SLAM_camera` config JSON): per-camera
//! intrinsics (`fc`, `cc`), distortion (`kc`), and `imu_to_camera` pose. The
//! stereo relative pose is derived as
//! `left_t_right = imu_to_camera_left⁻¹ ∘ imu_to_camera_right`, which
//! reproduces the raw `leftcam_q_rightcam` rotation and a +X baseline.

use neuromancer_vo::rectify::{CalibCam, Rectifier};
use neuromancer_vo::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use crate::{log_info, log_warn};

/// Build a unit quaternion from (w, x, y, z) components.
fn quat(w: f64, x: f64, y: f64, z: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

/// Fallback calibration: this unit's factory values (`cam_probe --calib`,
/// j35ter-A5, 2026-08-04). Used when the device is not reachable (replay).
pub fn fallback_calib_for_probe() -> (CalibCam, CalibCam, Isometry3<f64>) {
    fallback()
}

fn fallback() -> (CalibCam, CalibCam, Isometry3<f64>) {    let left = CalibCam {
        fx: 234.4626,
        fy: 234.4677,
        cx: 325.1567,
        cy: 245.7264,
        kc: [0.066807, -0.025084, 0.001402, 0.003390, 0.0],
        width: 640,
        height: 480,
    };
    let right = CalibCam {
        fx: 234.3956,
        fy: 234.8681,
        cx: 318.9894,
        cy: 214.0771,
        kc: [0.064640, -0.022963, 0.003997, -0.003452, 0.0],
        width: 640,
        height: 480,
    };
    let left_t_right = left_t_right_from_imu_to_camera(
        &Isometry3::from_parts(
            Translation3::new(-0.049104, 0.005821, 0.021972),
            quat(0.991640, 0.128183, -0.003317, 0.014438),
        ),
        &Isometry3::from_parts(
            Translation3::new(0.053799, 0.008115, 0.021758),
            quat(0.991876, 0.126886, -0.001336, 0.008974),
        ),
    );
    (left, right, left_t_right)
}

/// `left_t_right` from the two cameras' `imu_to_camera` poses:
/// `p_left = imu_to_camera_left⁻¹ ∘ imu_to_camera_right · p_right`.
/// Verified 2026-08-04 to reproduce the raw `leftcam_q_rightcam` rotation
/// (0.0119 rad) and a +X baseline of ~0.103 m.
fn left_t_right_from_imu_to_camera(
    imu_to_cam_l: &Isometry3<f64>,
    imu_to_cam_r: &Isometry3<f64>,
) -> Isometry3<f64> {
    imu_to_cam_l.inverse() * imu_to_cam_r
}

/// Build the rectifier. In live (`--input visual`, hardware) mode the
/// on-device factory calibration is read from the glasses; in replay mode
/// the device is not needed (and probing it from parallel test processes
/// contends on the MCU), so the unit fallback constants are used.
pub fn build_rectifier(live: bool) -> (Rectifier, Option<Isometry3<f64>>) {
    if live {
        if let Some((left, right, left_t_right, imu_to_cam_left)) = device_calibration() {
            let r = Rectifier::new(&left, &right, left_t_right);
            log_info!("camera calibration: live device (baseline {:.3} m, coverage {:.1}%)",
                r.rig.left_t_right.translation.vector.x, r.coverage() * 100.0);
            return (r, Some(imu_to_cam_left));
        }
    }
    let (left, right, left_t_right) = fallback();
    let r = Rectifier::new(&left, &right, left_t_right);
    log_warn!(
        "camera calibration: no live descriptors — using unit fallback constants (baseline {:.3} m)",
        r.rig.left_t_right.translation.vector.x
    );
    (r, None)
}

/// Read the SLAM cameras' calibration from the live device. Returns
/// (left CalibCam, right CalibCam, left_t_right, left imu_to_camera).
fn device_calibration() -> Option<(CalibCam, CalibCam, Isometry3<f64>, Isometry3<f64>)> {
    use ar_drivers::ARGlasses;
    let glasses = ar_drivers::nreal_light::NrealLight::new().ok()?;
    let cams = glasses.cameras().ok()?;
    let left = cams.iter().find(|c| c.name == "Nreal Light SLAM left")?;
    let right = cams.iter().find(|c| c.name == "Nreal Light SLAM right")?;

    let to_calib = |c: &ar_drivers::CameraDescriptor| CalibCam {
        fx: c.intrinsic_matrix[(0, 0)],
        fy: c.intrinsic_matrix[(1, 1)],
        cx: c.intrinsic_matrix[(0, 2)],
        cy: c.intrinsic_matrix[(1, 2)],
        kc: c.distortion,
        width: c.resolution.x as u32,
        height: c.resolution.y as u32,
    };
    let left_c = to_calib(left);
    let right_c = to_calib(right);
    let left_t_right = left_t_right_from_imu_to_camera(&left.imu_to_camera, &right.imu_to_camera);
    let imu_to_cam_left = left.imu_to_camera;
    Some((left_c, right_c, left_t_right, imu_to_cam_left))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fallback_baseline_and_rotation_sanity() {
        let (left, right, left_t_right) = fallback();
        assert_eq!(left.width, 640);
        assert_eq!(right.height, 480);
        // Baseline ~0.103 m along +X, tiny rotation (matches leftcam_q_rightcam).
        let t = left_t_right.translation.vector;
        assert!((t.x - 0.103).abs() < 0.002, "baseline x = {}", t.x);
        assert!(t.y.abs() < 0.01 && t.z.abs() < 0.01);
        let ang = left_t_right.rotation.angle();
        assert!((ang - 0.0119).abs() < 0.002, "rotation {ang}");
    }

    #[test]
    fn fallback_rectifier_builds() {
        let (left, right, left_t_right) = fallback();
        let rect = Rectifier::new(&left, &right, left_t_right);
        assert!((rect.rig.left.fx - 234.4).abs() < 0.1);
        assert!(rect.coverage() > 0.9, "coverage {}", rect.coverage());
    }
}
