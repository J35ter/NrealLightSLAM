//! Axis mapping: inversion flags + sensitivity, applied to the filtered
//! yaw/pitch/roll **in degrees** before any output (spec §3.5).

/// Coordinate & calibration layer.
#[derive(Debug, Clone, Copy)]
pub struct AxisMap {
    pub invert_yaw: bool,
    pub invert_pitch: bool,
    pub invert_roll: bool,
    /// Output sensitivity multiplier (≥ 0).
    pub sensitivity: f64,
}

impl Default for AxisMap {
    fn default() -> Self {
        AxisMap {
            invert_yaw: false,
            invert_pitch: false,
            invert_roll: false,
            sensitivity: 1.0,
        }
    }
}

impl AxisMap {
    /// Apply inversion (sign flip) then sensitivity (scale) in place.
    pub fn apply(&self, ypr_deg: &mut [f64; 3]) {
        if self.invert_yaw {
            ypr_deg[0] = -ypr_deg[0];
        }
        if self.invert_pitch {
            ypr_deg[1] = -ypr_deg[1];
        }
        if self.invert_roll {
            ypr_deg[2] = -ypr_deg[2];
        }
        if self.sensitivity != 1.0 {
            ypr_deg[0] *= self.sensitivity;
            ypr_deg[1] *= self.sensitivity;
            ypr_deg[2] *= self.sensitivity;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_map_leaves_angles_untouched() {
        let m = AxisMap::default();
        let mut ypr = [10.0, -20.0, 30.0];
        m.apply(&mut ypr);
        assert_eq!(ypr, [10.0, -20.0, 30.0]);
    }

    #[test]
    fn inversion_flips_signs() {
        let m = AxisMap {
            invert_yaw: true,
            invert_pitch: false,
            invert_roll: true,
            sensitivity: 1.0,
        };
        let mut ypr = [10.0, -20.0, 30.0];
        m.apply(&mut ypr);
        assert_eq!(ypr, [-10.0, -20.0, -30.0]);
    }

    #[test]
    fn sensitivity_scales_all_axes() {
        let m = AxisMap {
            sensitivity: 2.0,
            ..Default::default()
        };
        let mut ypr = [10.0, -20.0, 30.0];
        m.apply(&mut ypr);
        assert_eq!(ypr, [20.0, -40.0, 60.0]);
    }
}
