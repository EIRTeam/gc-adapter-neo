use binread::BinRead;

/// The connection-time origin used to normalize a GameCube stick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StickCalibration {
    center: (u8, u8),
}

impl Default for StickCalibration {
    fn default() -> Self {
        Self { center: (128, 128) }
    }
}

impl StickCalibration {
    pub const RADIUS: f32 = 80.0;

    /// Capture the controller's current stick position as its origin.
    pub fn recenter(&mut self, center: (u8, u8)) {
        self.center = center;
    }

    pub fn center(&self) -> (u8, u8) {
        self.center
    }

    /// Convert a raw stick position using Melee's fixed radius-80 normalization.
    /// No deadzone is applied.
    pub fn normalize(&self, raw: (u8, u8)) -> (f32, f32) {
        let mut x = raw.0 as f32 - self.center.0 as f32;
        let mut y = raw.1 as f32 - self.center.1 as f32;
        let magnitude = (x * x + y * y).sqrt();

        if magnitude > Self::RADIUS {
            let scale = Self::RADIUS / magnitude;
            x *= scale;
            y *= scale;
        }

        (x / Self::RADIUS, y / Self::RADIUS)
    }
}

/// An unsigned axis, representing a centered value, such as a joystick axis.
#[derive(BinRead, Debug, Default, Clone, Copy)]
pub struct SignedAxis(u8);

impl SignedAxis {
    pub fn from_raw(val: u8) -> Self {
        Self(val)
    }

    pub fn raw(&self) -> u8 {
        self.0
    }

    /// Return axis as an f32 in the range of [-1.0, 1.0]
    ///
    /// **Note:** the gamecube doesn't have a true center point. To have the controller properly
    /// centered, use [`SignedAxis::float_centered`] and provide a centerpoint (typically pulled
    /// from when the controller is first registered or on recalibration).
    pub fn float(&self) -> f32 {
        ((self.0 as f32) - 127.5) / 127.5
    }

    /// Return axis as an f64 in the range of [-1.0, 1.0]
    ///
    /// **Note:** You likely do not want the additional precision.
    pub fn double(&self) -> f64 {
        ((self.0 as f64) - 127.5) / 127.5
    }

    /// Return axis as an `f32` in the range of [-1.0, 1.0] centered around a given raw value.
    /// It is recommended the provided center value is pulled on start and on recalibration on a
    /// per-axis basis as most controllers have slight variation in their center.
    ///
    /// **Note:** this assumes the axis can reach the full `0..=255` range, which real GameCube
    /// controllers almost never do, they max out well short of the edges. For
    /// accurate fixed-radius output, use [`StickCalibration::normalize`].
    pub fn float_centered(&self, center: u8) -> f32 {
        let center_offset = ((self.0 as i16) - (center as i16)) as f32;
        let scale = if self.0 >= center {
            (u8::MAX - center).max(1)
        } else {
            center.max(1)
        } as f32;

        (center_offset / scale).clamp(-1.0, 1.0)
    }

    /// Return axis as an `f64` in the range of [-1.0, 1.0] centered around a given raw value.
    /// It is recommended the provided center value is pulled on start and on recalibration on a
    /// per-axis basis as most controllers have slight variation in their center.
    ///
    /// **Note:** see the same caveat as [`float_centered`](SignedAxis::float_centered).
    pub fn double_centered(&self, center: u8) -> f64 {
        let center_offset = ((self.0 as i16) - (center as i16)) as f64;
        let scale = if self.0 >= center {
            (u8::MAX - center).max(1)
        } else {
            center.max(1)
        } as f64;

        (center_offset / scale).clamp(-1.0, 1.0)
    }
}

/// An unsigned axis, representing a positive or zero value.
#[derive(BinRead, Debug, Default, Clone, Copy)]
pub struct UnsignedAxis(u8);

impl UnsignedAxis {
    pub fn from_raw(val: u8) -> Self {
        Self(val)
    }

    pub fn raw(&self) -> u8 {
        self.0
    }

    /// Return axis as an `f32` in the range of [0.0, 1.0]
    pub fn float(&self) -> f32 {
        (self.0 as f32) / 255.0
    }

    /// Return axis as an `f64` in the range of [0.0, 1.0]
    ///
    /// **Note:** You likely do not want the additional precision.
    pub fn double(&self) -> f64 {
        (self.0 as f64) / 255.0
    }
}

#[cfg(test)]
mod tests {
    use super::StickCalibration;

    #[test]
    fn fixed_radius_normalization_has_no_deadzone() {
        let calibration = StickCalibration::default();

        assert_eq!(calibration.normalize((128, 128)), (0.0, 0.0));
        assert_eq!(calibration.normalize((129, 128)), (1.0 / 80.0, 0.0));
        assert_eq!(calibration.normalize((208, 128)), (1.0, 0.0));
    }

    #[test]
    fn fixed_radius_normalization_clamps_magnitude() {
        let calibration = StickCalibration::default();
        let (x, y) = calibration.normalize((255, 255));

        assert!((x * x + y * y - 1.0).abs() < 0.000001);
        assert!((x - y).abs() < 0.000001);
    }

    //#[test]
    //fn test_signed() {
    //    assert_eq!(SignedAxis::from_raw(127).float(), 1.0);
    //    assert_eq!(SignedAxis::from_raw(-128).float(), -1.0);
    //    assert_eq!(SignedAxis::from_raw(0).float(), 0.0);
    //    assert_eq!(SignedAxis::from_raw(127).double(), 1.0);
    //    assert_eq!(SignedAxis::from_raw(-128).double(), -1.0);
    //    assert_eq!(SignedAxis::from_raw(0).double(), 0.0);
    //}

    //#[test]
    //fn test_inverted() {
    //    assert_eq!(InvertedSignedAxis::from_raw(127).float(), -1.0);
    //    assert_eq!(InvertedSignedAxis::from_raw(-128).float(), 1.0);
    //    assert_eq!(InvertedSignedAxis::from_raw(0).float(), 0.0);
    //    assert_eq!(InvertedSignedAxis::from_raw(127).double(), -1.0);
    //    assert_eq!(InvertedSignedAxis::from_raw(-128).double(), 1.0);
    //    assert_eq!(InvertedSignedAxis::from_raw(0).double(), 0.0);
    //}
    //
    //#[test]
    //fn test_unsigned() {
    //    assert_eq!(UnsignedAxis::from_raw(255).float(), 1.0);
    //    assert_eq!(UnsignedAxis::from_raw(0).float(), 0.0);
    //    assert_eq!(UnsignedAxis::from_raw(255).double(), 1.0);
    //    assert_eq!(UnsignedAxis::from_raw(0).double(), 0.0);
    //}
}
