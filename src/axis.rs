use binread::BinRead;

/// Dolphin/GC-style automatic bound calibration based on minimum and maximum values reported
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisCalibration {
    center: u8,
    min: u8,
    max: u8,
}

impl Default for AxisCalibration {
    /// A reasonable starting point based on typical real GameCube controller hardware, used
    /// before any samples have been observed.
    fn default() -> Self {
        Self { center: 128, min: 38, max: 218 }
    }
}

impl AxisCalibration {
    /// Reset the calibration around a freshly observed center value, e.g. when a controller is
    /// newly connected. This tightens `min`/`max` back down to the new center; call
    /// [`observe`](AxisCalibration::observe) on subsequent readings to widen the range again as
    /// the stick moves.
    pub fn recenter(&mut self, raw: u8) {
        self.center = raw;
        self.min = raw;
        self.max = raw;
    }

    /// Widen the known range to include a newly observed raw sample. Never shrinks the range.
    pub fn observe(&mut self, raw: u8) {
        if raw < self.min {
            self.min = raw;
        }
        if raw > self.max {
            self.max = raw;
        }
    }

    pub fn center(&self) -> u8 {
        self.center
    }

    pub fn min(&self) -> u8 {
        self.min
    }

    pub fn max(&self) -> u8 {
        self.max
    }
}

/// An unsigned axis, representing a centered value, such as a joystick axis.
#[derive(BinRead, Debug, Default)]
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
    /// accurate full-range output, prefer [`float_calibrated`](SignedAxis::float_calibrated)
    /// with an [`AxisCalibration`] derived from observed hardware readings.
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

    /// Return axis as an `f32` in the range of [-1.0, 1.0], scaled using the observed hardware
    pub fn float_calibrated(&self, cal: &AxisCalibration) -> f32 {
        let raw = self.0 as f32;
        let center = cal.center as f32;

        if self.0 >= cal.center {
            let span = (cal.max as f32 - center).max(1.0);
            ((raw - center) / span).min(1.0)
        } else {
            let span = (center - cal.min as f32).max(1.0);
            ((raw - center) / span).max(-1.0)
        }
    }

    /// `f64` variant of [`float_calibrated`](SignedAxis::float_calibrated).
    pub fn double_calibrated(&self, cal: &AxisCalibration) -> f64 {
        let raw = self.0 as f64;
        let center = cal.center as f64;

        if self.0 >= cal.center {
            let span = (cal.max as f64 - center).max(1.0);
            ((raw - center) / span).min(1.0)
        } else {
            let span = (center - cal.min as f64).max(1.0);
            ((raw - center) / span).max(-1.0)
        }
    }
}

/// An unsigned axis, representing a positive or zero value.
#[derive(BinRead, Debug, Default)]
pub struct UnsignedAxis(u8);

impl UnsignedAxis {
    pub fn from_raw(val: u8) -> Self {
        Self(val)
    }

    pub fn raw(&self) -> u8 {
        self.0
    }

    /// Return axis as an `f32` in the range of [-1.0, 1.0]
    pub fn float(&self) -> f32 {
        (self.0 as f32) / 255.0
    }

    /// Return axis as an `f64` in the range of [-1.0, 1.0]
    ///
    /// **Note:** You likely do not want the additional precision.
    pub fn double(&self) -> f64 {
        (self.0 as f64) / 255.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
