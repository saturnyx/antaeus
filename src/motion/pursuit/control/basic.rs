use measurements::Length;

use crate::motion::pursuit::control::PursuitControl;

/// A Basic Control Algorithm that generates wheel velocities depending on a
/// point relative to the robot.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BasicControl {
    pub track_width: Length,
    pub tolerance:   Length,
}

impl BasicControl {
    /// Create a new instance of `Basic Control`
    /// - `track_width`: The width of the drivetrain
    /// - `tolerance`: A leeway at which the algorithm will end. Smaller
    /// tolerances mean more accuracy but more time spent.
    pub fn new(track_width: Length, tolerance: Length) -> Self {
        Self {
            track_width,
            tolerance,
        }
    }
}

impl PursuitControl for BasicControl {
    fn control(&self, x: Length, y: Length, lookahead: Length) -> ((f64, f64), bool) {
        // Your frame:
        // +x = right, +y = forward
        let x_in = x.as_inches();
        let y_in = y.as_inches();

        let track_w_in = self.track_width.as_inches();
        let lookahead_in = lookahead.as_inches();

        // Distance to the lookahead point
        let d2 = x_in * x_in + y_in * y_in;
        let dist = d2.sqrt();

        // Signed curvature kappa (1/in). For heading along +y:
        // kappa = 2x / (x^2 + y^2)
        let kappa = if d2 < 1e-12 { 0.0 } else { 2.0 * x_in / d2 };

        // Base speed scaling (same idea you already had)
        let speed_scale = if lookahead_in <= 1e-9 {
            1.0
        } else {
            (dist / lookahead_in).clamp(0.0, 1.0)
        };

        // Wheel mix from curvature
        let mut left = speed_scale * (2.0 + kappa * track_w_in) / 2.0;
        let mut right = speed_scale * (2.0 - kappa * track_w_in) / 2.0;

        // Normalize to <= 1.0 magnitude before converting to volts
        let mag = left.abs().max(right.abs());
        if mag > 1.0 {
            left /= mag;
            right /= mag;
        }

        let max_voltage = 12.0;
        let dir = y_in.signum();
        (
            (dir * left * max_voltage, dir * right * max_voltage),
            dist > self.tolerance.as_inches(),
        )
    }
}

#[cfg(test)]
mod tests {
    use measurements::{Length, test_utils::assert_almost_eq};

    use crate::motion::pursuit::control::{PursuitControl, basic::BasicControl};

    #[test]
    fn basic_normal() {
        let basic_control = BasicControl {
            track_width: Length::from_inches(12.0),
            tolerance:   Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control.control(
            Length::from_inches(15.0),
            Length::from_inches(15.0),
            Length::from_inches(8.0),
        );
        assert_almost_eq(left, 12.0);
        assert_almost_eq(right, 5.14285);
    }
    #[test]
    fn basic_negative_x() {
        let basic_control = BasicControl {
            track_width: Length::from_inches(12.0),
            tolerance:   Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control.control(
            Length::from_inches(-15.0),
            Length::from_inches(15.0),
            Length::from_inches(8.0),
        );
        assert_almost_eq(left, 5.14285);
        assert_almost_eq(right, 12.0);
    }
    #[test]
    fn basic_negative_y() {
        let basic_control = BasicControl {
            track_width: Length::from_inches(12.0),
            tolerance:   Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control.control(
            Length::from_inches(15.0),
            Length::from_inches(-15.0),
            Length::from_inches(8.0),
        );
        assert_almost_eq(right, -5.14285);
        assert_almost_eq(left, -12.0);
    }
    #[test]
    fn basic_negative_both() {
        let basic_control = BasicControl {
            track_width: Length::from_inches(12.0),
            tolerance:   Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control.control(
            Length::from_inches(-15.0),
            Length::from_inches(-15.0),
            Length::from_inches(8.0),
        );
        assert_almost_eq(left, -5.14285);
        assert_almost_eq(right, -12.0);
    }
    #[test]
    fn tolernace_check() {
        let basic_control = BasicControl {
            track_width: Length::from_inches(12.0),
            tolerance:   Length::from_inches(5.1),
        };
        let ((..), running) = basic_control.control(
            Length::from_inches(4.0),
            Length::from_inches(3.0),
            Length::from_inches(8.0),
        );

        assert!(!running);
    }
}
