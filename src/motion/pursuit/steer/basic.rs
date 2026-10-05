//! Basic Steering Algorithm
//!
//! Provides a simple curvature-based controller that converts a lookahead
//! point (expressed in the robot frame) into left/right wheel voltages for a
//! differential drive.

use std::convert::Infallible;

use crate::{motion::pursuit::steer::ArcSteer, prelude::Differential, utils::units::Length};

/// A Basic Steering Algorithm that generates wheel velocities depending on a
/// point relative to the robot.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BasicSteer {
    /// A leeway at which the algorithm will end. Smaller tolerances mean more
    /// accuracy but more time spent.
    pub tolerance: Length,
}

impl BasicSteer {
    /// Create a new instance of [`BasicSteer`]
    /// - `track_width`: The width of the drivetrain
    /// - `tolerance`: A leeway at which the algorithm will end. Smaller tolerances mean more accuracy but more time spent.
    pub fn new(tolerance: Length) -> Self { Self { tolerance } }
}

impl ArcSteer for BasicSteer {
    type Error = Infallible;

    fn steer<D: Differential>(
        &mut self,
        x: Length,
        y: Length,
        lookahead: Length,
        drivetrain: &D,
    ) -> Result<((f64, f64), bool), Infallible> {
        // Your frame:
        // +x = right, +y = forward
        let x_in = x.as_inches();
        let y_in = y.as_inches();

        let track_w_in = drivetrain.get_track_width().as_inches();
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
        Ok((
            (dir * left * max_voltage, dir * right * max_voltage),
            dist > self.tolerance.as_inches(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        motion::pursuit::steer::{ArcSteer, basic::BasicSteer},
        prelude::differential::StandardDifferential,
        utils::units::{Length, assert_almost_eq},
    };

    #[test]
    fn basic_normal() {
        let mut basic_control = BasicSteer {
            tolerance: Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control
            .steer(
                Length::from_inches(15.0),
                Length::from_inches(15.0),
                Length::from_inches(8.0),
                &StandardDifferential::empty(),
            )
            .unwrap();
        assert_almost_eq(left, 12.0);
        assert_almost_eq(right, 5.14285);
    }
    #[test]
    fn basic_negative_x() {
        let mut basic_control = BasicSteer {
            tolerance: Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control
            .steer(
                Length::from_inches(-15.0),
                Length::from_inches(15.0),
                Length::from_inches(8.0),
                &StandardDifferential::empty(),
            )
            .unwrap();
        assert_almost_eq(left, 5.14285);
        assert_almost_eq(right, 12.0);
    }
    #[test]
    fn basic_negative_y() {
        let mut basic_control = BasicSteer {
            tolerance: Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control
            .steer(
                Length::from_inches(15.0),
                Length::from_inches(-15.0),
                Length::from_inches(8.0),
                &StandardDifferential::empty(),
            )
            .unwrap();
        assert_almost_eq(right, -5.14285);
        assert_almost_eq(left, -12.0);
    }
    #[test]
    fn basic_negative_both() {
        let mut basic_control = BasicSteer {
            tolerance: Length::from_inches(0.1),
        };
        let ((left, right), _) = basic_control
            .steer(
                Length::from_inches(-15.0),
                Length::from_inches(-15.0),
                Length::from_inches(8.0),
                &StandardDifferential::empty(),
            )
            .unwrap();
        assert_almost_eq(left, -5.14285);
        assert_almost_eq(right, -12.0);
    }
    #[test]
    fn tolerance_check() {
        let mut basic_control = BasicSteer {
            tolerance: Length::from_inches(5.1),
        };
        let ((..), running) = basic_control
            .steer(
                Length::from_inches(4.0),
                Length::from_inches(3.0),
                Length::from_inches(8.0),
                &StandardDifferential::empty(),
            )
            .unwrap();

        assert!(!running);
    }
}
