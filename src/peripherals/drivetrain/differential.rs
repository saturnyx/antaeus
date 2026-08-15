//! Differential drivetrain control.
//!
//! This module defines [`Differential`], a small helper for controlling a
//! left/right ("tank" / "differential") drivetrain.
//!
//! The API is intentionally lightweight:
//! - **Driver control** helpers (`tank`, `arcade`, and their reversed variants)
//!   read a [`Controller`] and set motor voltages.
//! - **Utility** helpers (`set_voltage`, `set_brakemode`, encoder position
//!   queries, etc.) are useful for autonomous code.
//!
//! ## Supported drive modes
//!
//! - **Tank**: each joystick directly controls one side of the drivetrain.
//! - **Arcade**: forward/back + turn are mixed into left/right output.
//! - **Reverse Tank/Arcade**: inverted mappings that feel natural when the
//!   robot is driving "backwards".
//!
//! ## Motor direction
//!
//! Motors on opposite sides of a drivetrain often must spin in opposite
//! directions for the robot to drive forward. Ensure you configure each
//! [`Motor`] with the correct direction when constructing the drivetrain.
//!
//! ## Error handling
//!
//! Most methods return a `Vec<DrivetrainError>` rather than failing fast.
//! This lets driver-control loops keep running even if one motor or controller
//! read fails; errors are also logged with [`log::warn`].
//!
//! ## Example
//!
//! ```ignore
//! use antaeus::peripherals::drivetrain::Differential;
//! use vexide::prelude::*;
//!
//! let drivetrain = Differential::new(
//!     [
//!         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
//!         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
//!     ],
//!     [
//!         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
//!         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
//!     ],
//! );
//!
//! // In your control loop:
//! let controller = Controller::new(ControllerId::Primary);
//! let _errors = drivetrain.tank(&controller);
//! ```

use std::{cell::RefCell, rc::Rc};

use vexide::{
    math::Angle,
    prelude::{Controller, Motor},
    smart::motor::BrakeMode,
};

use super::DrivetrainError;
use crate::{
    motion::localization::tracker::devices::{Trackable, TrackingSensorError},
    peripherals::drivetrain::{Differential, Drivable},
    utils::error::Report,
};

/// A left/right (“tank”) drivetrain controller.
///
/// `Differential` owns (or shares ownership of) two motor groups:
/// - [`Differential::left`]
/// - [`Differential::right`]
///
/// The groups are stored as `Rc<RefCell<dyn AsMut<[Motor]>>>` so you can share
/// them with other subsystems (PID, odometry, motion profiling) while still
/// being able to mutate motor state.
///
/// ## Expectations
///
/// - Each side should contain motors that are oriented consistently (all spin
///   “forward” together).
/// - It is normal for left vs right sides to use opposite motor directions.
/// - Controller inputs are assumed to be normalized to `[-1.0, 1.0]` and are
///   scaled to volts by multiplying by `12.0`.
///
/// ## Example
///
/// ```ignore
/// let drivetrain = Differential::new(
///     [motor_left_1, motor_left_2],
///     [motor_right_1, motor_right_2],
/// );
/// let _ = drivetrain.set_brakemode(BrakeMode::Brake);
/// ```
#[derive(Clone)]
#[allow(dead_code)]
pub struct StandardDifferential {
    /// The left motor group.
    ///
    /// Contains all motors on the left side of the drivetrain.
    /// These motors should be configured to spin in the same direction
    /// relative to each other.
    pub left: Rc<RefCell<dyn AsMut<[Motor]>>>,

    /// The right motor group.
    ///
    /// Contains all motors on the right side of the drivetrain.
    /// These motors should be configured to spin in the same direction
    /// relative to each other (typically opposite to the left side for
    /// forward movement).
    pub right: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl StandardDifferential {
    /// Creates a new drivetrain from left/right motor groups.
    ///
    /// This constructor takes ownership of the provided motor collections and
    /// stores them behind `Rc<RefCell<_>>` so they can be shared.
    ///
    ///
    /// # Arguments
    ///
    /// * `left` - An array of motors for the left side of the drivetrain.
    /// * `right` - An array of motors for the right side of the drivetrain.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use antaeus::peripherals::drivetrain::Differential;
    /// use vexide::prelude::*;
    ///
    /// let drivetrain = Differential::new(
    ///     [
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ],
    ///     [
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    /// ```
    pub fn new<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: L,
        right: R,
    ) -> Self {
        Self {
            left:  Rc::new(RefCell::new(left)),
            right: Rc::new(RefCell::new(right)),
        }
    }

    /// Creates a new drivetrain with shared ownership of the left/right motors.
    ///
    /// This constructor is useful when you need to share motor references
    /// with other systems (e.g., PID controllers or odometry).
    ///
    /// # Arguments
    ///
    /// * `left` - A reference-counted cell containing the left motor array.
    /// * `right` - A reference-counted cell containing the right motor array.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use antaeus::peripherals::drivetrain::Differential;
    /// use std::{cell::RefCell, rc::Rc};
    /// use vexide::prelude::*;
    ///
    /// let drivetrain = Differential::from_shared(
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ])),
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ])),
    /// );
    /// ```
    pub fn from_shared<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: Rc<RefCell<L>>,
        right: Rc<RefCell<R>>,
    ) -> Self {
        Self { left, right }
    }
}

impl Drivable for StandardDifferential {
    /// Controls a tank-style drivetrain using the input from a controller.
    ///
    /// In tank drive mode, each joystick directly controls one side of the
    /// drivetrain. The left stick Y-axis controls the left motors, and the
    /// right stick Y-axis controls the right motors.
    ///
    /// # Arguments
    ///
    /// * `controller` - The VEX controller to read input from.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use antaeus::peripherals::drivetrain::Differential;
    /// use vexide::prelude::*;
    ///
    /// let controller = Controller::new(ControllerId::Primary);
    /// let _ = drivetrain.tank(&controller);
    /// ```
    fn tank(&mut self, controller: &Controller) -> Result<(), DrivetrainError> {
        let state = controller.state()?;

        let left_power = state.left_stick.y();
        let right_power = state.right_stick.y();

        let left_voltage = left_power * 12.0;
        let right_voltage = right_power * 12.0;

        let mut left_motors = self.left.try_borrow_mut()?;
        let mut right_motors = self.right.try_borrow_mut()?;

        for motor in left_motors.as_mut() {
            motor.set_voltage(left_voltage)?;
        }

        for motor in right_motors.as_mut() {
            motor.set_voltage(right_voltage)?;
        }

        Ok(())
    }

    /// Drive the robot using arcade controls (single-stick forward/back + single-stick turn).
    ///
    /// Behavior:
    /// * Forward/backward is read from the left stick Y axis.
    /// * Turning is read from the right stick X axis.
    /// * The two values are mixed into left/right voltages as:
    ///   * left = (fwd + turn) * 12.0
    ///   * right = (fwd - turn) * 12.0
    ///
    /// Notes:
    /// * Inputs are assumed to be in the range [-1.0, 1.0] and are scaled to volts by 12.0.
    /// * Consider applying your own deadband before calling if small-stick noise is an issue.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// let _ = drivetrain.arcade(&controller);
    /// ```
    fn arcade(&mut self, controller: &Controller) -> Result<(), DrivetrainError> {
        let state = controller.state()?;

        let fwd = state.left_stick.y();
        let turn = state.right_stick.x();

        let left_voltage = (fwd + turn) * 12.0;
        let right_voltage = (fwd - turn) * 12.0;

        let mut left_motors = self.left.try_borrow_mut()?;
        let mut right_motors = self.right.try_borrow_mut()?;

        for motor in left_motors.as_mut() {
            motor.set_voltage(left_voltage)?;
        }

        for motor in right_motors.as_mut() {
            motor.set_voltage(right_voltage)?;
        }

        Ok(())
    }

    /// Drive the robot using reversed tank controls (sticks swapped and inverted).
    ///
    /// Behavior:
    /// * Left motors take input from the RIGHT stick Y axis, inverted.
    /// * Right motors take input from the LEFT stick Y axis, inverted.
    /// * Computation:
    ///   * left = (-right_y) * 12.0
    ///   * right = (-left_y) * 12.0
    /// * This is useful when the robot is driving backwards but you want the sticks
    ///   to maintain an intuitive left/right mapping relative to the robot's new front.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// let _ = drivetrain.reverse_tank(&controller);
    /// ```
    fn reverse_tank(&mut self, controller: &Controller) -> Result<(), DrivetrainError> {
        let state = controller.state()?;

        let left_voltage = (-state.right_stick.y()) * 12.0;
        let right_voltage = (-state.left_stick.y()) * 12.0;

        let mut left_motors = self.left.try_borrow_mut()?;
        let mut right_motors = self.right.try_borrow_mut()?;

        for motor in left_motors.as_mut() {
            motor.set_voltage(left_voltage)?;
        }

        for motor in right_motors.as_mut() {
            motor.set_voltage(right_voltage)?;
        }

        Ok(())
    }

    /// Drive the robot using reversed arcade controls (forward/turn both inverted).
    ///
    /// Behavior:
    /// * Forward/backward is read from the left stick Y axis, but inverted (fwd = -left_y).
    /// * Turning is read from the right stick X axis, also inverted (turn = -right_x).
    /// * Mixed into left/right voltages as:
    ///   * left = (fwd + turn) * 12.0
    ///   * right = (fwd - turn) * 12.0
    /// * This inversion preserves intuitive steering when the robot is driving backwards
    ///   (pushing the right stick right still causes a clockwise turn relative to the driver).
    ///
    /// Notes:
    /// * Inputs are assumed to be in the range [-1.0, 1.0] and are scaled to volts by 12.0.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// let _ = drivetrain.reverse_arcade(&controller);
    /// ```
    fn reverse_arcade(&mut self, controller: &Controller) -> Result<(), DrivetrainError> {
        let state = controller.state()?;

        let fwd = -state.left_stick.y();
        let turn = -state.right_stick.x();

        let left_voltage = (fwd + turn) * 12.0;
        let right_voltage = (fwd - turn) * 12.0;

        let mut left_motors = self.left.try_borrow_mut()?;
        let mut right_motors = self.right.try_borrow_mut()?;

        for motor in left_motors.as_mut() {
            motor.set_voltage(left_voltage)?;
        }

        for motor in right_motors.as_mut() {
            motor.set_voltage(right_voltage)?;
        }

        Ok(())
    }
}

impl Differential for StandardDifferential {
    /// Sets the brake mode for all motors in the drivetrain.
    ///
    /// The brake mode determines how motors behave when no voltage is applied:
    ///
    /// * [`BrakeMode::Coast`]: Motors spin freely.
    /// * [`BrakeMode::Brake`]: Motors actively resist rotation.
    /// * [`BrakeMode::Hold`]: Motors actively hold their position.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use vexide::smart::motor::BrakeMode;
    ///
    /// // Set motors to brake mode for better control
    /// let _ = drivetrain.set_brakemode(BrakeMode::Brake);
    /// ```
    fn set_brakemode(&self, brakemode: BrakeMode) -> Result<(), DrivetrainError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.brake(brakemode)?;
        }

        for motor in right.as_mut() {
            motor.brake(brakemode)?;
        }

        Ok(())
    }

    /// Returns the average encoder position of all motors (left + right).
    ///
    /// The position is read from each motor’s integrated encoder and averaged.
    /// The result is returned as an [`Angle`].
    ///
    /// ## Error behavior
    ///
    /// - If reading any motor fails, that motor contributes `0` to the sum and an
    ///   error is recorded/logged.
    /// - If a motor group cannot be borrowed, the group is skipped and an error is
    ///   recorded/logged.
    ///
    /// Notes:
    /// - The current implementation divides by the number of motors successfully
    ///   iterated. If *no* motors are available, the divisor becomes `0.0` and the
    ///   returned angle will be non-finite. If you need a stricter guarantee,
    ///   consider handling the `errors` vector and/or adding your own guard.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let (pos, errs) = drivetrain.position();
    /// for e in errs { println!("warn: {e}"); }
    /// println!("Drivetrain position: {} degrees", pos.as_degrees());
    /// ```
    fn position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        let mut errors = Vec::new();
        let left = self.left.try_borrow_mut();
        let right = self.right.try_borrow_mut();
        let mut angle: Angle = Angle::from_degrees(0.0);
        let mut denom: f64 = 0.0;
        match left {
            Ok(mut motors) => {
                for motor in motors.as_mut() {
                    angle += motor.position().unwrap_or_else(|e| {
                        let err = DrivetrainError::PortError { source: e };
                        errors.push(err);
                        denom -= 1.0;
                        Angle::ZERO
                    });
                    denom += 1.0;
                }
            }
            Err(e) => {
                let err = DrivetrainError::BorrowMutError { source: e };
                errors.push(err);
            }
        }

        match right {
            Ok(mut motors) => {
                for motor in motors.as_mut() {
                    angle += motor.position().unwrap_or_else(|e| {
                        let err = DrivetrainError::PortError { source: e };
                        errors.push(err);
                        denom -= 1.0;
                        Angle::ZERO
                    });
                    denom += 1.0;
                }
            }
            Err(e) => {
                let err = DrivetrainError::BorrowMutError { source: e };
                errors.push(err);
            }
        }
        match errors.is_empty() {
            true => Report::new(angle / denom),
            false => Report::from_parts(angle / denom, errors),
        }
    }

    /// Returns the average encoder position of all left motors.
    ///
    /// See [`Differential::position`] for notes on error behavior and averaging.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let (pos, _errs) = drivetrain.left_position();
    /// println!("Left position: {} degrees", pos.as_degrees());
    /// ```
    fn left_position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        let mut errors = Vec::new();
        let left = self.left.try_borrow_mut();
        let mut angle: Angle = Angle::from_degrees(0.0);
        let mut denom: f64 = 0.0;
        match left {
            Ok(mut motors) => {
                for motor in motors.as_mut() {
                    angle += motor.position().unwrap_or_else(|e| {
                        let err = DrivetrainError::PortError { source: e };
                        errors.push(err);
                        denom -= 1.0;
                        Angle::ZERO
                    });
                    denom += 1.0;
                }
            }
            Err(e) => {
                let err = DrivetrainError::BorrowMutError { source: e };
                errors.push(err);
            }
        }
        match errors.is_empty() {
            true => Report::new(angle / denom),
            false => Report::from_parts(angle / denom, errors),
        }
    }

    /// Returns the average encoder position of all right motors.
    ///
    /// See [`Differential::position`] for notes on error behavior and averaging.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let (pos, _errs) = drivetrain.right_position();
    /// println!("Right position: {} degrees", pos.as_degrees());
    /// ```
    fn right_position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        let mut errors = Vec::new();
        let right = self.right.try_borrow_mut();
        let mut angle: Angle = Angle::from_degrees(0.0);
        let mut denom: f64 = 0.0;
        match right {
            Ok(mut motors) => {
                for motor in motors.as_mut() {
                    angle += motor.position().unwrap_or_else(|e| {
                        let err = DrivetrainError::PortError { source: e };
                        errors.push(err);
                        denom -= 1.0;
                        Angle::ZERO
                    });
                    denom += 1.0;
                }
            }
            Err(e) => {
                let err = DrivetrainError::BorrowMutError { source: e };
                errors.push(err);
            }
        }
        match errors.is_empty() {
            true => Report::new(angle / denom),
            false => Report::from_parts(angle / denom, errors),
        }
    }

    /// Resets the integrated encoder position on all drivetrain motors.
    ///
    /// This will attempt to reset both left and right motor groups.
    fn reset_position(&self) -> Result<(), DrivetrainError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.reset_position()?;
        }

        for motor in right.as_mut() {
            motor.reset_position()?;
        }

        Ok(())
    }

    /// Sets the integrated encoder position on all drivetrain motors.
    fn set_position(&self, position: Angle) -> Result<(), DrivetrainError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.set_position(position)?
        }

        for motor in right.as_mut() {
            motor.set_position(position)?
        }

        Ok(())
    }

    /// Sets the same voltage on all drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        let mut left_motors = self.left.try_borrow_mut()?;
        let mut right_motors = self.right.try_borrow_mut()?;

        for motor in left_motors.as_mut() {
            motor.set_voltage(voltage)?;
        }

        for motor in right_motors.as_mut() {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }

    /// Sets the same voltage on all *left* drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_left_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        let mut left = self.left.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }

    /// Sets the same voltage on all *right* drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_right_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        let mut right = self.right.try_borrow_mut()?;

        for motor in right.as_mut() {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }
}

impl Trackable for StandardDifferential {
    fn track_position(
        &mut self,
    ) -> Result<Angle, crate::motion::localization::tracker::devices::TrackingSensorError> {
        let res = Self::position(self);
        if res.has_errors() {
            let (_, e) = res.into_parts();
            // Take ownership of the vector, then remove and return the first element
            let source_err = e.unwrap_or_default().remove(0);

            let err = TrackingSensorError::DrivetrainError { source: source_err };
            Err(err)
        } else {
            Ok(res.value())
        }
    }

    fn reset_track_position(
        &mut self,
    ) -> Result<(), crate::motion::localization::tracker::devices::TrackingSensorError> {
        match Self::reset_position(self) {
            Ok(_) => Ok(()),
            Err(e) => Err(TrackingSensorError::DrivetrainError { source: e }),
        }
    }

    fn set_track_position(
        &mut self,
        position: Angle,
    ) -> Result<(), crate::motion::localization::tracker::devices::TrackingSensorError> {
        match Self::set_position(self, position) {
            Ok(_) => Ok(()),
            Err(e) => Err(TrackingSensorError::DrivetrainError { source: e }),
        }
    }
}
