//! # vexide-motorgroup
//!
//! Missing `MotorGroup` from VEXCode or PROS? This is a simple implementation of a
//! `MotorGroup` for vexide which allows you to group motors together and control
//! them as one.
//!
//! ## Installation
//!
//! Add the following to your `Cargo.toml`:
//!
//! ```toml,ignore
//! [dependencies]
//! # ... other dependencies
//! vexide-motorgroup = "2.1.0"
//! ```
//!
//! Or if you prefer the command line:
//!
//! ```sh,ignore
//! cargo add vexide-motorgroup
//! ```
//!
//! ## Usage
//!
//! Normally, you would have to set each motor's target and other values
//! individually even if the motors were physically connected in a drivetrain or
//! similar, but with `MotorGroup`, you can control them as if they were one motor.
//!
//! Just create a `MotorGroup` with a `Vec` of `Motor`s and use the `MotorGroup`
//! methods just like you would with a `Motor`. It's that simple!
//!
//! ```rust,ignore
//! use std::time::Duration;
//!
//! use alloc::vec;
//! use vexide_motorgroup::*;
//!
//! use vexide::prelude::*;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     // Here's where the magic happens
//!     let mut motor_group = MotorGroup::new(vec![
//!         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
//!         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
//!     ]);
//!
//!     // Set the motor group's target to a voltage as if it were a motor
//!     motor_group.set_voltage(5.0).unwrap();
//!     sleep(Duration::from_secs(1)).await;
//!
//!     // Set the motor group's target to a position
//!     motor_group
//!         .set_position_target(Angle::from_degrees(90.0), 200)
//!         .unwrap();
//!     sleep(Duration::from_secs(1)).await;
//!
//!     // Set the motor group's target to a velocity
//!     motor_group.set_velocity(100).unwrap();
//!     sleep(Duration::from_secs(1)).await;
//!
//!     // Brake the motor group
//!     motor_group.brake(BrakeMode::Hold).unwrap();
//! }
//! ```
//!
//! ## Error handling
//!
//! ### Read errors
//!
//! For functions returning values and reading data (i.e., those taking a
//! read-only reference to self), upon encountering an error accessing any
//! motor, the result will be a MotorGroupError that contains all the errors
//! encountered during the operation. Using [`MotorGroupError::result`] will
//! return the average of all the results that were successfully read.
//!
//! ### Write errors
//!
//! vexide-motorgroup provides two different strategies for handling write
//! errors. Both of them will return an `Err` when any motor returns an error.
//!
//! 1. [`WriteErrorStrategy::Ignore`] (default): This strategy will ignore
//!    errors and continue writing to the other motors.
//! 2. [`WriteErrorStrategy::Stop`]: This strategy will stop writing to the
//!    other motors and return the error immediately.

extern crate alloc;
use alloc::vec::Vec;

use vexide::{
    math::Angle,
    prelude::{Direction, Gearset},
    smart::{
        PortError,
        motor::{BrakeMode, Motor, MotorControl, SetGearsetError},
    },
};

/// An error that occurs when controlling a motor group.
///
/// This error is returned when an individual motor in the group encounters an
/// error. The error contains a list of all the errors that occurred.
///
/// A MotorGroupError is guaranteed to have at least one error in it.
///
/// MotorGroupError also implements `Into<MotorError>`, which will return the
/// first error that occurred. This means that you can use the `?` operator
/// with a `MotorGroupError` to return a `MotorError` to a result.
#[derive(Debug)]
#[non_exhaustive]
pub struct MotorGroupError<E = PortError, T = ()> {
    pub errors: Vec<E>,
    pub result: Option<T>,
}

impl<E> MotorGroupError<E, ()> {
    /// Creates a new motor group error from a `Vec` of motor errors.
    ///
    /// # Panics
    ///
    /// Panics if the errors vector is empty.
    pub(crate) fn new(errors: Vec<E>) -> Self {
        assert!(!errors.is_empty(), "Cannot create a MotorGroupError with no errors");
        Self {
            errors,
            result: None,
        }
    }
}

impl<E, T> MotorGroupError<E, T> {
    pub(crate) fn with_result(errors: Vec<E>, result: T) -> Self {
        assert!(!errors.is_empty(), "Cannot create a MotorGroupError with no errors");
        Self {
            errors,
            result: Some(result),
        }
    }

    pub(crate) fn with_empty_result(errors: Vec<E>) -> Self {
        assert!(!errors.is_empty(), "Cannot create a MotorGroupError with no errors");
        Self {
            errors,
            result: None,
        }
    }

    /// Returns the result of the motor group error.
    ///
    /// For getters that return a result, this is the value that would be returned
    /// if there were no errors. It is usually an average of the available data.
    /// If all motors in the group return an error, this will be None.
    pub fn result(&self) -> &Option<T> { &self.result }

    /// The first error that occurred in the motor group.
    pub fn first(&self) -> &E { &self.errors[0] }
}

impl From<MotorGroupError<PortError>> for PortError {
    fn from(error: MotorGroupError<PortError>) -> Self { error.errors.into_iter().next().unwrap() }
}

impl From<MotorGroupError<SetGearsetError>> for SetGearsetError {
    fn from(error: MotorGroupError<SetGearsetError>) -> Self {
        error.errors.into_iter().next().unwrap()
    }
}

impl core::fmt::Display for MotorGroupError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "error(s) in MotorGroup: {:?}", self.errors)
    }
}

impl core::error::Error for MotorGroupError {}

/// The mode for handling errors when writing to a motor group.
///
/// This is used to determine how to handle errors when writing to a motor group.
/// "Writing" means doing things like setting a target, setting a voltage,
/// setting a gearset, etc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WriteErrorStrategy {
    /// Ignore errors and continue writing.
    ///
    /// This means that if a motor is, for example, unplugged, then writes to
    /// other plugged in motors will still be attempted. You should use this
    /// mode for most places where redundancy is practiced. Note that methods will
    /// still return an `Err` variant when an error occurs even if some writes
    /// succeed.
    ///
    /// This is the default mode.
    #[default]
    Ignore,
    /// Stop writing on the first error and return early.
    ///
    /// This means that if a motor encounters an error, no further writes will
    /// be attempted, and the error will be returned immediately. This is useful
    /// for debugging or when you want to ensure that all motors are in a valid
    /// state at all times (e.g. a subsystem should either 100% work or not work
    /// at all.)
    Stop,
}

/// A group of motors that can be controlled together.
///
/// This is a simple wrapper around a vector of motors, with methods to easily
/// control all motors in the group at once as if they were a single motor.
///
/// A motor group is guaranteed to have at least one motor in it.
#[derive(Debug)]
pub struct MotorGroup<M: AsRef<[Motor]> + AsMut<[Motor]> = Vec<Motor>> {
    pub(crate) motors:    M,
    write_error_strategy: WriteErrorStrategy,
}

type GetterResult<T> = Result<T, MotorGroupError<PortError, T>>;

impl<M: AsRef<[Motor]> + AsMut<[Motor]>> MotorGroup<M> {
    /// Creates a new motor group from a vector of motors.
    ///
    /// You can set the write handling mode afterwards by calling
    /// [`write_error_handling_mode`].
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     _ = motor_group.set_voltage(5.0);
    /// }
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if there are no motors in the vector.
    pub fn new(motors: M) -> Self {
        assert!(
            !motors.as_ref().is_empty(),
            "Cannot create a motor group with no motors"
        );
        Self {
            motors,
            write_error_strategy: WriteErrorStrategy::default(),
        }
    }

    /// Sets the write error handling strategy for the motor group.
    ///
    /// This determines how to handle errors when writing to the motor group
    /// using methods like [`set_target`], [`set_voltage`], etc.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    pub fn write_error_strategy(&mut self, mode: WriteErrorStrategy) -> &mut Self {
        self.write_error_strategy = mode;
        self
    }

    /// Sets the target that the motor group should attempt to reach.
    ///
    /// This could be a voltage, velocity, position, or even brake mode.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_target(MotorControl::Voltage(5.0));
    ///     sleep(Duration::from_secs(1)).await;
    ///     let _ = motor_group.set_target(MotorControl::Brake(BrakeMode::Hold));
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_target).
    pub fn set_target(&mut self, target: MotorControl) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_target(target) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's target to a given [`BrakeMode`].
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.brake(BrakeMode::Hold);
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.brake).
    pub fn brake(&mut self, mode: BrakeMode) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.brake(mode) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Spins the motor group at a target velocity.
    ///
    /// This velocity corresponds to different actual speeds in RPM depending on the gearset used for the motor.
    /// Velocity is held with an internal PID controller to ensure consistent speed, as opposed to setting the
    /// motor's voltage.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Spin a motor group at 100 RPM:
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_velocity(100);
    ///     sleep(Duration::from_secs(1)).await;
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_velocity).
    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_velocity(rpm) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's output voltage.
    ///
    /// This voltage value spans from -12 (fully spinning reverse) to +12 (fully spinning forwards) volts, and
    /// controls the raw output of the motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Give the motor group full power:
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_voltage(motor_group.max_voltage());
    /// }
    /// ```
    ///
    /// Drive the motor group based on a controller joystick:
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let controller = peripherals.primary_controller;
    ///     loop {
    ///         let controller_state = controller.state().unwrap_or_default();
    ///         let voltage = controller_state.left_stick.x() * motor_group.max_voltage();
    ///         motor_group.set_voltage(voltage).unwrap();
    ///     }
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_voltage).
    pub fn set_voltage(&mut self, volts: f64) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_voltage(volts) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets an absolute position target for the motor group to attempt to reach.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    ///
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     let _ = motor_group.set_position_target(Angle::from_degrees(90.0), 200);
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_position_target).
    pub fn set_position_target(
        &mut self,
        position: Angle,
        velocity: i32,
    ) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_position_target(position, velocity) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Changes the output velocity for a profiled movement (motor_move_absolute or motor_move_relative).
    ///
    /// This will have no effect if the motor group is not following a profiled movement.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///     // Set the motor group's target to a Angle so that changing the velocity isn't a noop.
    ///     let _ = motor_group.set_target(MotorControl::Angle(Angle::from_degrees(90.0), 200));
    ///     let _ = motor_group.set_profiled_velocity(100).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_profiled_velocity).
    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_profiled_velocity(velocity) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the gearset of an 11W motor group.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    /// - A [`MotorError::SetGearsetExp`] is returned if the motor is a 5.5W EXP Smart Motor, which has no swappable gearset.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // This must be a V5 motor group
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     // Set the motor group to use the red gearset
    ///     motor_group.set_gearset(Gearset::Red).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_gearset).
    pub fn set_gearset(
        &mut self,
        gearset: Gearset,
    ) -> Result<(), MotorGroupError<SetGearsetError>> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_gearset(gearset) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Returns `true` if the motor group has a 5.5W (EXP) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new_exp(peripherals.port_1, Direction::Forward),
    ///         Motor::new_exp(peripherals.port_2, Direction::Forward),
    ///     ]);
    ///     if motor_group.has_exp() {
    ///         println!("Motor group has a 5.5W EXP Smart Motor");
    ///     }
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_exp).
    pub fn has_exp(&self) -> bool { self.motors.as_ref().iter().any(|motor| motor.is_exp()) }

    /// Returns `true` if the motor group has an 11W (V5) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Red, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Red, Direction::Forward),
    ///     ]);
    ///     if motor_group.has_v5() {
    ///         println!("Motor group has an 11W V5 Smart Motor");
    ///     }
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_v5).
    pub fn has_v5(&self) -> bool { self.motors.as_ref().iter().any(|motor| motor.is_v5()) }

    /// Returns the maximum voltage for the motor group based off of its [motor type](Motor::motor_type).
    ///
    /// # Examples
    ///
    /// Run a motor group at max speed, agnostic of its type:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// fn run_motor_group_at_max_speed(motor_group: &mut MotorGroup) {
    ///     motor_group.set_voltage(motor_group.max_voltage()).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.max_voltage).
    pub fn max_voltage(&self) -> f64 {
        self.motors
            .as_ref()
            .iter()
            .map(|motor| motor.max_voltage())
            .reduce(f64::max)
            .unwrap()
    }

    /// Returns the average estimated angular velocity of motors in a motor group in rotations per minute (RPM).
    ///
    /// # Accuracy
    ///
    /// In some cases, this reported value may be noisy or innaccurate, especially for systems where accurate
    /// velocity control at high speeds is required (such as flywheels). If the accuracy of this value proves
    /// inadequate, you may opt to perform your own velocity calculations by differentiating [`Motor::position`]
    /// over the reported internal timestamp of the motor using [`Motor::timestamp`].
    ///
    /// > For more information about Smart motor velocity estimation, see [this article](https://sylvie.fyi/sylib/docs/db/d8e/md_module_writeups__velocity__estimation.html).
    ///
    /// # Note
    ///
    /// To get the current **target** velocity instead of the estimated velocity, use [`Motor::target`].
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Get the current velocity of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.velocity().unwrap());
    /// }
    /// ```
    ///
    /// Calculate acceleration of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     let mut last_velocity = motor_group.velocity().unwrap();
    ///     let mut start_time = Instant::now();
    ///     loop {
    ///         let velocity = motor_group.velocity().unwrap();
    ///         // Make sure we don't divide by zero
    ///         let elapsed = start_time.elapsed().as_secs_f64() + 0.001;
    ///
    ///         // Calculate acceleration
    ///         let acceleration = (velocity - last_velocity) / elapsed;
    ///         println!("Velocity: {:.2} RPM, Acceleration: {:.2} RPM/s", velocity, acceleration);
    ///
    ///         last_velocity = velocity;
    ///         start_time = Instant::now();
    ///    }
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.velocity).
    pub fn velocity(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.velocity() {
                Ok(velocity) => {
                    sum += velocity;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the average power drawn by a motor in this the motor group in Watts.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the power drawn by a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.power().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.power).
    pub fn power(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.power() {
                Ok(power) => {
                    sum += power;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the average torque of motors in the motor group in Newton-meters.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the torque of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.torque().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.torque).
    pub fn torque(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.torque() {
                Ok(torque) => {
                    sum += torque;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the motor group's output voltage.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the voltage of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.voltage().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.voltage).
    pub fn voltage(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.voltage() {
                Ok(voltage) => {
                    sum += voltage;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the motor group's average position in ticks.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the position of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.position().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.position).
    pub fn position(&self) -> GetterResult<Angle> {
        let mut errors = Vec::new();
        let mut sum = Angle::ZERO;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.position() {
                Ok(position) => {
                    sum += position;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the motor group's average current in Amperes.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the current of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.current().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.current).
    pub fn current(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.current() {
                Ok(current) => {
                    sum += current;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns the motor group's average efficiency as a percentage.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the efficiency of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.efficiency().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.efficiency).
    pub fn efficiency(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.efficiency() {
                Ok(efficiency) => {
                    sum += efficiency;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Resets every motor in the motor group's position to zero.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Reset the position of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     motor_group.reset_position().unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.reset_position).
    pub fn reset_position(&mut self) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.reset_position() {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's position to a given value.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the position of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     motor_group.set_position(Angle::from_degrees(90.0)).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_position).
    pub fn set_position(&mut self, position: Angle) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_position(position) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's current limit in Amperes.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the current limit of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     motor_group.set_current_limit(2.5).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_current_limit).
    pub fn set_current_limit(&mut self, limit: f64) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_current_limit(limit) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's voltage limit in Volts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the voltage limit of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     motor_group.set_voltage_limit(10.0).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_voltage_limit).
    pub fn set_voltage_limit(&mut self, limit: f64) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_voltage_limit(limit) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Returns the motor group's temperature in degrees Celsius.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor in the group encounters an error.
    ///
    /// # Examples
    ///
    /// Print the temperature of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.temperature().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.temperature).
    pub fn temperature(&self) -> GetterResult<f64> {
        let mut errors = Vec::new();
        let mut sum = 0.0;
        let mut count = 0;
        for motor in self.motors.as_ref() {
            match motor.temperature() {
                Ok(temperature) => {
                    sum += temperature;
                    count += 1;
                }
                Err(error) => errors.push(error),
            }
        }
        if errors.is_empty() {
            Ok(sum / count as f64)
        } else if count > 0 {
            Err(MotorGroupError::with_result(errors, sum / count as f64))
        } else {
            Err(MotorGroupError::with_empty_result(errors))
        }
    }

    /// Returns `true` if any motor in the motor group is over temperature.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor encounters an error.
    ///
    /// Note that this method will still return `Ok` if a motor encounters an
    /// error but a motor in a group is still over temperature.
    ///
    /// # Examples
    ///
    /// Check if a motor group is over temperature:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.is_over_temperature().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_over_temperature).
    pub fn is_over_temperature(&self) -> Result<bool, MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_ref() {
            match motor.is_over_temperature() {
                Ok(true) => return Ok(true),
                Err(error) => errors.push(error),
                _ => {}
            }
        }
        if errors.is_empty() {
            Ok(false)
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Returns `true` if any motor in the motor group is over current.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor encounters an error.
    ///
    /// Note that this method will still return `Ok` if a motor encounters an
    /// error but a motor in a group is still over current.
    ///
    /// # Examples
    ///
    /// Check if a motor group is over current:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.is_over_current().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_over_current).
    pub fn is_over_current(&self) -> Result<bool, MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_ref() {
            match motor.is_over_current() {
                Ok(true) => return Ok(true),
                Err(error) => errors.push(error),
                _ => {}
            }
        }
        if errors.is_empty() {
            Ok(false)
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Returns `true` if any motor in the motor group has a driver fault.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor encounters an error.
    ///
    /// Note that this method will still return `Ok` if a motor encounters an
    /// error but a motor in a group has a driver fault.
    ///
    /// # Examples
    ///
    /// Check if a motor group has a driver fault:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.is_driver_fault().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_driver_fault).
    pub fn is_driver_fault(&self) -> Result<bool, MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_ref() {
            match motor.is_driver_fault() {
                Ok(true) => return Ok(true),
                Err(error) => errors.push(error),
                _ => {}
            }
        }
        if errors.is_empty() {
            Ok(false)
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Returns `true` if the any motor in the motor group is over current.
    ///
    /// # Errors
    ///
    /// - A [`MotorGroupError`] error is returned if any motor encounters an error.
    ///
    /// Note that this method will still return `Ok` if a motor encounters an
    /// error but a motor in a group is still over current.
    ///
    /// # Examples
    ///
    /// Check if a motor group is over current:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     println!("{:?}", motor_group.is_driver_over_current().unwrap());
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.is_driver_over_current).
    pub fn is_driver_over_current(&self) -> Result<bool, MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_ref() {
            match motor.is_driver_over_current() {
                Ok(true) => return Ok(true),
                Err(error) => errors.push(error),
                _ => {}
            }
        }
        if errors.is_empty() {
            Ok(false)
        } else {
            Err(MotorGroupError::new(errors))
        }
    }

    /// Sets the motor group's direction.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the direction of a motor group:
    /// ```rust,ignore
    /// use vexide::prelude::*;
    /// use vexide_motorgroup::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor_group = MotorGroup::new(vec![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ]);
    ///
    ///     motor_group.set_direction(Direction::Reverse).unwrap();
    /// }
    /// ```
    ///
    /// See the original method [here](https://docs.rs/vexide/latest/vexide/devices/smart/struct.Motor.html#method.set_direction).
    pub fn set_direction(&mut self, direction: Direction) -> Result<(), MotorGroupError> {
        let mut errors = Vec::new();
        for motor in self.motors.as_mut() {
            if let Err(error) = motor.set_direction(direction) {
                errors.push(error);
                if self.write_error_strategy == WriteErrorStrategy::Stop {
                    break;
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(MotorGroupError::new(errors))
        }
    }
}

// Unit tests for `lib.rs` covering MotorGroupError helpers and error strategy
// These tests avoid hardware-specific APIs and focus on pure-data helpers

#[cfg(test)]
mod tests {

    use super::{MotorGroupError, WriteErrorStrategy};

    #[derive(Debug, PartialEq, Eq, Clone)]
    struct FakeErr(&'static str);

    #[test]
    fn motor_group_error_new_panics_on_empty() {
        // MotorGroupError::new should panic when given an empty errors vector
        let v: Vec<FakeErr> = vec![];
        let res = std::panic::catch_unwind(|| MotorGroupError::new(v));
        assert!(
            res.is_err(),
            "expected MotorGroupError::new to panic on empty errors vector"
        );
    }

    #[test]
    fn motor_group_error_with_result_and_first() {
        let errors = vec![FakeErr("a"), FakeErr("b")];

        // with_result is crate-private but available to tests in the same crate
        let mg_err = MotorGroupError::with_result(errors.clone(), 42u32);

        // first() returns a reference to the first error
        assert_eq!(mg_err.first(), &errors[0]);

        // result() should return the provided result
        assert_eq!(mg_err.result(), &Some(42u32));

        // errors vector should be preserved
        assert_eq!(mg_err.errors.len(), 2);
    }

    #[test]
    fn motor_group_error_with_empty_result() {
        let errors = vec![FakeErr("x")];

        let mg_err: MotorGroupError<FakeErr, ()> =
            MotorGroupError::with_empty_result(errors.clone());

        // No result available when using with_empty_result
        assert_eq!(mg_err.result(), &None);

        // Error list is present and first() works
        assert_eq!(mg_err.errors.len(), 1);
        assert_eq!(mg_err.first(), &errors[0]);
    }

    #[test]
    fn write_error_strategy_default_is_ignore() {
        // Default strategy should be Ignore
        assert_eq!(WriteErrorStrategy::default(), WriteErrorStrategy::Ignore);
    }
}
