//! Drivetrain PID Control
//! A PID implementation used for controlling the drivetrain.
//!
//! This module provides [`DrivePID`], a dual-loop PID controller for a
//! differential drivetrain. It maintains independent left/right PID state
//! and converts motor angle feedback into linear wheel travel.
//!
//! # Features
//! - Left and right PID loops (`CorePID`) with shared tuning or custom instances
//! - Relative and absolute target APIs
//! - Automatic tick loop with timeout via [`DrivePID::autotick`]
//! - Gear-ratio + wheel-diameter based distance conversion
//!
//! # Units and Conversions
//! - Targets/tolerance are expressed as [`Length`](crate::misc::units::Length)
//! - Internal PID values are currently computed in inches (`f64`)
//! - Encoder/motor position is converted using:
//!   - motor-to-wheel ratio
//!   - wheel radius
//!   - arc-length relation `s = r * θ`
//!
//! # Notes
//! - Call `tick()` at a stable interval for best derivative behavior.
//! - Reset integral / derivative state when changing targets.
//! - Validate gear inputs (non-zero gear teeth) to avoid invalid ratios.
//!
//! # Example
//! ```
#![doc = include_str!("../../../examples/drive_pid.rs")]
//! ```

use std::{num::NonZeroU32, time::Duration};

use snafu::Snafu;
use vexide::{
    math::Angle,
    smart::imu::{InertialError, InertialSensor},
    time::user_uptime,
};

use crate::{
    motion::{
        control::DriveControl,
        primitive::{Feedback, pid::Pid},
    },
    peripherals::drivetrain::Differential,
    utils::units::Length,
};

/// Outcome of [`DrivePID::autotick`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoTickOutcome {
    /// The controller reached both targets within tolerance before `timeout`.
    Completed,
    /// The controller did not settle before `timeout` elapsed.
    TimedOut,
}

/// Dual-loop PID controller for a differential drivetrain.
///
/// This type owns a drivetrain handle and two [`CorePID`] instances:
/// one for the left side and one for the right side.
pub struct DriveFeedbackControl<D: Differential, F: Feedback> {
    /// Differential drivetrain interface used to read positions and command voltages.
    pub drivetrain:        D,
    /// Left-side PID controller.
    pub feedback_left:     F,
    /// Right-side PID controller.
    pub feedback_right:    F,
    /// Physical wheel diameter used for angle-to-distance conversion.
    pub wheel_diameter:    Length,
    /// Motor rotations per wheel rotation.
    pub motor_wheel_ratio: f64,
    /// Track width
    pub track_width:       Length,
    /// Timestamp of the previous PID update.
    pub last_update:       Duration,
}

impl<D: Differential, F: Feedback> DriveFeedbackControl<D, F> {
    /// Creates a [`DrivePID`] from preconfigured left and right [`Feedback`] instances.
    ///
    /// Use this constructor when each side requires different gains or state.
    pub fn new(
        drivetrain: D,
        feedback_left: F,
        feedback: F,
        wheel_diameter: Length,
        motor_gear_teeth: NonZeroU32,
        wheel_gear_teeth: NonZeroU32,
        track_width: Length,
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

        Self {
            drivetrain,
            feedback_left,
            feedback_right: feedback,
            wheel_diameter,
            motor_wheel_ratio,
            track_width,
            last_update: user_uptime(),
        }
    }

    /// Advances both PID loops by one control step and applies output voltage.
    ///
    /// This method:
    /// - computes `dt` from [`user_uptime`]
    /// - reads left/right motor angles
    /// - converts angle to linear distance
    /// - evaluates each PID loop
    /// - writes side-specific drivetrain voltages
    pub fn tick(&mut self) -> Result<(), F::Error> {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let left_reading = self.drivetrain.left_position().value();
        let right_reading = self.drivetrain.right_position();
        let left_power = self.feedback_left.tick(
            arc_length(left_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
            dt,
        )?;
        let right_power = self.feedback_right.tick(
            arc_length(right_reading.value(), self.wheel_diameter, self.motor_wheel_ratio)
                .as_inches(),
            dt,
        )?;
        let _ = self.drivetrain.set_left_voltage(left_power); // TODO: Implement Errors
        let _ = self.drivetrain.set_right_voltage(right_power); // TODO: Implement Errors
        self.last_update = now;
        Ok(())
    }

    /// Sets targets relative to the drivetrain's current positions.
    ///
    /// `left` and `right` are interpreted as deltas from the current wheel travel.
    /// PID integral and derivative history are reset to avoid carry-over between goals.
    pub fn set_relative_target(&mut self, left: Length, right: Length) -> Result<(), F::Error> {
        self.feedback_left.set_target(
            left.as_inches() +
                arc_length(
                    self.drivetrain.left_position().value(),
                    self.wheel_diameter,
                    self.motor_wheel_ratio,
                )
                .as_inches(),
        )?;
        self.feedback_right.set_target(
            right.as_inches() +
                arc_length(
                    self.drivetrain.right_position().value(),
                    self.wheel_diameter,
                    self.motor_wheel_ratio,
                )
                .as_inches(),
        )?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Sets absolute left/right distance targets.
    ///
    /// Targets are stored internally in inches.
    /// PID integral and derivative history are reset to avoid carry-over between goals.
    pub fn set_target(&mut self, left: Length, right: Length) -> Result<(), F::Error> {
        self.feedback_left.set_target(left.as_inches())?;
        self.feedback_right.set_target(right.as_inches())?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Resets only the integral terms for both PID loops.
    pub fn reset(&mut self) -> Result<(), F::Error> {
        self.feedback_left.reset()?;
        self.feedback_right.reset()?;
        Ok(())
    }

    /// Repeatedly calls [`DrivePID::tick`] until both loops are inactive or timeout.
    ///
    /// The loop sleeps for 10ms between iterations. Returns:
    /// - [`AutoTickOutcome::Completed`] when both sides settle within tolerance
    /// - [`AutoTickOutcome::TimedOut`] when `timeout` elapses first
    ///
    /// On completion, drivetrain voltage is set to zero.
    pub async fn autotick(&mut self, timeout: Duration) -> Result<AutoTickOutcome, F::Error> {
        let start = user_uptime();
        while self.feedback_left.is_active(
            arc_length(
                self.drivetrain.left_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches(),
        ) || self.feedback_right.is_active(
            arc_length(
                self.drivetrain.right_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches(),
        ) {
            self.tick()?;
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
            if (user_uptime() - start) > timeout {
                return Ok(AutoTickOutcome::TimedOut);
            }
        }
        let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
        Ok(AutoTickOutcome::Completed)
    }
}

/// Drive Control Error type
#[derive(Snafu, Clone, Copy)]
pub enum DriveControlError<F: Feedback> {
    /// An error returned by the feedback controller.
    Feedback {
        /// Source of error
        source: F::Error,
    },

    /// Inertial sensor error
    #[snafu(transparent)]
    InertialError {
        /// Source of error
        source: InertialError,
    },
}

impl<F: Feedback> std::fmt::Debug for DriveControlError<F> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Feedback { source } => formatter
                .debug_struct("DriveControlError")
                .field("source", source)
                .finish(),
            Self::InertialError { source } => formatter
                .debug_struct("DriveControlError")
                .field("source", source)
                .finish(),
        }
    }
}

// impl<F: Feedback> std::fmt::Display for DriveControlError<F> {
//     fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         match self {
//             Self::Feedback { source } => source.fmt(formatter),
//             Self::InertialError { source } => source.fmt(formatter),
//         }
//     }
// }

// impl<F: Feedback> std::error::Error for DriveControlError<F> {
//     fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
//         match self {
//             Self::Feedback { source } => Some(source),
//             Self::InertialError { source } => Some(source),
//         }
//     }
// }

impl<D: Differential, F: Feedback> DriveControl for DriveFeedbackControl<D, F> {
    type Error = DriveControlError<F>;

    /// Drives both sides forward/backward by the same relative distance.
    ///
    /// This sets equal left/right relative targets, then runs [`DrivePID::autotick`]
    /// until completion or `timeout`.
    async fn travel(
        &mut self,
        target: Length,
        timeout: Duration,
    ) -> Result<(), DriveControlError<F>> {
        self.set_relative_target(target, target)
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.autotick(timeout)
            .await
            .map_err(|source| DriveControlError::Feedback { source })?;
        Ok(())
    }

    /// Rotates the drivetrain in place by commanding opposite wheel travel.
    ///
    /// Positive and negative `angle` values rotate in opposite directions.
    async fn rotate(
        &mut self,
        angle: Angle,
        timeout: Duration,
    ) -> Result<(), DriveControlError<F>> {
        let len = track_rad_rotate(angle, self.track_width);
        self.set_relative_target(len, -len)
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.autotick(timeout)
            .await
            .map_err(|source| DriveControlError::Feedback { source })?;
        Ok(())
    }

    /// Pivots the drivetrain about one side.
    ///
    /// For positive `angle`, the left side moves while the right side is held.
    /// For negative `angle`, the right side moves while the left side is held.
    /// A zero angle returns immediately.
    async fn pivot(&mut self, angle: Angle, timeout: Duration) -> Result<(), DriveControlError<F>> {
        let len = track_rad_pivot(angle, self.track_width);
        if angle.as_degrees() > 0.0 {
            self.set_relative_target(len, Length::zero())
                .map_err(|source| DriveControlError::Feedback { source })?;
        } else if angle.as_degrees() < 0.0 {
            self.set_relative_target(Length::zero(), len)
                .map_err(|source| DriveControlError::Feedback { source })?;
        } else {
            return Ok(());
        }
        self.autotick(timeout)
            .await
            .map_err(|source| DriveControlError::Feedback { source })?;
        Ok(())
    }

    /// IMU-assisted in-place rotation to an angular offset from the current heading.
    ///
    /// This method continuously recomputes the remaining heading error and updates
    /// wheel travel targets without resetting PID history each iteration.
    /// The command ends when heading error is within `angle_tolerance` or when
    /// `timeout` elapses.
    async fn imu_rotate(
        &mut self,
        angle: Angle,
        timeout: Duration,
        imu: &InertialSensor,
        angle_tolerance: Angle,
    ) -> Result<(), Self::Error> {
        let start_heading = imu.rotation()?;
        let target_heading = start_heading + angle;
        let start_time = user_uptime();

        // Reset controller state once at the beginning (not every loop).
        self.reset()
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
                return Ok(()); // TODO: Add snafu error handling
            }

            let current_heading = match imu.rotation() {
                Ok(a) => a,
                Err(_) => {
                    vexide::time::sleep(std::time::Duration::from_millis(10)).await;
                    continue;
                }
            };

            let heading_error = target_heading - current_heading;
            if heading_error.as_degrees().abs() <= angle_tolerance.as_degrees() {
                let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
                return Ok(()); // TODO: Add snafu error handling
            }

            // Convert remaining heading error to side travel.
            let len = track_rad_rotate(heading_error, self.track_width);

            // Current wheel travel (absolute, in inches).
            let left_now = arc_length(
                self.drivetrain.left_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches();
            let right_now = arc_length(
                self.drivetrain.right_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches();

            // Update targets WITHOUT resetting PID history each cycle.
            self.feedback_left
                .set_target(left_now + len.as_inches())
                .map_err(|source| DriveControlError::Feedback { source })?;
            self.feedback_right
                .set_target(right_now - len.as_inches())
                .map_err(|source| DriveControlError::Feedback { source })?;

            self.tick()
                .map_err(|source| DriveControlError::Feedback { source })?;
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    }

    /// IMU-assisted pivot turn to an angular offset from the current heading.
    ///
    /// The moving side is chosen from the sign of `angle`:
    /// positive moves the left side, negative moves the right side.
    /// The command ends when heading error is within `angle_tolerance` or when
    /// `timeout` elapses.
    async fn imu_pivot(
        &mut self,
        angle: Angle,
        timeout: Duration,
        imu: &InertialSensor,
        angle_tolerance: Angle,
    ) -> Result<(), Self::Error> {
        let start_heading = match imu.rotation() {
            Ok(a) => a,
            Err(_) => return Ok(()), // TODO: Add snafu error handling
        };
        let target_heading = start_heading + angle;
        let start_time = user_uptime();

        // Choose pivot side from commanded turn direction:
        // +angle => move left side, hold right side
        // -angle => move right side, hold left side
        let move_left = if angle.as_degrees() > 0.0 {
            true
        } else if angle.as_degrees() < 0.0 {
            false
        } else {
            let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
            return Ok(()); // TODO: Add snafu error handling
        };

        // Reset controller state once at the beginning.
        self.reset()
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
                return Ok(()); // TODO: Add snafu error handling
            }

            let current_heading = match imu.rotation() {
                Ok(a) => a,
                Err(_) => {
                    vexide::time::sleep(std::time::Duration::from_millis(10)).await;
                    continue;
                }
            };

            let heading_error = target_heading - current_heading;
            if heading_error.as_degrees().abs() <= angle_tolerance.as_degrees() {
                let _ = self.drivetrain.set_voltage(0.0); // TODO: Implement Errors
                return Ok(()); // TODO: Add snafu error handling
            }

            // Convert remaining heading error to travel needed for a pivot.
            let len = track_rad_pivot(heading_error, self.track_width);

            // Current wheel travel (absolute, in inches).
            let left_now = arc_length(
                self.drivetrain.left_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches();
            let right_now = arc_length(
                self.drivetrain.right_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches();

            // Update targets WITHOUT resetting PID history each cycle.
            // Keep one side fixed at its current position and move the other.
            if move_left {
                self.feedback_left
                    .set_target(left_now + len.as_inches())
                    .map_err(|source| DriveControlError::Feedback { source })?;
                self.feedback_right
                    .set_target(right_now)
                    .map_err(|source| DriveControlError::Feedback { source })?;
            } else {
                self.feedback_left
                    .set_target(left_now)
                    .map_err(|source| DriveControlError::Feedback { source })?;
                self.feedback_right
                    .set_target(right_now + len.as_inches())
                    .map_err(|source| DriveControlError::Feedback { source })?;
            }

            self.tick()
                .map_err(|source| DriveControlError::Feedback { source })?;
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    }
}

impl<D: Differential> DriveFeedbackControl<D, Pid> {
    /// A QoL function which directly creates PID instances within the DriveFeedbackControl
    pub fn pid(
        drivetrain: D,
        kp: f64,
        ki: f64,
        kd: f64,
        max: f64,
        wheel_diameter: Length,
        motor_gear_teeth: NonZeroU32,
        wheel_gear_teeth: NonZeroU32,
        track_width: Length,
        default_target: Length,
        tolerance: Length,
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

        Self {
            drivetrain,
            feedback_left: Pid::new(
                kp,
                ki,
                kd,
                default_target.as_inches(),
                max,
                tolerance.as_inches(),
            ),
            feedback_right: Pid::new(
                kp,
                ki,
                kd,
                default_target.as_inches(),
                max,
                tolerance.as_inches(),
            ),
            wheel_diameter,
            track_width,
            motor_wheel_ratio,
            last_update: user_uptime(),
        }
    }
}

/// Converts motor shaft angle to wheel travel distance.
///
/// Uses:
/// - `mw_ratio`: motor rotations per wheel rotation
/// - wheel arc length relation `s = r * θ`
fn arc_length(motor_angle: Angle, wheel_diameter: Length, mw_ratio: f64) -> Length {
    debug_assert!(mw_ratio > 0.0);

    let radius_in = wheel_diameter.as_inches() * 0.5;
    let wheel_angle_rad = motor_angle.as_radians() / mw_ratio;
    Length::from_inches(radius_in * wheel_angle_rad)
}

/// Computes motor-to-wheel rotation ratio.
///
/// Returns motor rotations per one wheel rotation.
fn gears_to_motor_wheel_ratio(motor_gear_teeth: NonZeroU32, wheel_gear_teeth: NonZeroU32) -> f64 {
    wheel_gear_teeth.get() as f64 / motor_gear_teeth.get() as f64
}

fn track_rad_rotate(angle: Angle, track_width: Length) -> Length {
    Length::from_inches((track_width.as_inches() / 2.0 * angle.as_radians()) / 2.0)
}

fn track_rad_pivot(angle: Angle, track_width: Length) -> Length {
    Length::from_inches((track_width.as_inches() * angle.as_radians()) / 2.0)
}
