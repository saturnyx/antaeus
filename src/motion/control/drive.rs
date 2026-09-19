//! Drivetrain Feedback Control
//! A Feedback Control implementation used for controlling the drivetrain.
//!
//! This module provides [`DriveFeedbackControl`], a dual-loop feedback controller for a
//! differential drivetrain. It maintains independent left/right states
//! and converts motor angle feedback into linear wheel travel.
//!
//! # Features
//! - Left and right feedback loops with shared tuning or custom instances
//! - Relative and absolute target APIs
//! - Automatic tick loop with timeout via [`DriveFeedbackControl::autotick`]
//! - Gear-ratio + wheel-diameter based distance conversion
//!
//! # Units and Conversions
//! - Targets/tolerance are expressed as [`Length`](crate::misc::units::Length)
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
    motion::control::DriveControl,
    peripherals::drivetrain::Differential,
    prelude::{
        AutoTickOutcome,
        DrivetrainError,
        primitive::{Feedback, pid::Pid},
    },
    utils::units::Length,
};

/// Dual-loop feedback controller for a differential drivetrain.
///
/// This type owns a drivetrain handle and two [`Feedback`] instances:
/// one for the left side and one for the right side.
pub struct DriveFeedbackControl<D: Differential, F: Feedback> {
    /// Differential drivetrain interface used to read positions and command voltages.
    pub drivetrain:        D,
    /// Left-side feedback controller.
    pub feedback_left:     F,
    /// Right-side feedback controller.
    pub feedback_right:    F,
    /// Physical wheel diameter used for angle-to-distance conversion.
    pub wheel_diameter:    Length,
    /// Motor rotations per wheel rotation.
    pub motor_wheel_ratio: f64,
    /// Track width
    pub track_width:       Length,
    /// Timestamp of the previous update.
    pub last_update:       Duration,
}

/// Drive Control Error type
#[derive(Snafu)]
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

    #[snafu(transparent)]
    /// A Error from the Drivetrain
    DrivetrainError {
        /// Source of error
        source: DrivetrainError,
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
            Self::DrivetrainError { source } => formatter
                .debug_struct("DriveControlError")
                .field("source", source)
                .finish(),
        }
    }
}

impl<D: Differential, F: Feedback> DriveFeedbackControl<D, F> {
    /// Creates a [`DriveFeedbackControl`] from preconfigured left and right [`Feedback`] instances.
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

    /// Advances both feedback loops by one control step and applies output voltage.
    ///
    /// This method:
    /// - computes `dt` from [`user_uptime`]
    /// - reads left/right motor angles
    /// - converts angle to linear distance
    /// - evaluates each feedback loop
    /// - writes side-specific drivetrain voltages
    pub fn tick(&mut self) -> Result<(), DriveControlError<F>> {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let left_reading = self.drivetrain.left_position().value();
        let right_reading = self.drivetrain.right_position();
        let left_power = self
            .feedback_left
            .tick(
                arc_length(left_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
                dt,
            )
            .map_err(|source| DriveControlError::Feedback { source })?;
        let right_power = self
            .feedback_right
            .tick(
                arc_length(right_reading.value(), self.wheel_diameter, self.motor_wheel_ratio)
                    .as_inches(),
                dt,
            )
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.drivetrain.set_left_voltage(left_power)?;
        self.drivetrain.set_right_voltage(right_power)?;
        self.last_update = now;
        Ok(())
    }

    /// Sets targets relative to the drivetrain's current positions.
    ///
    /// `left` and `right` are interpreted as deltas from the current wheel travel.
    /// Feedback control is reset to avoid carry-over between goals.
    pub fn set_relative_target(
        &mut self,
        left: Length,
        right: Length,
    ) -> Result<(), DriveControlError<F>> {
        self.feedback_left
            .set_target(
                left.as_inches() +
                    arc_length(
                        self.drivetrain.left_position().value(),
                        self.wheel_diameter,
                        self.motor_wheel_ratio,
                    )
                    .as_inches(),
            )
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.feedback_right
            .set_target(
                right.as_inches() +
                    arc_length(
                        self.drivetrain.right_position().value(),
                        self.wheel_diameter,
                        self.motor_wheel_ratio,
                    )
                    .as_inches(),
            )
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Sets absolute left/right distance targets.
    ///
    /// Targets are stored internally in inches.
    /// Feedback control is reset to avoid carry-over between goals.
    pub fn set_target(&mut self, left: Length, right: Length) -> Result<(), DriveControlError<F>> {
        self.feedback_left
            .set_target(left.as_inches())
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.feedback_right
            .set_target(right.as_inches())
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Feedback control is reset
    pub fn reset(&mut self) -> Result<(), DriveControlError<F>> {
        self.feedback_left
            .reset()
            .map_err(|source| DriveControlError::Feedback { source })?;
        self.feedback_right
            .reset()
            .map_err(|source| DriveControlError::Feedback { source })?;
        Ok(())
    }

    /// Repeatedly calls [`DriveFeedbackControl::tick`] until both loops are inactive or timeout.
    ///
    /// The loop sleeps for 10ms between iterations. Returns:
    /// - [`AutoTickOutcome::Completed`] when both sides settle within tolerance
    /// - [`AutoTickOutcome::TimedOut`] when `timeout` elapses first
    ///
    /// On completion, drivetrain voltage is set to zero.
    pub async fn autotick(
        &mut self,
        timeout: Duration,
    ) -> Result<AutoTickOutcome, DriveControlError<F>> {
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
        self.drivetrain.set_voltage(0.0)?;
        Ok(AutoTickOutcome::Completed)
    }
}

impl<D: Differential, F: Feedback> DriveControl for DriveFeedbackControl<D, F> {
    type Error = DriveControlError<F>;

    /// Drives both sides forward/backward by the same relative distance.
    ///
    /// This sets equal left/right relative targets, then runs [`DriveFeedbackControl::autotick`]
    /// until completion or `timeout`.
    async fn travel(
        &mut self,
        target: Length,
        timeout: Duration,
    ) -> Result<AutoTickOutcome, DriveControlError<F>> {
        self.set_relative_target(target, target)?;
        self.autotick(timeout).await
    }

    /// Rotates the drivetrain in place by commanding opposite wheel travel.
    ///
    /// Positive and negative `angle` values rotate in opposite directions.
    async fn rotate(
        &mut self,
        angle: Angle,
        timeout: Duration,
    ) -> Result<AutoTickOutcome, DriveControlError<F>> {
        let len = track_rad_rotate(angle, self.track_width);
        self.set_relative_target(len, -len)?;
        self.autotick(timeout).await
    }

    /// Pivots the drivetrain about one side.
    ///
    /// For positive `angle`, the left side moves while the right side is held.
    /// For negative `angle`, the right side moves while the left side is held.
    /// A zero angle returns immediately.
    async fn pivot(
        &mut self,
        angle: Angle,
        timeout: Duration,
    ) -> Result<AutoTickOutcome, DriveControlError<F>> {
        let len = track_rad_pivot(angle, self.track_width);
        if angle.as_degrees() > 0.0 {
            self.set_relative_target(len, Length::zero())?;
        } else if angle.as_degrees() < 0.0 {
            self.set_relative_target(Length::zero(), len)?;
        } else {
            return Ok(AutoTickOutcome::Completed);
        }
        self.autotick(timeout).await
    }

    /// IMU-assisted in-place rotation to an angular offset from the current heading.
    ///
    /// This method continuously recomputes the remaining heading error and updates
    /// wheel travel targets without resetting feedback history each iteration.
    /// The command ends when heading error is within `angle_tolerance` or when
    /// `timeout` elapses.
    async fn imu_rotate(
        &mut self,
        angle: Angle,
        timeout: Duration,
        imu: &InertialSensor,
        angle_tolerance: Angle,
    ) -> Result<AutoTickOutcome, Self::Error> {
        let start_heading = imu.rotation()?;
        let target_heading = start_heading + angle;
        let start_time = user_uptime();

        // Reset controller state once at the beginning (not every loop).
        self.reset()?;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                self.drivetrain.set_voltage(0.0)?;
                return Ok(AutoTickOutcome::TimedOut);
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
                self.drivetrain.set_voltage(0.0)?;
                return Ok(AutoTickOutcome::Completed);
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

            // Update targets WITHOUT resetting feedback history each cycle.
            self.feedback_left
                .set_target(left_now + len.as_inches())
                .map_err(|source| DriveControlError::Feedback { source })?;
            self.feedback_right
                .set_target(right_now - len.as_inches())
                .map_err(|source| DriveControlError::Feedback { source })?;

            self.tick()?;
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
    ) -> Result<AutoTickOutcome, Self::Error> {
        let start_heading = imu.rotation()?;
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
            self.drivetrain.set_voltage(0.0)?;
            return Ok(AutoTickOutcome::Completed);
        };

        // Reset controller state once at the beginning.
        self.reset()?;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                self.drivetrain.set_voltage(0.0)?;
                return Ok(AutoTickOutcome::TimedOut);
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
                self.drivetrain.set_voltage(0.0)?;
                return Ok(AutoTickOutcome::Completed);
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

            // Update targets WITHOUT resetting feedback history each cycle.
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

            self.tick()?;
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
