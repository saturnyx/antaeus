//! # Drivetrain PID Control
//! A PID implementation used for controlling the drivetrain.
//!
//! This module provides [`DrivePID`], a dual-loop PID controller for a
//! differential drivetrain. It maintains independent left/right PID state
//! and converts motor angle feedback into linear wheel travel.
//!
//! ## Features
//! - Left and right PID loops (`CorePID`) with shared tuning or custom instances
//! - Relative and absolute target APIs
//! - Automatic tick loop with timeout via [`DrivePID::autotick`]
//! - Gear-ratio + wheel-diameter based distance conversion
//!
//! ## Units and Conversions
//! - Targets/tolerance are expressed as [`measurements::Length`]
//! - Internal PID values are currently computed in inches (`f64`)
//! - Encoder/motor position is converted using:
//!   - motor-to-wheel ratio
//!   - wheel radius
//!   - arc-length relation `s = r * θ`
//!
//! ## Notes
//! - Call `tick()` at a stable interval for best derivative behavior.
//! - Reset integral / derivative state when changing targets.
//! - Validate gear inputs (non-zero gear teeth) to avoid invalid ratios.

use std::{num::NonZeroU32, time::Duration};

use measurements::Length;
use vexide::{math::Angle, smart::imu::InertialSensor, time::user_uptime};

use super::core_pid::CorePID;
use crate::{motion::feedback_control::DriveControl, peripherals::drivetrain::Differential};

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
pub struct DrivePID {
    /// Differential drivetrain interface used to read positions and command voltages.
    pub drivetrain:        Differential,
    /// Left-side PID controller.
    pub pid_left:          CorePID,
    /// Right-side PID controller.
    pub pid_right:         CorePID,
    /// Physical wheel diameter used for angle-to-distance conversion.
    pub wheel_diameter:    Length,
    /// Motor rotations per wheel rotation.
    pub motor_wheel_ratio: f64,
    /// Track width
    pub track_width:       Length,
    /// Timestamp of the previous PID update.
    pub last_update:       Duration,
}

impl DrivePID {
    /// Creates a new [`DrivePID`] with symmetric PID gains for both sides.
    ///
    /// The provided `default_target` and `tolerance` are converted to inches
    /// for internal PID calculations.
    ///
    /// `motor_gear_teeth` and `wheel_gear_teeth` are `NonZeroU32` to guarantee
    /// a valid, non-zero gear ratio at construction.
    pub fn new(
        drivetrain: Differential,
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
            pid_left: CorePID::new(
                kp,
                ki,
                kd,
                default_target.as_inches(),
                max,
                tolerance.as_inches(),
            ),
            pid_right: CorePID::new(
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

    /// Creates a [`DrivePID`] from preconfigured left and right [`CorePID`] instances.
    ///
    /// Use this constructor when each side requires different gains or state.
    pub fn from_basic_pid(
        drivetrain: Differential,
        pid_left: CorePID,
        pid_right: CorePID,
        wheel_diameter: Length,
        motor_gear_teeth: NonZeroU32,
        wheel_gear_teeth: NonZeroU32,
        track_width: Length,
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

        Self {
            drivetrain,
            pid_left,
            pid_right,
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
    pub fn tick(&mut self) {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let left_reading = self.drivetrain.left_position().value();
        let right_reading = self.drivetrain.right_position();
        let left_power = self.pid_left.tick(
            arc_length(left_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
            dt,
        );
        let right_power = self.pid_right.tick(
            arc_length(right_reading.value(), self.wheel_diameter, self.motor_wheel_ratio)
                .as_inches(),
            dt,
        );
        self.drivetrain.set_left_voltage(left_power);
        self.drivetrain.set_right_voltage(right_power);
        self.last_update = now;
    }

    /// Sets targets relative to the drivetrain's current positions.
    ///
    /// `left` and `right` are interpreted as deltas from the current wheel travel.
    /// PID integral and derivative history are reset to avoid carry-over between goals.
    pub fn set_relative_target(&mut self, left: Length, right: Length) {
        self.pid_left.set_target(
            left.as_inches() +
                arc_length(
                    self.drivetrain.left_position().value(),
                    self.wheel_diameter,
                    self.motor_wheel_ratio,
                )
                .as_inches(),
        );
        self.pid_right.set_target(
            right.as_inches() +
                arc_length(
                    self.drivetrain.right_position().value(),
                    self.wheel_diameter,
                    self.motor_wheel_ratio,
                )
                .as_inches(),
        );
        self.reset_integral();
        self.pid_left.prev_error = 0.0;
        self.pid_right.prev_error = 0.0;
        self.last_update = user_uptime();
    }

    /// Sets absolute left/right distance targets.
    ///
    /// Targets are stored internally in inches.
    /// PID integral and derivative history are reset to avoid carry-over between goals.
    pub fn set_target(&mut self, left: Length, right: Length) {
        self.pid_left.set_target(left.as_inches());
        self.pid_right.set_target(right.as_inches());
        self.reset_integral();
        self.pid_left.prev_error = 0.0;
        self.pid_right.prev_error = 0.0;
        self.last_update = user_uptime();
    }

    /// Resets only the integral terms for both PID loops.
    pub fn reset_integral(&mut self) {
        self.pid_left.reset_integral();
        self.pid_right.reset_integral();
    }

    /// Repeatedly calls [`DrivePID::tick`] until both loops are inactive or timeout.
    ///
    /// The loop sleeps for 10ms between iterations. Returns:
    /// - [`AutoTickOutcome::Completed`] when both sides settle within tolerance
    /// - [`AutoTickOutcome::TimedOut`] when `timeout` elapses first
    ///
    /// On completion, drivetrain voltage is set to zero.
    pub async fn autotick(&mut self, timeout: Duration) -> AutoTickOutcome {
        let start = user_uptime();
        while self.pid_left.is_active(
            arc_length(
                self.drivetrain.left_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches(),
        ) || self.pid_right.is_active(
            arc_length(
                self.drivetrain.right_position().value(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches(),
        ) {
            self.tick();
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
            if (user_uptime() - start) > timeout {
                return AutoTickOutcome::TimedOut;
            }
        }
        self.drivetrain.set_voltage(0.0);
        AutoTickOutcome::Completed
    }
}

impl DriveControl for DrivePID {
    /// Drives both sides forward/backward by the same relative distance.
    ///
    /// This sets equal left/right relative targets, then runs [`DrivePID::autotick`]
    /// until completion or `timeout`.
    async fn travel(&mut self, target: Length, timeout: Duration) {
        self.set_relative_target(target, target);
        self.autotick(timeout).await;
        // TODO: Add snafu error handling
    }

    /// Rotates the drivetrain in place by commanding opposite wheel travel.
    ///
    /// Positive and negative `angle` values rotate in opposite directions.
    async fn rotate(&mut self, angle: Angle, timeout: Duration) {
        let len = track_rad_rotate(angle, self.track_width);
        self.set_relative_target(len, Length::from_inches(0.0) - len);
        self.autotick(timeout).await;
        // TODO: Add snafu error handling
    }

    /// Pivots the drivetrain about one side.
    ///
    /// For positive `angle`, the left side moves while the right side is held.
    /// For negative `angle`, the right side moves while the left side is held.
    /// A zero angle returns immediately.
    async fn pivot(&mut self, angle: Angle, timeout: Duration) {
        let len = track_rad_pivot(angle, self.track_width);
        if angle.as_degrees() > 0.0 {
            self.set_relative_target(len, Length::from_inches(0.0));
        } else if angle.as_degrees() < 0.0 {
            self.set_relative_target(Length::from_inches(0.0), len);
        } else {
            return;
        }
        self.autotick(timeout).await;
        // TODO: Add snafu error handling
    }

    /// IMU-assisted in-place rotation to an angular offset from the current heading.
    ///
    /// This method continuously recomputes the remaining heading error and updates
    /// wheel travel targets without resetting PID history each iteration.
    /// The command ends when heading error is within `angle_tolerance` or when
    /// `timeout` elapses.
    async fn imu_rotate<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &InertialSensor,
        angle_tolerance: Angle,
    ) {
        let start_heading = match imu.rotation() {
            Ok(a) => a,
            Err(_) => return, // TODO: Add snafu error handling
        };
        let target_heading = start_heading + angle;
        let start_time = user_uptime();

        // Reset controller state once at the beginning (not every loop).
        self.reset_integral();
        self.pid_left.prev_error = 0.0;
        self.pid_right.prev_error = 0.0;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                self.drivetrain.set_voltage(0.0);
                return; // TODO: Add snafu error handling
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
                self.drivetrain.set_voltage(0.0);
                return; // TODO: Add snafu error handling
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
            self.pid_left.set_target(left_now + len.as_inches());
            self.pid_right.set_target(right_now - len.as_inches());

            self.tick();
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    }

    /// IMU-assisted pivot turn to an angular offset from the current heading.
    ///
    /// The moving side is chosen from the sign of `angle`:
    /// positive moves the left side, negative moves the right side.
    /// The command ends when heading error is within `angle_tolerance` or when
    /// `timeout` elapses.
    async fn imu_pivot<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &InertialSensor,
        angle_tolerance: Angle,
    ) {
        let start_heading = match imu.rotation() {
            Ok(a) => a,
            Err(_) => return, // TODO: Add snafu error handling
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
            self.drivetrain.set_voltage(0.0);
            return; // TODO: Add snafu error handling
        };

        // Reset controller state once at the beginning.
        self.reset_integral();
        self.pid_left.prev_error = 0.0;
        self.pid_right.prev_error = 0.0;
        self.last_update = user_uptime();

        loop {
            if (user_uptime() - start_time) > timeout {
                self.drivetrain.set_voltage(0.0);
                return; // TODO: Add snafu error handling
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
                self.drivetrain.set_voltage(0.0);
                return; // TODO: Add snafu error handling
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
                self.pid_left.set_target(left_now + len.as_inches());
                self.pid_right.set_target(right_now);
            } else {
                self.pid_left.set_target(left_now);
                self.pid_right.set_target(right_now + len.as_inches());
            }

            self.tick();
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
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
