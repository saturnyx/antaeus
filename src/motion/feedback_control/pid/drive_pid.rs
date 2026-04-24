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
use vexide::{math::Angle, time::user_uptime};

use super::core_pid::CorePID;
use crate::peripherals::drivetrain::Differential;

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
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

        Self {
            drivetrain,
            pid_left,
            pid_right,
            wheel_diameter,
            motor_wheel_ratio,
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
        let left_reading = self.drivetrain.left_position();
        let right_reading = self.drivetrain.right_position();
        let left_power = self.pid_left.tick(
            arc_length(left_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
            dt,
        );
        let right_power = self.pid_right.tick(
            arc_length(right_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
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
                    self.drivetrain.left_position(),
                    self.wheel_diameter,
                    self.motor_wheel_ratio,
                )
                .as_inches(),
        );
        self.pid_right.set_target(
            right.as_inches() +
                arc_length(
                    self.drivetrain.right_position(),
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
                self.drivetrain.left_position(),
                self.wheel_diameter,
                self.motor_wheel_ratio,
            )
            .as_inches(),
        ) || self.pid_right.is_active(
            arc_length(
                self.drivetrain.right_position(),
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
