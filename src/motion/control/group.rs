//! # Group Feedback Control
//! This feedcontroller controls all motors using a single feedback loop.
//!
//! # Example
//! ```
#![doc = include_str!("../../../examples/group_pid.rs")]
//! ```
use std::time::Duration;

use vexide::{math::Angle, prelude::Motor, time::user_uptime};

use super::AutoTickOutcome;
use crate::motion::primitive::{Feedback, pid::Pid};

/// Group Feedback Controller
/// Used for controlling a group of motors simultaneously
pub struct GroupFeedbackControl<const N: usize, F: Feedback> {
    /// Single feedback loop controls all motors
    pub feedback:    F,
    /// An array of Motors that will be controlled
    pub motors:      [Motor; N],
    /// Update interval used for differentiation
    pub last_update: Duration,
}

impl<const N: usize, F: Feedback> GroupFeedbackControl<N, F> {
    /// Create a `GroupFeedbackControl` instance from an already existing feedback instance
    /// by adding an array of motors. All values are taken in Radians.
    pub fn new(motors: [Motor; N], feedback: F) -> Self {
        Self {
            feedback,
            motors,
            last_update: Duration::ZERO,
        }
    }

    /// Updates the [`GroupFeedbackControl`] instance by one tick. It is recommended to call this function once
    /// every loop cycle.
    pub fn tick(&mut self) -> Result<(), F::Error> {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let reading = get_mean_pos(&self.motors);
        let power = self.feedback.tick(reading.as_radians(), dt)?;
        set_voltage_group(&mut self.motors, power);
        self.last_update = now;
        Ok(())
    }

    /// Sets targets relative to the motors' current positions.
    pub fn set_relative_target(&mut self, target: Angle) -> Result<(), F::Error> {
        self.feedback.set_target(target.as_radians())?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Sets absolute targets
    pub fn set_target(&mut self, target: Angle) -> Result<(), F::Error> {
        self.feedback.set_target(target.as_radians())?;
        self.reset()?;
        self.last_update = user_uptime();
        Ok(())
    }

    /// Resets only the integral terms
    pub fn reset(&mut self) -> Result<(), F::Error> { self.feedback.reset() }

    /// Repeatedly calls [`GroupFeedbackControl::tick`] until both loops are inactive or timeout.
    pub async fn autotick(&mut self, timeout: Duration) -> Result<AutoTickOutcome, F::Error> {
        let start = user_uptime();
        while self
            .feedback
            .is_active(get_mean_pos(&self.motors).as_radians())
        {
            self.tick()?;
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
            if (user_uptime() - start) > timeout {
                return Ok(AutoTickOutcome::TimedOut);
            }
        }
        set_voltage_group(&mut self.motors, 0.0);
        Ok(AutoTickOutcome::Completed)
    }
}

impl<const N: usize> GroupFeedbackControl<N, Pid> {
    /// Create a new instance using an array of motors and PID constants
    pub fn pid(
        motors: [Motor; N],
        kp: f64,
        ki: f64,
        kd: f64,
        target: Angle,
        max: f64,
        tolerance: Angle,
    ) -> Self {
        Self {
            feedback: Pid::new(kp, ki, kd, target.as_radians(), max, tolerance.as_radians()),
            motors,
            last_update: Duration::ZERO,
        }
    }
}

fn get_mean_pos<const N: usize>(motors: &[Motor; N]) -> Angle {
    let mut total_angle = Angle::ZERO;
    let mut count = 0.0;
    for motor in motors {
        if let Ok(pos) = motor.position() {
            total_angle += pos;
            count += 1.0;
        }
    }

    total_angle / count
}

fn set_voltage_group<const N: usize>(motors: &mut [Motor; N], volts: f64) {
    for motor in motors {
        let _ = motor.set_voltage(volts);
    }
}
