//! # Group PID
//! This PID controls all motors using a single PID instance.

use std::{time::Duration, usize};

use vexide::{math::Angle, prelude::Motor, time::user_uptime};

use crate::{
    motion::feedback_control::pid::{core_pid::CorePID, drive_pid::AutoTickOutcome},
    utils::units::Length,
};

/// Group PID
/// Used for controlling a group of motors simultaneously
pub struct GroupPID<const N: usize> {
    /// Single `CorePID` instance controls all motors
    pub pid:         CorePID,
    /// An array of Motors that will be controlled
    pub motors:      [Motor; N],
    /// Update interval used for differentiation
    pub last_update: Duration,
}

impl<const N: usize> GroupPID<N> {
    /// Create a new instance using an array of motors and PID constants
    pub fn new(
        motors: [Motor; N],
        kp: f64,
        ki: f64,
        kd: f64,
        target: f64,
        max: f64,
        tolerance: f64,
    ) -> Self {
        Self {
            pid: CorePID::new(kp, ki, kd, target, max, tolerance),
            motors,
            last_update: Duration::ZERO,
        }
    }

    /// Create a `GroupPID` instance from an already existing `CorePID` instance
    /// by adding an array of motors
    pub fn from_core_pid(motors: [Motor; N], core_pid: CorePID) -> Self {
        Self {
            pid: core_pid,
            motors,
            last_update: Duration::ZERO,
        }
    }

    /// Updates the GroupPID instance by one tick. It is recommended to call this function once
    /// every loop cycle.
    pub fn tick(&mut self) {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let reading = get_mean_pos(&self.motors);
        let power = self.pid.tick(reading.as_radians(), dt);
        set_voltage_group(&mut self.motors, power);
        self.last_update = now;
    }

    /// Sets targets relative to the motors' current positions.
    pub fn set_relative_target(&mut self, target: Length) {
        self.pid.set_target(target.as_inches());
        self.reset_integral();
        self.pid.prev_error = 0.0;
        self.last_update = user_uptime();
    }

    /// Sets absolute targets
    pub fn set_target(&mut self, target: Length) {
        self.pid.set_target(target.as_inches());
        self.reset_integral();
        self.pid.prev_error = 0.0;
        self.last_update = user_uptime();
    }

    /// Resets only the integral terms
    pub fn reset_integral(&mut self) { self.pid.reset_integral(); }

    /// Repeatedly calls `GroupPID::tick` until both loops are inactive or timeout.
    pub async fn autotick(&mut self, timeout: Duration) -> AutoTickOutcome {
        let start = user_uptime();
        while self.pid.is_active(get_mean_pos(&self.motors).as_radians()) {
            self.tick();
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
            if (user_uptime() - start) > timeout {
                return AutoTickOutcome::TimedOut;
            }
        }
        let _ = set_voltage_group(&mut self.motors, 0.0);
        AutoTickOutcome::Completed
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
