//! # Bang-Bang Feedback Control Algorithm
//! Mostly for niche or educational purposes. It is recommended to use PID in
//! most cases.

use std::convert::Infallible;

use crate::prelude::primitive::Feedback;

/// Bang-Bang Algorithm with hysteresis
pub struct BangBang {
    /// Maximum output amplitude
    pub max:         f64,
    /// Target setpoint
    pub target:      f64,
    /// Hysteresis band width
    pub tolerance:   f64,
    /// Last output (held inside the band)
    pub last_output: f64,
}

impl BangBang {
    /// Create a new bang-bang instance
    pub fn new(max: f64, target: f64, tolerance: f64) -> Self {
        Self {
            max,
            target,
            tolerance,
            last_output: 0.0,
        }
    }

    /// Sets the target
    pub fn set_target(&mut self, target: f64) { self.target = target; }
}

impl Feedback for BangBang {
    type Error = Infallible;

    fn set_target(&mut self, target: f64) -> Result<(), Self::Error> {
        self.target = target;
        Ok(())
    }

    fn tick(&mut self, reading: f64, _: f64) -> Result<f64, Self::Error> {
        let error = self.target - reading;
        if error > self.tolerance {
            self.last_output = self.max;
        } else if error < -self.tolerance {
            self.last_output = -self.max;
        }
        // Inside the band: hold last_output (hysteresis)
        Ok(self.last_output)
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.last_output = 0.0;
        Ok(())
    }

    fn is_active(&self, reading: f64) -> bool {
        let error = self.target - reading;
        error.abs() > self.tolerance
    }
}
