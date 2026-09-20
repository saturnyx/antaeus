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
    /// Hysteresis bandwidth
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

    fn set_target(&mut self, target: f64) -> Result<(), Self::Error> {
        self.target = target;
        Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    fn bb() -> BangBang { BangBang::new(10.0, 50.0, 2.0) }

    #[test]
    fn initial_output_is_zero() {
        let mut b = bb();
        assert_eq!(b.tick(50.0, 0.0).unwrap(), 0.0);
    }

    #[test]
    fn switches_and_holds_inside_band() {
        let mut b = bb();
        assert_eq!(b.tick(47.0, 0.0).unwrap(), 10.0); // latch +max
        assert_eq!(b.tick(49.0, 0.0).unwrap(), 10.0); // still +max inside band
        assert_eq!(b.tick(53.0, 0.0).unwrap(), -10.0); // flip to -max
        assert_eq!(b.tick(51.0, 0.0).unwrap(), -10.0); // still -max inside band
    }

    #[test]
    fn reset_clears_state() {
        let mut b = bb();
        b.tick(47.0, 0.0).unwrap();
        b.reset().unwrap();
        assert_eq!(b.tick(50.0, 0.0).unwrap(), 0.0);
    }
}
