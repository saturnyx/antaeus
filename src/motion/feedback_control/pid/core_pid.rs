//! # Core PID Algorithm
//! This is the core PID library. All PID instances will rely on this. It is
//! state machine based.

/// The Core PID instance
#[derive(Debug, Clone)]
pub struct CorePID {
    pub target:     f64,
    pub kp:         f64,
    pub ki:         f64,
    pub kd:         f64,
    /// Maximum Output
    pub max:        f64,
    pub prev_error: f64,
    pub integral:   f64,
    /// Leeway/Tolerance
    pub tolerance:  f64,
}

impl CorePID {
    /// Create a new core PID instance using:
    /// - Kp
    /// - Ki
    /// - Kd
    /// - target (point to reach)
    /// - max (maximum output)
    /// - tolerance (the leeway)
    pub fn new(kp: f64, ki: f64, kd: f64, target: f64, max: f64, tolerance: f64) -> Self {
        Self {
            target,
            kp,
            ki,
            kd,
            max,
            tolerance,
            prev_error: 0.0,
            integral: 0.0,
        }
    }

    /// Update the core PID by one tick
    pub fn tick(&mut self, reading: f64, mut dt: f64) -> f64 {
        dt = dt.max(1e-6);
        let error = self.target - reading;
        self.integral += error * dt;
        if self.ki != 0.0 {
            let integral_limit = self.max / self.ki.abs();
            self.integral = self.integral.clamp(-integral_limit, integral_limit);
        }
        let p = self.kp * error;
        let i = self.ki * self.integral;
        let d = self.kd * (error - self.prev_error) / dt;
        let output = (p + i + d).clamp(-self.max, self.max);
        self.prev_error = error;

        if error.abs() > self.tolerance {
            output
        } else {
            0.0
        }
    }

    /// Reset the integral term
    ///
    /// This has to be done every time the target changes
    pub fn reset_integral(&mut self) { self.integral = 0.0 }

    /// Sets the target
    ///
    /// Important: this function does not update the integral term
    /// It is recommended to reset the integral term along with the prev_error
    /// and last_update term.
    pub fn set_target(&mut self, target: f64) { self.target = target; }

    /// Returns whether the PID is active
    /// - `true`: The PID is active and the error is greater than the tolerance
    /// - `false`: The PID is inactive and the error is smaller than the tolerance
    pub fn is_active(&self, reading: f64) -> bool {
        let error = self.target - reading;
        error.abs() > self.tolerance
    }
}
