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
#[cfg(test)]
mod tests {
    use super::CorePID;

    fn approx_eq(a: f64, b: f64, eps: f64) {
        assert!(
            (a - b).abs() <= eps,
            "left={} right={} diff={} eps={}",
            a,
            b,
            (a - b).abs(),
            eps
        );
    }

    #[test]
    fn new_initializes_fields() {
        let pid = CorePID::new(1.2, 0.3, 0.4, 10.0, 100.0, 0.5);

        approx_eq(pid.kp, 1.2, 1e-12);
        approx_eq(pid.ki, 0.3, 1e-12);
        approx_eq(pid.kd, 0.4, 1e-12);
        approx_eq(pid.target, 10.0, 1e-12);
        approx_eq(pid.max, 100.0, 1e-12);
        approx_eq(pid.tolerance, 0.5, 1e-12);
        approx_eq(pid.prev_error, 0.0, 1e-12);
        approx_eq(pid.integral, 0.0, 1e-12);
    }

    #[test]
    fn tick_proportional_only() {
        // error = target - reading = 10 - 7 = 3
        // output = kp * error = 2 * 3 = 6
        let mut pid = CorePID::new(2.0, 0.0, 0.0, 10.0, 100.0, 0.0);
        let out = pid.tick(7.0, 0.1);

        approx_eq(out, 6.0, 1e-12);
        approx_eq(pid.prev_error, 3.0, 1e-12);
    }

    #[test]
    fn tick_output_is_zero_inside_tolerance() {
        let mut pid = CorePID::new(10.0, 0.0, 0.0, 10.0, 100.0, 0.5);
        // error = 0.3 => inside tolerance => output forced to 0
        let out = pid.tick(9.7, 0.1);

        approx_eq(out, 0.0, 1e-12);
        assert!(!pid.is_active(9.7));
    }

    #[test]
    fn tick_output_is_clamped_by_max() {
        // raw P output = 100 * 10 = 1000, but max is 5
        let mut pid = CorePID::new(100.0, 0.0, 0.0, 10.0, 5.0, 0.0);
        let out = pid.tick(0.0, 0.1);

        approx_eq(out, 5.0, 1e-12);
    }

    #[test]
    fn integral_is_clamped_to_prevent_windup() {
        // integral limit = max / |ki| = 10 / 2 = 5
        // without clamp, integral would become 10 * 10 = 100
        let mut pid = CorePID::new(0.0, 2.0, 0.0, 10.0, 10.0, 0.0);
        let out = pid.tick(0.0, 10.0);

        approx_eq(pid.integral, 5.0, 1e-12);
        // i term = ki * integral = 2 * 5 = 10; clamped output remains 10
        approx_eq(out, 10.0, 1e-12);
    }

    #[test]
    fn derivative_uses_dt_with_lower_bound() {
        // dt is clamped to 1e-6; derivative can become huge but output still clamped by max
        let mut pid = CorePID::new(0.0, 0.0, 1.0, 10.0, 7.0, 0.0);
        let out = pid.tick(0.0, 0.0);

        approx_eq(out, 7.0, 1e-12);
    }

    #[test]
    fn reset_integral_and_set_target_work() {
        let mut pid = CorePID::new(0.0, 1.0, 0.0, 10.0, 100.0, 0.0);
        let _ = pid.tick(0.0, 1.0); // builds integral to 10

        assert!(pid.integral > 0.0);

        pid.reset_integral();
        approx_eq(pid.integral, 0.0, 1e-12);

        pid.set_target(42.0);
        approx_eq(pid.target, 42.0, 1e-12);
    }

    #[test]
    fn is_active_reflects_error_vs_tolerance() {
        let pid = CorePID::new(1.0, 0.0, 0.0, 10.0, 100.0, 0.25);

        assert!(pid.is_active(9.0)); // error = 1.0 > 0.25
        assert!(!pid.is_active(9.9)); // error = 0.1 <= 0.25
    }
}
