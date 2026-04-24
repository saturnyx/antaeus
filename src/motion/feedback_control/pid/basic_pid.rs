use std::time::Duration;

use vexide::time::user_uptime;

#[derive(Debug, Clone)]
pub struct BasicPID {
    pub target:      f64,
    pub kp:          f64,
    pub ki:          f64,
    pub kd:          f64,
    pub max:         f64,
    pub prev_error:  f64,
    pub integral:    f64,
    pub last_update: Duration,
    pub tolerance:   f64,
}

impl BasicPID {
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
            last_update: user_uptime(),
        }
    }

    pub fn tick(&mut self, reading: f64) -> f64 {
        let now = user_uptime();
        let dt = (now - self.last_update).as_secs_f64();
        let dt = dt.max(1e-6);
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
        self.last_update = now;

        if error.abs() > self.tolerance {
            output
        } else {
            0.0
        }
    }

    pub fn reset_integral(&mut self) { self.integral = 0.0 }

    pub fn set_target(&mut self, target: f64) { self.target = target; }

    pub fn is_active(&self, reading: f64) -> bool {
        let error = self.target - reading;
        error.abs() > self.tolerance
    }
}
