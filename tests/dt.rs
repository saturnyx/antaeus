use std::{cell::Cell, rc::Rc, time::Duration};

use antaeus::{
    peripherals::drivetrain::{Differential, DrivetrainError},
    utils::error::Report,
};
use vexide::{math::Angle, smart::motor::BrakeMode};

/// A simulated differential drivetrain for testing PID/odometry logic
/// without real hardware. Call [`SimDrive::tick`] once per simulation
/// step (e.g. once per macroquad frame) to integrate voltage -> velocity
/// -> position.
#[derive(Clone)]
pub struct SimDrive {
    left_voltage:  Rc<Cell<f64>>,
    right_voltage: Rc<Cell<f64>>,

    // Raw accumulated angle for each side, in radians, never reset.
    left_raw:  Rc<Cell<f64>>,
    right_raw: Rc<Cell<f64>>,

    // Offsets applied by `set_position`/`reset_position` so callers can
    // zero/re-target the reported position without disturbing the
    // underlying simulated motion.
    left_offset:  Rc<Cell<f64>>,
    right_offset: Rc<Cell<f64>>,

    brake_mode: Rc<Cell<BrakeMode>>,

    /// Free-spin angular velocity at 12V, in rad/s. Tune to match the
    /// cartridge you're simulating, e.g. a 200 rpm blue cartridge is
    /// `200.0 * std::f64::consts::TAU / 60.0`.
    max_angular_velocity: f64,
}

impl SimDrive {
    pub fn new(max_angular_velocity: f64) -> Self {
        Self {
            left_voltage: Rc::new(Cell::new(0.0)),
            right_voltage: Rc::new(Cell::new(0.0)),
            left_raw: Rc::new(Cell::new(0.0)),
            right_raw: Rc::new(Cell::new(0.0)),
            left_offset: Rc::new(Cell::new(0.0)),
            right_offset: Rc::new(Cell::new(0.0)),
            brake_mode: Rc::new(Cell::new(BrakeMode::Coast)),
            max_angular_velocity,
        }
    }

    /// Advances the simulation by `dt`, integrating each side's commanded
    /// voltage into an angular velocity and accumulating position.
    /// Call this once per sim step — it's the "physics update," separate
    /// from the trait methods which only read/write state.
    pub fn tick(&self, dt: Duration) {
        let dt = dt.as_secs_f64();

        let left_vel = (self.left_voltage.get() / 12.0) * self.max_angular_velocity;
        let right_vel = (self.right_voltage.get() / 12.0) * self.max_angular_velocity;

        self.left_raw.set(self.left_raw.get() + left_vel * dt);
        self.right_raw.set(self.right_raw.get() + right_vel * dt);
    }

    fn left_angle_rad(&self) -> f64 { self.left_raw.get() - self.left_offset.get() }

    fn right_angle_rad(&self) -> f64 { self.right_raw.get() - self.right_offset.get() }
}

impl Differential for SimDrive {
    fn left_position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        Report::Ok(Angle::from_radians(self.left_angle_rad()))
    }

    fn right_position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        Report::Ok(Angle::from_radians(self.right_angle_rad()))
    }

    fn position(&self) -> Report<Angle, Vec<DrivetrainError>> {
        let avg = (self.left_angle_rad() + self.right_angle_rad()) / 2.0;
        Report::Ok(Angle::from_radians(avg))
    }

    fn reset_position(&self) -> Result<(), DrivetrainError> {
        self.left_offset.set(self.left_raw.get());
        self.right_offset.set(self.right_raw.get());
        Ok(())
    }

    fn set_brakemode(&self, brakemode: BrakeMode) -> Result<(), DrivetrainError> {
        self.brake_mode.set(brakemode);
        Ok(())
    }

    fn set_left_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        self.left_voltage.set(voltage.clamp(-12.0, 12.0));
        Ok(())
    }

    fn set_right_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        self.right_voltage.set(voltage.clamp(-12.0, 12.0));
        Ok(())
    }

    fn set_position(&self, position: Angle) -> Result<(), DrivetrainError> {
        // Shift both offsets so `position()` immediately reads `position`
        // at the current raw angle, without moving the simulated motors.
        let target = position.as_radians();
        let current_avg_raw = (self.left_raw.get() + self.right_raw.get()) / 2.0;
        let offset = current_avg_raw - target;
        self.left_offset.set(offset);
        self.right_offset.set(offset);
        Ok(())
    }

    fn set_voltage(&self, voltage: f64) -> Result<(), DrivetrainError> {
        self.set_left_voltage(voltage)?;
        self.set_right_voltage(voltage)
    }
}
