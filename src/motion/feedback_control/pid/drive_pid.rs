use measurements::Length;
use vexide::math::Angle;

use super::basic_pid::BasicPID;
use crate::peripherals::drivetrain::Differential;
pub struct DrivePID {
    pub drivetrain:     Differential,
    pub pid_left:       BasicPID,
    pub pid_right:      BasicPID,
    pub wheel_diameter: Length,
}

impl DrivePID {
    pub fn new(
        drivetrain: Differential,
        kp: f64,
        ki: f64,
        kd: f64,
        max: f64,
        wheel_diameter: Length,
        default_target: Length,
        tolerance: Length,
    ) -> Self {
        Self {
            drivetrain,
            pid_left: BasicPID::new(
                kp,
                ki,
                kd,
                default_target.as_inches(),
                max,
                tolerance.as_inches(),
            ),
            pid_right: BasicPID::new(
                kp,
                ki,
                kd,
                default_target.as_inches(),
                max,
                tolerance.as_inches(),
            ),

            wheel_diameter,
        }
    }

    pub fn from_basic_pid(
        drivetrain: Differential,
        pid_left: BasicPID,
        pid_right: BasicPID,
        wheel_diameter: Length,
    ) -> Self {
        Self {
            drivetrain,
            pid_left: pid_left,
            pid_right: pid_right,
            wheel_diameter,
        }
    }

    pub fn tick(&mut self) {
        let left_reading = self.drivetrain.left_position();
        let right_reading = self.drivetrain.right_position();
        let left_power = self
            .pid_left
            .tick(circumference(left_reading, self.wheel_diameter).as_inches());
        let right_power = self
            .pid_right
            .tick(circumference(right_reading, self.wheel_diameter).as_inches());
        self.drivetrain.set_left_voltage(left_power);
        self.drivetrain.set_right_voltage(right_power);
    }

    pub fn set_target(&mut self, left: Length, right: Length) {
        self.pid_left.set_target(left.as_inches());
        self.pid_right.set_target(right.as_inches());
    }

    pub async fn autotick(&mut self) {
        while self.pid_left.is_active(
            circumference(self.drivetrain.left_position(), self.wheel_diameter).as_inches(),
        ) || self.pid_right.is_active(
            circumference(self.drivetrain.right_position(), self.wheel_diameter).as_inches(),
        ) {
            self.tick();
            vexide::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    }
}

fn circumference(angle: Angle, diameter: Length) -> Length {
    let radius = diameter.as_inches() / 2.0;
    let angle_rad = angle.as_radians();
    let arc_length = radius * angle_rad;
    Length::from_inches(arc_length)
}
