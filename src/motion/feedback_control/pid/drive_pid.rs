use std::time::Duration;

use measurements::Length;
use vexide::{math::Angle, time::user_uptime};

use super::basic_pid::BasicPID;
use crate::peripherals::drivetrain::Differential;
pub struct DrivePID {
    pub drivetrain:        Differential,
    pub pid_left:          BasicPID,
    pub pid_right:         BasicPID,
    pub wheel_diameter:    Length,
    pub motor_wheel_ratio: f64,
}

impl DrivePID {
    pub fn new(
        drivetrain: Differential,
        kp: f64,
        ki: f64,
        kd: f64,
        max: f64,
        wheel_diameter: Length,
        motor_gear_teeth: u32,
        wheel_gear_teeth: u32,
        default_target: Length,
        tolerance: Length,
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

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
            motor_wheel_ratio,
        }
    }

    pub fn from_basic_pid(
        drivetrain: Differential,
        pid_left: BasicPID,
        pid_right: BasicPID,
        wheel_diameter: Length,
        motor_gear_teeth: u32,
        wheel_gear_teeth: u32,
    ) -> Self {
        let motor_wheel_ratio = gears_to_motor_wheel_ratio(motor_gear_teeth, wheel_gear_teeth);

        Self {
            drivetrain,
            pid_left,
            pid_right,
            wheel_diameter,
            motor_wheel_ratio,
        }
    }

    pub fn tick(&mut self) {
        let left_reading = self.drivetrain.left_position();
        let right_reading = self.drivetrain.right_position();
        let left_power = self.pid_left.tick(
            arc_length(left_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
        );
        let right_power = self.pid_right.tick(
            arc_length(right_reading, self.wheel_diameter, self.motor_wheel_ratio).as_inches(),
        );
        self.drivetrain.set_left_voltage(left_power);
        self.drivetrain.set_right_voltage(right_power);
    }

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
        self.pid_left.last_update = user_uptime();
        self.pid_right.last_update = user_uptime();
    }

    pub fn set_target(&mut self, left: Length, right: Length) {
        self.pid_left.set_target(left.as_inches());
        self.pid_right.set_target(right.as_inches());
        self.reset_integral();
        self.pid_left.prev_error = 0.0;
        self.pid_right.prev_error = 0.0;
        self.pid_left.last_update = user_uptime();
        self.pid_right.last_update = user_uptime();
    }

    pub fn reset_integral(&mut self) {
        self.pid_left.reset_integral();
        self.pid_right.reset_integral();
    }

    pub async fn autotick(&mut self, timeout: Duration) {
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
                break;
            }
        }
        self.drivetrain.set_voltage(0.0);
    }
}

fn arc_length(motor_angle: Angle, wheel_diameter: Length, mw_ratio: f64) -> Length {
    debug_assert!(mw_ratio > 0.0);

    let radius_in = wheel_diameter.as_inches() * 0.5;
    let wheel_angle_rad = motor_angle.as_radians() / mw_ratio;
    Length::from_inches(radius_in * wheel_angle_rad)
}

fn gears_to_motor_wheel_ratio(motor_gear_teeth: u32, wheel_gear_teeth: u32) -> f64 {
    assert!(motor_gear_teeth > 0, "motor_gear_teeth must be > 0");
    assert!(wheel_gear_teeth > 0, "wheel_gear_teeth must be > 0");

    // motor rotations per 1 wheel rotation
    wheel_gear_teeth as f64 / motor_gear_teeth as f64
}
