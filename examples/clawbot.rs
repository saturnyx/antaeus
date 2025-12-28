//! Clawbot Control Example
//!
//! Demonstrates a program written for the V5 clawbot kit robot. This example is partially based on
//! jpearman's [`v5-drivecode`] repository.
//!
//! [`v5-drivecode`]: https://github.com/jpearman/v5-drivecode

use std::sync::Arc;

use antaeus::{drivetrain::Differential, motion::pid::pid::*, opcontrol::controller::*};
use heapless::Vec;
use vexide::{prelude::*, sync::Mutex};
struct Clawbot {
    drivetrain: Differential,
    claw:       Motor,
    arm:        Motor,
    controller: Controller,
}

impl Compete for Clawbot {
    async fn autonomous(&mut self) {
        let dt_conf = DrivetrainConfig {
            wheel_diameter: 4.15,
            driving_gear:   1.0,
            driven_gear:    1.0,
            track_width:    12.0,
        };
        let pid_values = PIDValues {
            kp:           0.1,
            ki:           0.0,
            kd:           0.01,
            tolerance:    0.5,
            maxpwr:       12.0,
            active:       false,
            target_left:  0.0,
            target_right: 0.0,
        };
        let pid = PIDMovement {
            drivetrain:        self.drivetrain.clone(),
            drivetrain_config: dt_conf,
            pid_values:        Arc::new(Mutex::new(pid_values)),
        };

        pid.init();
        pid.set_maximum_power(12.0).await;
        pid.travel(10.0, 2000, 10).await;
        pid.rotate(180.0, 2000, 10).await;
    }

    async fn driver(&mut self) {
        let control = ControllerControl::new(&self.controller, ControllerButton::ButtonX);
        loop {
            self.drivetrain.tank(&self.controller);
            control.dual_button_to_motors(
                ControllerButton::ButtonUp,
                ControllerButton::ButtonDown,
                Vec::from_array([&mut self.arm]),
                8.0,
                -8.0,
                0.0,
                false,
            );
            control.dual_button_to_motors(
                ControllerButton::ButtonLeft,
                ControllerButton::ButtonRight,
                Vec::from_array([&mut self.claw]),
                8.0,
                -8.0,
                4.0,
                false,
            );
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Configuring devices and handing off control to the competition API.
    Clawbot {
        drivetrain: Differential::new(
            [
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
            ],
            [
                Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
                Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
            ],
        ),
        claw:       Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
        arm:        Motor::new(peripherals.port_8, Gearset::Green, Direction::Forward),
        controller: peripherals.primary_controller,
    }
    .compete()
    .await;
}
