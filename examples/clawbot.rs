//! Clawbot Control Example
//!
//! Demonstrates a program written for the V5 clawbot kit robot. This example is partially based on
//! jpearman's [`v5-drivecode`] repository.
//!
//! [`v5-drivecode`]: https://github.com/jpearman/v5-drivecode

use std::{num::NonZeroU32, time::Duration};

use antaeus::{
    motion::feedback_control::pid::drive_pid::DrivePID,
    peripherals::{
        drivetrain::{Drivable, differential::Differential},
        mapper::{DigitalInput, motor::MotorMapper},
    },
    utils::units::Length,
};
use vexide::prelude::*;
struct Clawbot {
    drivetrain: Differential,
    claw:       Motor,
    arm:        Motor,
    controller: Controller,
}

impl Compete for Clawbot {
    async fn autonomous(&mut self) {
        let mut pid = DrivePID::new(
            self.drivetrain.clone(),
            0.5,
            0.0,
            0.0,
            1000.0,
            Length::from_inches(3.25),
            NonZeroU32::new(4).unwrap(),
            NonZeroU32::new(4).unwrap(),
            Length::from_inches(13.0),
            Length::from_inches(0.0),
            Length::from_inches(0.5),
        );

        pid.set_relative_target(Length::from_inches(10.0), Length::from_inches(10.0));
        pid.autotick(Duration::from_secs(5)).await;
        pid.set_relative_target(Length::from_inches(-10.0), Length::from_inches(10.0));
        pid.autotick(Duration::from_secs(5)).await;
    }

    async fn driver(&mut self) {
        loop {
            let _ = self.drivetrain.tank(&self.controller);
            let state = self.controller.state().unwrap_or_default();
            let _ = self.arm.from_dual_input(
                DigitalInput::Button(state.button_up),
                DigitalInput::Button(state.button_down),
                8.0,
                -8.0,
                0.0,
            );
            let _ = self.claw.from_dual_input(
                DigitalInput::Button(state.button_left),
                DigitalInput::Button(state.button_right),
                8.0,
                -8.0,
                2.0,
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
