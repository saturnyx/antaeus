// A Basic Drivetrain PID Example
use std::{num::NonZeroU32, time::Duration};

use antaeus::{
    peripherals::drivetrain::differential::StandardDifferential,
    prelude::{Length, pid::drive_pid::DrivePID},
};
use vexide::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // First, declare your drivetrain
    let drivetrain = StandardDifferential::new(
        [
            // Left-Side Motors
            Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
            Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        ],
        [
            // Right-Side Motors
            Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
        ],
    );

    // Next, declare your PID controller with the desired parameters
    let mut pid = DrivePID::new(
        drivetrain,                  // The above drivetrain
        0.5,  // Kp (This is a default value, remember to tune this for your own drivetrain)
        0.0,  // Ki (This is a default value, remember to tune this for your own drivetrain)
        0.1,  // Kd (This is a default value, remember to tune this for your own drivetrain)
        12.0, // Maximum Power the PID controller can output [0-12V]
        Length::from_inches(3.25), // Wheel Diameter (This uses 3.25" wheels)
        NonZeroU32::new(3).unwrap(), // Driving Gear (on motor axle) Teeth Count
        NonZeroU32::new(4).unwrap(), // Driven Gear (on wheel axle) Teeth Count
        Length::from_inches(12.0), // Track Width
        Length::from_inches(10.0), // Default Target
        Length::from_inches(0.1), // Tolerance
    );
    // IMPORTANT: Autotick must be called for PID to run. But you can also do this manually by
    // calling `pid.tick()` in a loop.
    pid.autotick(Duration::from_secs(12)).await;
}
