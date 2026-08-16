// Group PID Drivetrain Example
use std::time::Duration;

use antaeus::prelude::pid::group_pid::GroupPID;
use vexide::{math::Angle, prelude::*};

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Define all motors that you need to control
    let motors = [
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ];

    // Declare a PID instance
    let mut pid = GroupPID::new(
        motors,                    // Motors declared above
        0.5, // Kp (This is a default value, remember to tune this for your own drivetrain)
        0.0, // Ki (This is a default value, remember to tune this for your own drivetrain)
        0.1, // Kd (This is a default value, remember to tune this for your own drivetrain)
        Angle::from_degrees(90.0), // Default Target
        12.0, // Maximum Power [0-12V]
        Angle::from_degrees(0.1), // Tolerance
    );
    // IMPORTANT: Autotick must be called for PID to run. But you can also do this manually by
    // calling `pid.tick()` in a loop.
    pid.autotick(Duration::from_secs(12)).await;
}
