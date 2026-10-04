// Most minimalist Antaeus usage example
use antaeus::{
    peripherals::drivetrain::{Drivable, differential::StandardDifferential},
    prelude::{Length, differential::DifferentialConfig},
};
use vexide::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut drivetrain = StandardDifferential::new(
        [
            Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
            Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        ],
        [
            Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
        ],
        DifferentialConfig::new(
            Length::from_inches(12.0),
            Length::from_inches(3.25),
            1.0, // Direct Drive
            1.0,
        ),
    );

    let controller = peripherals.primary_controller;
    loop {
        let _ = drivetrain.tank(&controller);
    }
}
