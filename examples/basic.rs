// Most minimalist Antaeus usage example
use antaeus::peripherals::drivetrain::{Drivable, differential::StandardDifferential};
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
    );

    let controller = peripherals.primary_controller;
    loop {
        let _ = drivetrain.tank(&controller);
    }
}
