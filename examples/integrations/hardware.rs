use antaeus::*;
use vexide::prelude::*;

pub struct Robot {
    pub main_con: Controller,
    pub dt:       peripherals::drivetrain::differential::Differential,
}

impl Robot {
    pub fn default_config(peripherals: Peripherals) -> Robot {
        Robot {
            main_con: peripherals.primary_controller,
            dt:       peripherals::drivetrain::differential::Differential::new(
                [
                    Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                ],
                [
                    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
                ],
            ),
        }
    }
}
