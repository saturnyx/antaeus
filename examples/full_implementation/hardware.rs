use antaeus::display::{self, DisplayDriver};
use antaeus::*;
use vexide::prelude::*;

pub struct Robot {
    pub main_con: Controller,
    pub dt: drivetrain::Differential,
    pub intake1: Motor,
    pub intake2: Motor,
    pub display: display::DisplayDriver,

    pub h_tracker: RotationSensor,
    pub v_tracker: RotationSensor,
    pub imu: InertialSensor,
}

impl Robot {
    pub fn default_config(peripherals: Peripherals) -> Robot {
        Robot {
            main_con: peripherals.primary_controller,
            dt: drivetrain::Differential::new(
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
            intake1: Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
            intake2: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),

            h_tracker: RotationSensor::new(peripherals.port_9, Direction::Forward),
            v_tracker: RotationSensor::new(peripherals.port_10, Direction::Forward),
            imu: InertialSensor::new(peripherals.port_11),

            display: DisplayDriver::new(peripherals.display),
        }
    }
}
