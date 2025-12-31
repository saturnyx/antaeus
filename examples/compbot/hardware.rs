use std::sync::Arc;

use antaeus::*;
use vexide::{prelude::*, sync::Mutex};

pub struct Robot {
    pub main_con: Controller,
    pub dt:       peripherals::drivetrain::Differential,
    pub intake1:  Motor,
    pub intake2:  Motor,

    pub h_tracker: Arc<Mutex<RotationSensor>>,
    pub v_tracker: Arc<Mutex<RotationSensor>>,
    pub imu:       Arc<Mutex<InertialSensor>>,
}

impl Robot {
    pub fn default_config(peripherals: Peripherals) -> Robot {
        Robot {
            main_con: peripherals.primary_controller,
            dt:       peripherals::drivetrain::Differential::new(
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
            intake1:  Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
            intake2:  Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),

            h_tracker: to_mutex(RotationSensor::new(peripherals.port_9, Direction::Forward)),
            v_tracker: to_mutex(RotationSensor::new(peripherals.port_10, Direction::Forward)),
            imu:       to_mutex(InertialSensor::new(peripherals.port_12)),
        }
    }
}
