use std::{cell::RefCell, rc::Rc};

use antaeus::{
    make_cloneable,
    peripherals::drivetrain::differential::StandardDifferential,
    prelude::{
        Localizer,
        control::basic::BasicControl,
        tracker::{
            Tracker,
            devices::{TrackerMech, TrackerPod},
        },
    },
    utils::units::Length,
};
use vexide::prelude::*;
pub struct Robot {
    pub main_con: Controller,
    pub dt:       StandardDifferential,
    pub intake1:  Motor,
    pub intake2:  Motor,

    pub h_tracker: RotationSensor,
    pub v_tracker: RotationSensor,
    pub imu:       Rc<RefCell<InertialSensor>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let _basic_ctrl = BasicControl {
            track_width: Length::from_inches(13.9),
            tolerance:   Length::from_inches(0.5),
        };

        let vertical = TrackerPod {
            sensor:         &mut self.dt.clone(),
            offset:         Length::zero(),
            wheel_diameter: Length::from_inches(3.25),
            driven_gear:    1.0,
            driving_gear:   1.0,
        };

        let horizontal = TrackerPod {
            sensor:         &mut self.h_tracker,
            offset:         Length::zero(),
            wheel_diameter: Length::from_inches(3.25),
            driven_gear:    1.0,
            driving_gear:   1.0,
        };

        let trackers = TrackerMech::new(vertical, horizontal, self.imu.clone());

        let mut odomtrack = Tracker::new(trackers);
        loop {
            println!("{:?}", odomtrack.get_coords());
            let _ = odomtrack.tick();
        }
    }

    async fn driver(&mut self) {
        // Driver Control Here
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Configuring devices and handing off control to the competition API.
    Robot {
        main_con: peripherals.primary_controller,
        dt:       StandardDifferential::new(
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

        h_tracker: RotationSensor::new(peripherals.port_9, Direction::Forward),
        v_tracker: RotationSensor::new(peripherals.port_10, Direction::Forward),
        imu:       make_cloneable(InertialSensor::new(peripherals.port_12)),
    }
    .compete()
    .await;
}
