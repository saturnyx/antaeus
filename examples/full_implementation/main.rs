use log::info;
use vexide::prelude::*;
pub mod auton;
pub mod hardware;
pub mod opcontrol;

impl Compete for hardware::Robot {
    async fn autonomous(&mut self) {
        info!("Autonomous Started")
    }

    async fn driver(&mut self) {
        opcontrol::opcontrol(self);
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = hardware::Robot::default_config(peripherals);

    robot.compete().await;
}
