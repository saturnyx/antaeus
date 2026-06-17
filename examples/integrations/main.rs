use vexide::prelude::*;
pub mod auton;
pub mod hardware;
pub mod integrations;

impl Compete for hardware::Robot {
    async fn autonomous(&mut self) { auton::main_auton(self).await; }

    async fn driver(&mut self) {}
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = hardware::Robot::default_config(peripherals);

    robot.compete().await;
}
