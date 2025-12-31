use antaeus::{fs::logger, opcontrol::controller::*};
use heapless::Vec;
use log::{LevelFilter, info};

use crate::hardware::Robot;

pub fn opcontrol(robot: &mut Robot) {
    info!("Opcontrol Started");
    let _ = logger::init(LevelFilter::Info);
    antaeus::display::logo::print_badge(&mut robot.display);
    let cc = ControllerControl::new(&robot.main_con, ControllerButton::ButtonRight);

    info!("Opcontrol Loop Started");
    loop {
        robot.dt.tank(&robot.main_con);
        cc.dual_button_to_motors(
            ControllerButton::ButtonR1,
            ControllerButton::ButtonR2,
            Vec::from_array([&mut robot.intake1, &mut robot.intake2]),
            12.0,
            -12.0,
            0.0,
            false,
        );
    }
}
