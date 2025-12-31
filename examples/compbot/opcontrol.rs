use antaeus::peripherals::controller::*;
use heapless::Vec;

use crate::hardware::Robot;

pub fn opcontrol(robot: &mut Robot) {
    let cc = ControllerControl::new(&robot.main_con, ControllerButton::ButtonRight);
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
