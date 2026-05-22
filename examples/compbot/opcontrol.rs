use antaeus::peripherals::mapper::{DigitalInput, motor::MotorMapper};

use crate::hardware::Robot;

pub fn opcontrol(robot: &mut Robot) {
    loop {
        robot.dt.tank(&robot.main_con);

        let state = robot.main_con.state().unwrap_or_default();
        let _ = robot.intake1.from_dual_input(
            DigitalInput::Button(state.button_r1),
            DigitalInput::Button(state.button_r2),
            12.0,
            -12.0,
            0.0,
        );
    }
}
