pub mod adi;
pub mod motor;
use vexide::{
    adi::{analog::AdiAnalogIn, digital::AdiDigitalIn},
    controller::{ButtonState, JoystickState},
    smart::PortError,
};

pub enum DigitalInput {
    Button(ButtonState),
    AdiIn(AdiDigitalIn),
}

impl DigitalInput {
    pub fn as_bool(&self) -> Result<bool, PortError> {
        match self {
            DigitalInput::Button(state) => Ok(state.is_pressed()),
            DigitalInput::AdiIn(adi_in) => adi_in.is_high(),
        }
    }

    pub fn as_bool_now(&self) -> Result<bool, PortError> {
        match self {
            DigitalInput::Button(state) => Ok(state.is_now_pressed()),
            DigitalInput::AdiIn(adi_in) => adi_in.is_high(),
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum JoystickAxes {
    X,
    Y,
}

pub enum AnalogInput {
    Joystick(JoystickState, JoystickAxes),
    AdiIn(AdiAnalogIn),
}

impl AnalogInput {
    pub fn clamped(&self) -> Result<f64, PortError> {
        match self {
            AnalogInput::Joystick(state, axis) => match axis {
                &JoystickAxes::X => Ok(state.x()),
                &JoystickAxes::Y => Ok(state.y()),
            },
            AnalogInput::AdiIn(adi) => Ok(adi.voltage()? / 5.0),
        }
    }
}
