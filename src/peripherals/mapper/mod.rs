//! This module contains the code for mapping the inputs from the controller and
//! other peripherals to the outputs of the motors and ADI devices. This allows
//! for easy control of mechanisms like pneumatic solenoids using controller
//! inputs or other sensors.

pub mod adi;
pub mod motor;
use vexide::{
    adi::{analog::AdiAnalogIn, digital::AdiDigitalIn},
    controller::{ButtonState, JoystickState},
    smart::PortError,
};

/// A digital input that can be used to control a device (e.g. motor, pneumatic
/// solenoid). Digital means that your device only can give 2 values (e.g."high"
///  or "low").
#[derive(Debug, PartialEq, Eq)]
pub enum DigitalInput {
    /// A button's state on the controller (e.g. A, B, L1, R2)
    Button(ButtonState),
    /// An ADI digital input (e.g. a limit switch)
    AdiIn(AdiDigitalIn),
}

impl DigitalInput {
    /// Converts the digital input to a boolean value
    pub fn as_bool(&self) -> Result<bool, PortError> {
        match self {
            DigitalInput::Button(state) => Ok(state.is_pressed()),
            DigitalInput::AdiIn(adi_in) => adi_in.is_high(),
        }
    }

    /// Converts the digital input to a boolean value, but only returns true if
    /// the input was not pressed in the previous tick
    /// (Does not work for ADI inputs, as they do not have a "previous" state)
    pub fn as_bool_now(&self) -> Result<bool, PortError> {
        match self {
            DigitalInput::Button(state) => Ok(state.is_now_pressed()),
            DigitalInput::AdiIn(adi_in) => adi_in.is_high(),
        }
    }
}

/// Your Joystick Axis (X or Y)
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum JoystickAxes {
    /// The X axis of the joystick (left and right)
    X,
    /// The Y axis of the joystick (forward and backward)
    Y,
}

/// A analog input that can be used to control a device (e.g. motor, pneumatic
/// solenoid). Analog means that your device only can give a range between 2
/// values (e.g. 1.0 to -1.0).
#[derive(Debug, PartialEq, Eq)]
pub enum AnalogInput {
    /// A joystick axis on the controller (e.g. left stick X, right stick Y)
    Joystick(JoystickState, JoystickAxes),
    /// An ADI analog input (e.g. a potentiometer)
    AdiIn(AdiAnalogIn),
}

impl AnalogInput {
    /// Converts the analog input to a value between -1.0 and 1.0
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
