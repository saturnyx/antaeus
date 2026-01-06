//! Controller input mapping for operator control.
//!
//! This module provides utilities for mapping controller button presses
//! to motor voltages and ADI digital outputs. It supports:
//!
//! - **Toggle controls**: Button press toggles a state (e.g., open/close piston).
//! - **Momentary controls**: Button held activates, release deactivates.
//! - **Dual-button controls**: Two buttons for forward/reverse or extend/retract.
//! - **Control button modifiers**: Require a "shift" button to be held.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::opcontrol::controller::{ControllerControl, ControllerButton};
//!
//! let control = ControllerControl::new(&controller, ControllerButton::ButtonA);
//!
//! // R1/R2 controls intake forward/reverse
//! control.dual_button_to_motors(
//!     ControllerButton::ButtonR1,
//!     ControllerButton::ButtonR2,
//!     heapless::Vec::from_array([&mut intake]),
//!     12.0, -12.0, 0.0, false,
//! );
//! ```

use heapless::Vec;
use log::warn;
use vexide::{
    controller::{ButtonState, ControllerState},
    prelude::{AdiDigitalOut, Controller, Motor},
};

/// Controller input mapper for operator control.
///
/// This struct captures the current controller state and a designated
/// "control button" that acts as a modifier (like a shift key).
///
/// # Control Button
///
/// The control button enables extended controls. When `ctrl: true` is
/// passed to mapping methods, the action only triggers if the control
/// button is also held. This effectively doubles the available controls.
///
/// # Example
///
/// ```ignore
/// let control = ControllerControl::new(&controller, ControllerButton::ButtonA);
///
/// // ButtonB triggers normally
/// control.button_to_adi_toggle(ControllerButton::ButtonB, pistons, false);
///
/// // ButtonB + ButtonA (ctrl) triggers a different action
/// control.button_to_adi_toggle(ControllerButton::ButtonB, other_pistons, true);
/// ```
#[allow(dead_code)]
pub struct ControllerControl {
    /// The current state of all controller buttons and sticks.
    state:      ControllerState,
    /// The button designated as the control/modifier button.
    controlkey: ButtonState,
}

#[allow(dead_code)]
impl ControllerControl {
    /// Creates a new ControllerControl Instance that can be used to easily
    /// control motors and ADI ports.
    ///
    /// # Arguments
    ///
    /// * `controller` - The VEX controller to read input from.
    /// * `button` - The button to use as the control/modifier button.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// ```
    pub fn new(controller: &Controller, button: ControllerButton) -> Self {
        let state = get_state(controller);
        let control_button = get_button_state(state, button);

        ControllerControl {
            state,
            controlkey: control_button,
        }
    }

    /// Maps the output from a button to toggle one or more ADI Devices. A
    /// maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button` - The primary button that will control the device.
    /// * `adi_devices` - A `heapless::Vec` of ADI devices to control.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`, unless you
    ///   wish to press the control button and the primary button together.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut piston = AdiDigitalOut::new(peripherals.adi_a);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.button_to_adi_toggle(
    ///     ControllerButton::ButtonB,
    ///     heapless::Vec::from_array([&mut piston]),
    ///     false,
    /// );
    /// ```
    pub fn button_to_adi_toggle(
        &self,
        button: ControllerButton,
        adi_devices: Vec<&mut AdiDigitalOut, 8>,
        ctrl: bool,
    ) {
        let button_state = get_button_state(self.state, button);
        if button_state.is_now_released() && self.controlkey.is_pressed() == ctrl {
            for device in adi_devices {
                device.toggle().unwrap_or_else(|e| {
                    warn!("ADI Toggle Error: {}", e);
                });
            }
        }
    }

    /// Maps the output from a button to set one or more ADI Devices to high. A
    /// maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button` - The primary button that will control the device.
    /// * `adi_devices` - A `heapless::Vec` of ADI devices to control.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`, unless you
    ///   wish to press the control button and the primary button together.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut piston = AdiDigitalOut::new(peripherals.adi_a);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.button_to_adi_high(
    ///     ControllerButton::ButtonB,
    ///     heapless::Vec::from_array([&mut piston]),
    ///     false,
    /// );
    /// ```
    pub fn button_to_adi_high(
        &self,
        button: ControllerButton,
        adi_devices: Vec<&mut AdiDigitalOut, 8>,
        ctrl: bool,
    ) {
        let button_state = get_button_state(self.state, button);
        if button_state.is_now_released() && self.controlkey.is_pressed() == ctrl {
            for device in adi_devices {
                device.set_high().unwrap_or_else(|e| {
                    warn!("ADI Set High Error: {}", e);
                });
            }
        }
    }

    /// Maps the output from a button to set one or more ADI Devices to low. A
    /// maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button` - The primary button that will control the device.
    /// * `adi_devices` - A `heapless::Vec` of ADI devices to control.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`, unless you
    ///   wish to press the control button and the primary button together.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut piston = AdiDigitalOut::new(peripherals.adi_a);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.button_to_adi_low(
    ///     ControllerButton::ButtonB,
    ///     heapless::Vec::from_array([&mut piston]),
    ///     false,
    /// );
    /// ```
    pub fn button_to_adi_low(
        &self,
        button: ControllerButton,
        adi_devices: Vec<&mut AdiDigitalOut, 8>,
        ctrl: bool,
    ) {
        let button_state = get_button_state(self.state, button);
        if button_state.is_now_released() && self.controlkey.is_pressed() == ctrl {
            for device in adi_devices {
                device.set_low().unwrap_or_else(|e| {
                    warn!("ADI Set Low Error: {}", e);
                });
            }
        }
    }

    /// Maps 2 Buttons to one or more ADI Devices. The High Button outputs a
    /// high value to the ADI Devices. The Low Button outputs a low value to
    /// the ADI Devices. A maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button_high` - The button that will set the device to high.
    /// * `button_low` - The button that will set the device to low.
    /// * `adi_devices` - A `heapless::Vec` of ADI devices to control.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`, unless you
    ///   wish to press the control button and another button.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut piston = AdiDigitalOut::new(peripherals.adi_a);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.dual_button_to_adi(
    ///     ControllerButton::ButtonL1,
    ///     ControllerButton::ButtonL2,
    ///     heapless::Vec::from_array([&mut piston]),
    ///     false,
    /// );
    /// // L1 extends, L2 retracts
    /// ```
    pub fn dual_button_to_adi(
        &self,
        button_high: ControllerButton,
        button_low: ControllerButton,
        adi_devices: Vec<&mut AdiDigitalOut, 8>,
        ctrl: bool,
    ) {
        let button_high_state = get_button_state(self.state, button_high);
        let button_low_state = get_button_state(self.state, button_low);
        if self.controlkey.is_pressed() == ctrl {
            if button_high_state.is_now_released() {
                for device in adi_devices {
                    device.set_high().unwrap_or_else(|e| {
                        warn!("ADI Toggle Error: {}", e);
                    });
                }
            } else if button_low_state.is_now_released() {
                for device in adi_devices {
                    device.set_low().unwrap_or_else(|e| {
                        warn!("ADI Toggle Error: {}", e);
                    });
                }
            }
        }
    }

    /// Maps a button to one or more motors. Pressing the button will run the
    /// motor at active power. Otherwise, the motor will run at passive power.
    /// A maximum of 8 motors can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button` - The primary button that will control the motors.
    /// * `motors` - A `heapless::Vec` of motors to control.
    /// * `active_pwr` - The power (in Volts) given to the motors when button is pressed.
    /// * `passive_pwr` - The power (in Volts) given to the motors when button is not pressed.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut intake = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.button_to_motors(
    ///     ControllerButton::ButtonL1,
    ///     heapless::Vec::from_array([&mut intake]),
    ///     12.0,  // active power
    ///     0.0,   // passive power
    ///     false,
    /// );
    /// ```
    pub fn button_to_motors(
        &self,
        button: ControllerButton,
        motors: Vec<&mut Motor, 8>,
        active_pwr: f64,
        passive_pwr: f64,
        ctrl: bool,
    ) {
        let button_state = get_button_state(self.state, button);
        if self.controlkey.is_pressed() == ctrl {
            if button_state.is_pressed() {
                for motor in motors {
                    motor.set_voltage(active_pwr).unwrap_or_else(|e| {
                        warn!("Motor Set Voltage Error: {}", e);
                    });
                }
            } else {
                for motor in motors {
                    motor.set_voltage(passive_pwr).unwrap_or_else(|e| {
                        warn!("Motor Set Voltage Error: {}", e);
                    });
                }
            }
        }
    }

    /// Maps 2 buttons to one or more motors. The High button outputs a high
    /// power to the motors. The Low button outputs a low power to the motors.
    /// A maximum of 8 motors can be controlled at a time.
    ///
    /// # Arguments
    ///
    /// * `button_high` - The button that will give high power to the motors.
    /// * `button_low` - The button that will give low power to the motors.
    /// * `motors` - A `heapless::Vec` of motors to control.
    /// * `high_pwr` - The power (in Volts) given to the motors when High button is pressed.
    /// * `low_pwr` - The power (in Volts) given to the motors when Low button is pressed.
    /// * `passive_pwr` - The power (in Volts) given to the motors when no button is pressed.
    /// * `ctrl` - Whether to use the control button. Usually set to `false`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
    /// use vexide::prelude::*;
    ///
    /// let master = Controller::new(ControllerId::Primary);
    /// let mut intake = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///
    /// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// master_control.dual_button_to_motors(
    ///     ControllerButton::ButtonL1,  // forward
    ///     ControllerButton::ButtonL2,  // reverse
    ///     heapless::Vec::from_array([&mut intake]),
    ///     12.0,  // high power
    ///     -12.0, // low power
    ///     0.0,   // passive power
    ///     false,
    /// );
    /// ```
    pub fn dual_button_to_motors(
        &self,
        button_high: ControllerButton,
        button_low: ControllerButton,
        motors: Vec<&mut Motor, 8>,
        high_pwr: f64,
        low_pwr: f64,
        passive_pwr: f64,
        ctrl: bool,
    ) {
        let button_high_state = get_button_state(self.state, button_high);
        let button_low_state = get_button_state(self.state, button_low);

        if self.controlkey.is_pressed() == ctrl {
            if button_high_state.is_pressed() {
                for motor in motors {
                    motor.set_voltage(high_pwr).unwrap_or_else(|e| {
                        warn!("Motor Set Voltage Error: {}", e);
                    });
                }
            } else if button_low_state.is_pressed() {
                for motor in motors {
                    motor.set_voltage(low_pwr).unwrap_or_else(|e| {
                        warn!("Motor Set Voltage Error: {}", e);
                    });
                }
            } else {
                for motor in motors {
                    motor.set_voltage(passive_pwr).unwrap_or_else(|e| {
                        warn!("Motor Set Voltage Error: {}", e);
                    });
                }
            }
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
/// An enumeration of controller buttons.
///
/// These represent the physical buttons on a VEX controller that can be
/// mapped to robot actions.
///
/// # Example
///
/// ```ignore
/// use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
/// use vexide::prelude::*;
///
/// let master = Controller::new(ControllerId::Primary);
/// let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
/// ```
pub enum ControllerButton {
    ButtonA,
    ButtonB,
    ButtonX,
    ButtonY,
    ButtonUp,
    ButtonDown,
    ButtonLeft,
    ButtonRight,
    ButtonL1,
    ButtonL2,
    ButtonR1,
    ButtonR2,
}

fn get_button_state(state: ControllerState, button: ControllerButton) -> ButtonState {
    match button {
        ControllerButton::ButtonA => state.button_a,
        ControllerButton::ButtonB => state.button_b,
        ControllerButton::ButtonX => state.button_x,
        ControllerButton::ButtonY => state.button_y,
        ControllerButton::ButtonUp => state.button_up,
        ControllerButton::ButtonDown => state.button_down,
        ControllerButton::ButtonLeft => state.button_left,
        ControllerButton::ButtonRight => state.button_right,
        ControllerButton::ButtonL1 => state.button_l1,
        ControllerButton::ButtonL2 => state.button_l2,
        ControllerButton::ButtonR1 => state.button_r1,
        ControllerButton::ButtonR2 => state.button_r2,
    }
}

fn get_state(controller: &Controller) -> ControllerState {
    controller.state().unwrap_or_else(|e| {
        warn!("Controller State Error: {}", e);
        ControllerState::default()
    })
}
