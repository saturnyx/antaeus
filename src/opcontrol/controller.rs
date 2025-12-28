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
    /// # Examples
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    /// }
    /// ```
    ///
    /// or, if you prefer using a struct:
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     // The above method will log unwrap errors, this would not.
    ///     let master_state = master.state().unwrap_or_default();
    ///     let master_control = ControllerControl {
    ///         state:      master_state,
    ///         controlkey: master_state.button_a,
    ///     };
    /// }
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
    /// - `button`: The Primary Button that will control the device.
    /// - `adi_devices`: A `heapless::Vec` of ADI devices to control.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and the primary button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let mut pistonleft = AdiDigitalOut::new(Peripherals::steal().adi_a);
    ///     let mut pistonright = AdiDigitalOut::new(Peripherals::steal().adi_b);
    ///
    ///     let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    ///     master_control.button_to_adi_toggle(
    ///         ControllerButton::ButtonB,
    ///         heapless::Vec::from_array([&mut pistonleft, &mut pistonright]),
    ///         false,
    ///     ); // Button B will control both PistonLeft (ADI Port A) and PistonRight (ADI Port B).
    /// }
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
    /// - `button`: The Primary Button that will control the device.
    /// - `adi_devices`: A `heapless::Vec` of ADI devices to control.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and the primary button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let mut pistonleft = AdiDigitalOut::new(Peripherals::steal().adi_a);
    ///     let mut pistonright = AdiDigitalOut::new(Peripherals::steal().adi_b);
    ///
    ///     let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    ///     master_control.button_to_adi_high(
    ///         ControllerButton::ButtonB,
    ///         heapless::Vec::from_array([&mut pistonleft, &mut pistonright]),
    ///         false,
    ///     ); // Button B will control both PistonLeft (ADI Port A) and PistonRight (ADI Port B).
    /// }
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

    /// Maps the output from a button to set one or more ADI Devices to high. A
    /// maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    /// - `button`: The Primary Button that will control the device.
    /// - `adi_devices`: A `heapless::Vec` of ADI devices to control.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and the primary button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let mut pistonleft = AdiDigitalOut::new(Peripherals::steal().adi_a);
    ///     let mut pistonright = AdiDigitalOut::new(Peripherals::steal().adi_b);
    ///
    ///     let master_control = ControllerControl::new(&master, ControllerButton::ButtonA);
    ///     master_control.button_to_adi_low(
    ///         ControllerButton::ButtonB,
    ///         heapless::Vec::from_array([&mut pistonleft, &mut pistonright]),
    ///         false,
    ///     ); // Button B will control both PistonLeft (ADI Port A) and PistonRight (ADI Port B).
    /// }
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

    /// Maps 2 Buttons to one or more ADI Devices. The High Button output a
    /// high value to the ADI Devices. A Low Button will output a low value to
    /// the ADI Devices. A maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    /// - `button_high`: The Button that will set the device to high.
    /// - `button_low`: The Button that will set the device to low.
    /// - `adi_devices`: A `heapless::Vec` of ADI devices to control.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and another button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let master_state = master.state().unwrap_or_default();
    ///     let master_control = ControllerControl {
    ///         state:      master_state,
    ///         controlkey: master_state.button_a,
    ///     };
    ///
    ///     let mut pistonleft = AdiDigitalOut::new(Peripherals::steal().adi_a);
    ///     let mut pistonright = AdiDigitalOut::new(Peripherals::steal().adi_b);
    ///     master_control.dual_button_to_adi(
    ///         ControllerButton::ButtonL1,
    ///         ControllerButton::ButtonL2,
    ///         heapless::Vec::from_array([&mut pistonleft, &mut pistonright]),
    ///         false,
    ///     );
    ///     // Button L1 will extend both PistonLeft (ADI Port A) and PistonRight (ADI Port B).
    ///     // Button L2 will retract both PistonLeft (ADI Port A) and PistonRight (ADI Port B).
    /// }
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
    /// Motor at Active Power. Otherwise, the Motor will run at passive power.
    /// A maximum of 8 ADI devices can be controlled at a time.
    ///
    /// # Arguments
    /// - `button`: The Primary Button that will control the device.
    /// - `motors`: A `heapless::Vec` of Motors to control.
    /// - `active_pwr`: The power(in Volts) given to the Motor when button is pressed.
    /// - `passive_pwr`: The power(in Volts) given to the Motor when button is not pressed.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and the primary button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let master_state = master.state().unwrap_or_default();
    ///     let master_control = ControllerControl {
    ///         state:      master_state,
    ///         controlkey: master_state.button_a,
    ///     };
    ///
    ///     let mut intake_stage1 =
    ///         Motor::new(Peripherals::steal().port_1, Gearset::Red, Direction::Forward);
    ///     let mut intake_stage2 =
    ///         Motor::new(Peripherals::steal().port_2, Gearset::Red, Direction::Forward);
    ///
    ///     master_control.button_to_motors(
    ///         ControllerButton::ButtonL1,
    ///         heapless::Vec::from_array([&mut intake_stage1, &mut intake_stage2]),
    ///         12.0,
    ///         -4.0,
    ///         false,
    ///     );
    ///     // Button L1 will run both Intake Stage 1 and 2 Forward at 12 volts.
    ///     // If L1 is not pressed, Intake Stage 1 and 2 will run in reverse
    ///     // with 4 volts.
    /// }
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

    /// Maps 2 Buttons to one or more motors. The High Button output a high
    /// power to the Motors. A Low Button will output a low value to the Motors.
    /// A maximum of 8 Motors can be controlled at a time.
    ///
    /// # Arguments
    /// - `button_high`: The Button that will give the High Power to the device.
    /// - `button_low`: The Button that will give the Low Power to the device.
    /// - `motors`: A `heapless::Vec` of Motors to control.
    /// - `high_pwr`: The power(in Volts) given to the Motor when High Button is pressed.
    /// - `low_pwr`: The power(in Volts) given to the Motor when Low Button is pressed.
    /// - `passive_pwr`: The power(in Volts) given to the Motor when button is not pressed.
    /// - `ctrl`: Whether to use the control button. Usually set to false, unless you
    /// wish to press the control button and the primary button.
    ///
    /// # Example
    ///
    /// ```
    /// pub unsafe fn run() {
    ///     let master = Controller::new(ControllerId::Partner);
    ///     let master_state = master.state().unwrap_or_default();
    ///     let master_control = ControllerControl {
    ///         state:      master_state,
    ///         controlkey: master_state.button_a,
    ///     };
    ///
    ///     let mut intake_stage1 =
    ///         Motor::new(Peripherals::steal().port_1, Gearset::Blue, Direction::Forward);
    ///     let mut intake_stage2 =
    ///         Motor::new(Peripherals::steal().port_2, Gearset::Blue, Direction::Forward);
    ///
    ///     master_control.dual_button_to_motors(
    ///         ControllerButton::ButtonL1,
    ///         ControllerButton::ButtonL2,
    ///         heapless::Vec::from_array([&mut intake_stage1, &mut intake_stage2]),
    ///         12.0,
    ///         -12.0,
    ///         0.0,
    ///         false,
    ///     );
    ///     // Button L1 will run both Intake Stage 1 and 2 Forward.
    ///     // Button L2 will run both Intake Stage 1 and 2 in Reverse.
    /// }
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
/// A list of Controller Buttons.
///
/// # Example
///
/// ```
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
