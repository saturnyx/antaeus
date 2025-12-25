use std::{cell::RefCell, rc::Rc};

use log::warn;
use vexide::{
    controller::ControllerState,
    prelude::{Controller, Motor},
    smart::motor::BrakeMode,
};

#[derive(Clone)]
#[allow(dead_code)]
pub struct Differential {
    /// Left motors.
    pub left: Rc<RefCell<dyn AsMut<[Motor]>>>,

    /// Right motors.
    pub right: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

#[allow(dead_code)]
impl Differential {
    /// Creates a new drivetrain with the provided left/right motors.
    /// **Compatible with Evian**
    ///
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     [
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ],
    ///     [
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    /// ```
    pub fn new<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: L,
        right: R,
    ) -> Self {
        Self {
            left:  Rc::new(RefCell::new(left)),
            right: Rc::new(RefCell::new(right)),
        }
    }

    /// Controls a tank-style drivetrain using the input from a controller.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::Controller;
    /// let controller = Controller::new(ControllerId::Primary);
    /// motors.tank(&controller);
    /// ```
    pub fn tank(&self, controller: &Controller) {
        let state = controller.state().unwrap_or_else(|e| {
            warn!("Controller State Error: {}", e);
            ControllerState::default()
        });

        let left_power = state.left_stick.y();
        let right_power = state.right_stick.y();

        let left_voltage = left_power * 12.0;
        let right_voltage = right_power * 12.0;

        if let Ok(mut left_motors) = self.left.try_borrow_mut() {
            for motor in left_motors.as_mut() {
                let _ = motor.set_voltage(left_voltage);
            }
        }

        if let Ok(mut right_motors) = self.right.try_borrow_mut() {
            for motor in right_motors.as_mut() {
                let _ = motor.set_voltage(right_voltage);
            }
        }
    }

    /// Drive the robot using arcade controls (single-stick forward/back + single-stick turn).
    ///
    /// Behavior:
    /// - Forward/backward is read from the left stick Y axis.
    /// - Turning is read from the right stick X axis.
    /// - The two values are mixed into left/right voltages as:
    ///   - left = (fwd + turn) * 12.0
    ///   - right = (fwd - turn) * 12.0
    /// - If reading the controller state fails, zeroed inputs are used (no movement) and a warning is logged.
    ///
    /// Notes:
    /// - Inputs are assumed to be in the range [-1.0, 1.0] and are scaled to volts by 12.0.
    /// - Consider applying your own deadband before calling if small-stick noise is an issue.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// drivetrain.arcade(&controller);
    /// ```
    pub fn arcade(&self, controller: &Controller) {
        let state = controller.state().unwrap_or_else(|e| {
            warn!("Controller State Error: {}", e);
            ControllerState::default()
        });

        let fwd = state.left_stick.y();
        let turn = state.right_stick.x();

        let left_voltage = (fwd + turn) * 12.0;
        let right_voltage = (fwd - turn) * 12.0;

        if let Ok(mut left_motors) = self.left.try_borrow_mut() {
            for motor in left_motors.as_mut() {
                let _ = motor.set_voltage(left_voltage);
            }
        }

        if let Ok(mut right_motors) = self.right.try_borrow_mut() {
            for motor in right_motors.as_mut() {
                let _ = motor.set_voltage(right_voltage);
            }
        }
    }

    /// Drive the robot using reversed tank controls (sticks swapped and inverted).
    ///
    /// Behavior:
    /// - Left motors take input from the RIGHT stick Y axis, inverted.
    /// - Right motors take input from the LEFT stick Y axis, inverted.
    /// - Computation:
    ///   - left = (-right_y) * 12.0
    ///   - right = (-left_y) * 12.0
    /// - This is useful when the robot is driving backwards but you want the sticks
    ///   to maintain an intuitive left/right mapping relative to the robot's new front.
    /// - On controller read error, zeroed inputs are used and a warning is logged.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// drivetrain.reverse_tank(&controller);
    /// ```
    pub fn reverse_tank(&self, controller: &Controller) {
        let state = controller.state().unwrap_or_else(|e| {
            warn!("Controller State Error: {}", e);
            ControllerState::default()
        });

        let left_voltage = (-state.right_stick.y()) * 12.0;
        let right_voltage = (-state.left_stick.y()) * 12.0;

        if let Ok(mut left_motors) = self.left.try_borrow_mut() {
            for motor in left_motors.as_mut() {
                let _ = motor.set_voltage(left_voltage);
            }
        }

        if let Ok(mut right_motors) = self.right.try_borrow_mut() {
            for motor in right_motors.as_mut() {
                let _ = motor.set_voltage(right_voltage);
            }
        }
    }

    /// Drive the robot using reversed arcade controls (forward/turn both inverted).
    ///
    /// Behavior:
    /// - Forward/backward is read from the left stick Y axis, but inverted (fwd = -left_y).
    /// - Turning is read from the right stick X axis, also inverted (turn = -right_x).
    /// - Mixed into left/right voltages as:
    ///   - left = (fwd + turn) * 12.0
    ///   - right = (fwd - turn) * 12.0
    /// - This inversion preserves intuitive steering when the robot is driving backwards
    ///   (pushing the right stick right still causes a clockwise turn relative to the driver).
    /// - On controller read error, zeroed inputs are used and a warning is logged.
    ///
    /// Notes:
    /// - Inputs are assumed to be in the range [-1.0, 1.0] and are scaled to volts by 12.0.
    ///
    /// # Example
    /// ```ignore
    /// use vexide::prelude::Controller;
    /// use vexide::devices::controller::ControllerId;
    /// let controller = Controller::new(ControllerId::Primary);
    /// drivetrain.reverse_arcade(&controller);
    /// ```
    pub fn reverse_arcade(&self, controller: &Controller) {
        let state = controller.state().unwrap_or_else(|e| {
            warn!("Controller State Error: {}", e);
            ControllerState::default()
        });

        let fwd = -state.left_stick.y();
        let turn = -state.right_stick.x();

        let left_voltage = (fwd + turn) * 12.0;
        let right_voltage = (fwd - turn) * 12.0;

        if let Ok(mut left_motors) = self.left.try_borrow_mut() {
            for motor in left_motors.as_mut() {
                let _ = motor.set_voltage(left_voltage);
            }
        }

        if let Ok(mut right_motors) = self.right.try_borrow_mut() {
            for motor in right_motors.as_mut() {
                let _ = motor.set_voltage(right_voltage);
            }
        }
    }

    /// Sets the brakemode for the drivetrain
    pub fn set_brakemode(&self, brakemode: BrakeMode) {
        let left = self.left.try_borrow_mut();
        let right = self.right.try_borrow_mut();

        if let Ok(mut motors) = left {
            for motor in motors.as_mut() {
                let _ = motor.brake(brakemode);
            }
        }
        if let Ok(mut motors) = right {
            for motor in motors.as_mut() {
                let _ = motor.brake(brakemode);
            }
        }
    }

    /// Creates a new drivetrain with shared ownership of the left/right motors.
    /// **Compatible with Evian**
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ])),
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ])),
    /// );
    /// ```
    ///
    /// ```
    /// let motors = Differential::new(
    ///     shared_motors![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ],
    ///     shared_motors![
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    /// ```
    pub fn from_shared<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: Rc<RefCell<L>>,
        right: Rc<RefCell<R>>,
    ) -> Self {
        Self { left, right }
    }
}
