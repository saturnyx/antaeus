//! PID controllers
//!
//! This module provides PID controller implementations used for motion feedback
//! control.
//!
//! - [`core_pid`]: generic PID controller logic and supporting types.
//! - [`drive_pid`]: PID controller tuned/configured for drive/locomotion use.
//! - [`drive_pid`]: PID controller for controlling a group of motors simultaneously.

pub mod core_pid;
pub mod drive_pid;
pub mod group_pid;
