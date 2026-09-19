//! PID controllers
//!
//! This module provides PID controller implementations used for motion feedback
//! control.
//!
//! - [`drive_pid`]: PID controller tuned/configured for drive/locomotion use.
//! - [`group_pid`]: PID controller for controlling a group of motors simultaneously.

pub mod drive_pid;
pub mod group_pid;
