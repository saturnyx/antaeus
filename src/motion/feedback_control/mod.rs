pub mod pid;

/// PID control algorithms.
///
/// Contains multiple PID implementations:
/// - [`pid`](pid::pid): Standard PID for drivetrain control.
/// - [`arcpid`](pid::arcpid): PID that allows arc movements.
/// - [`singlepid`](pid::singlepid): PID for single motor groups.
pub mod legacy_pid;
