/// A module for ArcPID
/// A PID that allows the robot move in arcs
/// This is less precise than standard PID
pub mod arcpid;
/// A module for Odometry
pub mod odom;
/// Standard PID Module
pub mod pid;
/// A Module that implements the Candidate-Based
/// Pursuit Algorithm, a more robust variant of
/// pure pursuit
pub mod pusuit;
