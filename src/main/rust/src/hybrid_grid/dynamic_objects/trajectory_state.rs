// TODO: call a java class function to get the trajectory

use nalgebra::{Point2, Rotation2};

// Main trajectory struct
#[derive(Clone)]
pub struct PathPlannerTrajectory {
    states: Vec<TrajectoryState>,
}

// State at a point along the trajectory
#[derive(Clone)]
pub struct TrajectoryState {
    time_seconds: f64,
    pose: RobotPose,

    field_speeds: ChassisSpeeds,
    linear_velocity: f64,
    heading: Rotation2<f64>,

    module_states: Vec<SwerveModuleState>,

    feedforwards: DriveFeedforwards,

    delta_pos: f64,
    delta_rot: Rotation2<f64>,
    constraints: PathConstraints,
    waypoint_relative_pos: f64,
}

#[derive(Clone)]
pub struct RobotPose {
    translation: Point2<f64>,
    rotation: Rotation2<f64>,
}

#[derive(Clone)]
pub struct ChassisSpeeds {
    vx: f64,
    vy: f64,
    omega: f64,
}

#[derive(Clone)]
pub struct SwerveModuleState {
    speed: f64, // meters per second
    angle: Rotation2<f64>,
    field_angle: Rotation2<f64>,
    field_pos: Point2<f64>,
    delta_pos: f64,
}

#[derive(Clone)]
pub struct DriveFeedforwards {
    accel: Vec<f64>,
    linear_force: Vec<f64>,
    torque_current: Vec<f64>,
    force_x: Vec<f64>,
    force_y: Vec<f64>,
}

#[derive(Clone)]
pub struct PathConstraints {
    max_velocity: f64,
    max_acceleration: f64,
    max_angular_velocity: f64,
    max_angular_acceleration: f64,
}
