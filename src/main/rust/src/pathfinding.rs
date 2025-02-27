use std::collections::HashSet;

use nalgebra::Vector3;

use crate::{
    hybrid_grid::{DynamicObject, HashableVector3},
    time_structures::TimedPath,
};

pub mod a_star;
pub mod rrt_star;

/// A generic trait for pathfinding algorithms
pub trait Pathfinding<'a> {
    fn new(
        dynamic_objects: Vec<&'a DynamicObject>,
        filled_points: HashSet<HashableVector3>,
    ) -> Self;
    fn calculate_path(&self, start: Vector3<i32>, end: Vector3<i32>) -> Option<Vec<Vector3<i32>>>;
}

/// Pathfinders that calculate the total cost of a given node time relative rather than aboslute pos position h, g cost.
/// This might be slower, but it allows for more accurate pathfinding with a robot.
pub trait TimedPathfinding<'a>: Pathfinding<'a> {
    fn calculate_path_timed(
        &self,
        start: Vector3<f64>,
        end: Vector3<f64>,
        timed_path: &mut TimedPath,
        consider_n_intermediate_steps: usize,
    ) -> Vec<Vector3<f64>>;
}
