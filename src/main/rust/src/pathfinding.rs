use nalgebra::{Vector2, Vector3};

use crate::hybrid_grid::{GenericDynamicObject, HybridGrid};

pub mod a_star;
pub mod rrt_star;

pub struct NodeRadiusSearch {
    pub node_radius_search_radius: f32,

    pub do_absolute_discard: bool,
    pub avg_distance_min_discard_threshold: f32,

    pub avg_distance_cost: f32,
}

/// A generic trait for pathfinding algorithms
pub trait Pathfinding {
    fn new(hybrid_grid: HybridGrid) -> Self;
    fn calculate_path(&self, start: Vector2<i32>, end: Vector2<i32>) -> Option<Vec<Vector2<i32>>>;
}

/// Pathfinders that calculate the total cost of a given node time relative rather than absolute position h, g cost.
/// This might be slower, but it allows for more accurate pathfinding with a robot.
pub trait TimedPathfinding<D: GenericDynamicObject> {
    fn calculate_path_timed(
        &self,
        start: Vector3<f64>,
        end: Vector3<f64>,
        consider_n_intermediate_steps: usize,
    ) -> Vec<Vector3<f64>>;
}
