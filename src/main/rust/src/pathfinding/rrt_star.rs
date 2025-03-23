use nalgebra::Vector2;

use crate::hybrid_grid::HybridGrid;

use super::Pathfinding;

pub struct RRTStar {
    grid: HybridGrid,
}

impl Pathfinding for RRTStar {
    fn new(hybrid_grid: HybridGrid) -> Self {
        Self { grid: hybrid_grid }
    }

    fn calculate_path(&self, start: Vector2<i32>, end: Vector2<i32>) -> Option<Vec<Vector2<i32>>> {
        todo!()
    }
}
