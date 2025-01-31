use std::collections::HashSet;

use nalgebra::Vector3;

use crate::hybrid_grid::{DynamicObject, HashableVector3, HybridGrid};

use super::Pathfinding;

pub struct RRTStar<'a> {
    grid: HybridGrid<'a>,
}

impl<'a> Pathfinding<'a> for RRTStar<'a> {
    fn new(
        dynamic_objects: Vec<&'a DynamicObject>,
        filled_points: HashSet<HashableVector3>,
    ) -> Self {
        Self {
            grid: HybridGrid::new(filled_points, dynamic_objects),
        }
    }

    fn calculate_path(&self, start: Vector3<f64>, end: Vector3<f64>) -> Vec<Vector3<f64>> {
        todo!()
    }
}
