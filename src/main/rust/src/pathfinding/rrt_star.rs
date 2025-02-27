use std::{collections::HashSet, sync::Arc};

use nalgebra::{Vector2, Vector3};

use crate::hybrid_grid::{GenericDynamicObject, HybridGrid};

use super::Pathfinding;

pub struct RRTStar {
    grid: HybridGrid<Arc<dyn GenericDynamicObject>>,
}

impl Pathfinding for RRTStar {
    fn new(hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>>) -> Self {
        Self { grid: hybrid_grid }
    }

    fn calculate_path(&self, start: Vector2<i32>, end: Vector2<i32>) -> Option<Vec<Vector2<i32>>> {
        todo!()
    }
}
