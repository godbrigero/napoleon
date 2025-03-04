use std::{collections::HashSet, num::NonZero, sync::Arc};

use kiddo::{KdTree, NearestNeighbour};
use nalgebra::{Matrix3, Vector2, Vector3};
use num_traits::NumCast;

pub mod dynamic_objects;
pub mod math;

pub trait GenericDynamicObject: Send + Sync {
    fn calculate_transformation_matrix_at(&self, time: f64) -> Matrix3<f64>;
}

impl GenericDynamicObject for Box<dyn GenericDynamicObject> {
    fn calculate_transformation_matrix_at(&self, time: f64) -> Matrix3<f64> {
        (**self).calculate_transformation_matrix_at(time)
    }
}

impl GenericDynamicObject for Arc<dyn GenericDynamicObject> {
    fn calculate_transformation_matrix_at(&self, time: f64) -> Matrix3<f64> {
        (**self).calculate_transformation_matrix_at(time)
    }
}

#[derive(Clone)]
pub struct HybridGrid<D>
where
    D: GenericDynamicObject + Clone,
{
    size_x: i32,
    size_y: i32,
    center_x: i32,
    center_y: i32,
    square_size_meters: f32,

    static_obstacles: HashSet<Vector2<i32>>,
    hybrid_obstacles: KdTree<f32, 2>,
    dynamic_objects: Vec<D>,
}

impl<D: GenericDynamicObject + Clone> HybridGrid<D> {
    pub fn new_raw(
        size_x: i32,
        size_y: i32,
        square_size_meters: f32,
        center_x: i32,
        center_y: i32,
    ) -> Self {
        Self {
            size_x,
            size_y,
            center_x,
            center_y,
            square_size_meters,
            static_obstacles: HashSet::new(),
            dynamic_objects: Vec::new(),
            hybrid_obstacles: KdTree::new(),
        }
    }

    pub fn new(
        size_x: i32,
        size_y: i32,
        square_size_meters: f32,
        static_obstacles: Vec<Vector2<i32>>,
        center_x: i32,
        center_y: i32,
    ) -> Self {
        let mut grid = Self::new_raw(size_x, size_y, square_size_meters, center_x, center_y);

        for obstacle in static_obstacles {
            if !grid.is_outside_grid(obstacle) {
                grid.push_static_obstacle(obstacle);
            }
        }

        grid
    }

    pub fn get_square_size_meters(&self) -> f32 {
        self.square_size_meters
    }

    pub fn add_dynamic_object(&mut self, object: D) {
        self.dynamic_objects.push(object);
    }

    pub fn push_static_obstacle(&mut self, obstacle: Vector2<i32>) {
        self.static_obstacles.insert(obstacle);
    }

    pub fn get_dynamic_object_transformation_matrices_at(
        &self,
        time_ms_since_initial: f64,
    ) -> Vec<Matrix3<f64>> {
        self.dynamic_objects
            .iter()
            .map(|object| {
                let transformation_matrix =
                    object.calculate_transformation_matrix_at(time_ms_since_initial);
                transformation_matrix
            })
            .collect()
    }

    pub fn get_static_obstacles(&self) -> &HashSet<Vector2<i32>> {
        &self.static_obstacles
    }

    pub fn is_outside_grid(&self, position: Vector2<i32>) -> bool {
        let half_size_x = self.size_x / 2;
        let half_size_y = self.size_y / 2;

        position.x < self.center_x - half_size_x
            || position.x >= self.center_x + half_size_x
            || position.y < self.center_y - half_size_y
            || position.y >= self.center_y + half_size_y
    }

    pub fn is_obstructed(&self, position: Vector2<i32>) -> bool {
        self.static_obstacles.contains(&position)
    }

    pub fn add_hybrid_object(&mut self, object: &[f32; 2]) {
        self.hybrid_obstacles
            .add(object, self.hybrid_obstacles.size());
    }

    pub fn clear_hybrid_objects(&mut self) {
        self.hybrid_obstacles = KdTree::new()
    }

    pub fn get_nearest(
        &self,
        position: Vector2<i32>,
        dist: f32,
    ) -> Vec<NearestNeighbour<f32, u64>> {
        self.hybrid_obstacles
            .nearest_n_within::<kiddo::SquaredEuclidean>(
                &[position.x as f32, position.y as f32],
                dist,
                NonZero::new(10000).unwrap(),
                false,
            )
    }

    pub fn is_obstruction_in_radius(&self, position: Vector2<i32>, radius: i32) -> bool {
        for i in -radius..=radius {
            for j in -radius..=radius {
                if self.is_obstructed(position + Vector2::new(i, j)) {
                    return true;
                }
            }
        }

        false
    }
}
