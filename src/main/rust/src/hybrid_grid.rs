use std::{
    collections::{HashMap, HashSet},
    num::NonZero,
    sync::Arc,
};

use kiddo::{KdTree, NearestNeighbour};
use nalgebra::{Matrix3, Vector2};

pub mod math;

#[derive(Clone)]
pub struct UncertentyField {
    pub center: Vector2<f32>,
    pub radius: f32,
    pub intensity: f32,
}

#[derive(Clone)]
pub struct HybridGrid {
    size_x: i32,
    size_y: i32,
    center_x: i32,
    center_y: i32,
    square_size_meters: f32,

    static_obstacles: HashSet<Vector2<i32>>,

    hybrid_obstacles: KdTree<f32, 2>,

    uncertainty_defs: HashMap<u64, UncertentyField>,
    uncertainty_fields: KdTree<f32, 2>,
    max_field_radius: f32,
}

impl HybridGrid {
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
            hybrid_obstacles: KdTree::new(),
            uncertainty_defs: HashMap::new(),
            uncertainty_fields: KdTree::new(),
            max_field_radius: 0.0,
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

    pub fn push_static_obstacle(&mut self, obstacle: Vector2<i32>) {
        self.static_obstacles.insert(obstacle);
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

    pub fn get_nearest_hybrid(
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

    pub fn get_all_obstructions_in_radius(
        &self,
        position: Vector2<i32>,
        radius: i32,
    ) -> Vec<Vector2<i32>> {
        let mut obstructions = Vec::new();
        for i in -radius..=radius {
            for j in -radius..=radius {
                let check_pos = position + Vector2::new(i, j);
                if self.is_obstructed(check_pos) {
                    obstructions.push(check_pos);
                }
            }
        }

        obstructions
    }

    pub fn is_obstruction_in_radius(&self, position: Vector2<i32>, radius: i32) -> bool {
        return self.get_all_obstructions_in_radius(position, radius).len() != 0;
    }

    pub fn add_uncertainty_field(&mut self, center: Vector2<f32>, radius: f32, intensity: f32) {
        let position = (self.uncertainty_defs.len() + 1) as u64;
        if radius > self.max_field_radius {
            self.max_field_radius = radius;
        }

        self.uncertainty_fields
            .add(&[center.x as f32, center.y as f32], position);
        self.uncertainty_defs.insert(
            position,
            UncertentyField {
                center,
                radius,
                intensity,
            },
        );
    }

    pub fn clear_uncertenty_fields(&mut self) {
        self.uncertainty_defs.clear();
        self.uncertainty_fields = KdTree::new();
    }

    pub fn get_nearest_uncertainty_field(
        &self,
        position: Vector2<f32>,
    ) -> Option<(UncertentyField, f32)> {
        let output = self
            .uncertainty_fields
            .nearest_one::<kiddo::SquaredEuclidean>(&[position.x as f32, position.y as f32]);

        if !self.uncertainty_defs.contains_key(&output.item) {
            return None;
        }

        let field = self.uncertainty_defs.get(&output.item).unwrap();

        if field.radius < output.distance {
            return None;
        }

        Some((field.clone(), output.distance))
    }

    pub fn uncertainty_field_cost_ramping(
        &self,
        distance_cur: f32,
        distance_field: f32,
        intensity: f32,
    ) -> f32 {
        let distance_ratio = distance_cur / distance_field;
        let eased_ratio = 1.0 - (1.0 - distance_ratio).powi(2); // Quadratic easing out
        eased_ratio * intensity
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    #[test]
    fn test_grid_creation() {
        let grid = HybridGrid::new(100, 100, 1.0, vec![], 0, 0);
        assert_eq!(grid.get_square_size_meters(), 1.0);
        assert!(grid.get_static_obstacles().is_empty());
    }

    #[test]
    fn test_static_obstacles() {
        let mut grid = HybridGrid::new_raw(100, 100, 1.0, 0, 0);
        let obstacle = Vector2::new(5, 5);

        grid.push_static_obstacle(obstacle);
        assert!(grid.is_obstructed(obstacle));
        assert!(!grid.is_obstructed(Vector2::new(6, 6)));
    }

    #[test]
    fn test_grid_boundaries() {
        let grid = HybridGrid::new(10, 10, 1.0, vec![], 0, 0);

        assert!(grid.is_outside_grid(Vector2::new(6, 0)));
        assert!(grid.is_outside_grid(Vector2::new(0, 6)));
        assert!(!grid.is_outside_grid(Vector2::new(0, 0)));
    }

    #[test]
    fn test_hybrid_objects() {
        let mut grid = HybridGrid::new_raw(100, 100, 1.0, 0, 0);
        let object = [1.0, 1.0];

        grid.add_hybrid_object(&object);
        let nearest = grid.get_nearest_hybrid(Vector2::new(1, 1), 10.0);
        assert!(!nearest.is_empty());

        grid.clear_hybrid_objects();
        let nearest_after_clear = grid.get_nearest_hybrid(Vector2::new(1, 1), 10.0);
        assert!(nearest_after_clear.is_empty());
    }

    #[test]
    fn test_obstruction_in_radius() {
        let mut grid = HybridGrid::new_raw(100, 100, 1.0, 0, 0);
        grid.push_static_obstacle(Vector2::new(1, 1));

        assert!(grid.is_obstruction_in_radius(Vector2::new(0, 0), 2));
        assert!(!grid.is_obstruction_in_radius(Vector2::new(0, 0), 0));
    }

    #[test]
    fn test_uncertainty_fields() {
        let mut grid = HybridGrid::new_raw(100, 100, 1.0, 0, 0);
        let center = Vector2::new(1.0, 1.0);

        grid.add_uncertainty_field(center, 2.0, 0.5);
        let field = grid.get_nearest_uncertainty_field(center);

        assert!(field.is_some());
        let (field, distance) = field.unwrap();
        assert_eq!(field.radius, 2.0);
        assert_eq!(field.intensity, 0.5);
        assert_eq!(distance, 0.0);
        grid.clear_uncertenty_fields();
        assert!(grid.get_nearest_uncertainty_field(center).is_none());
    }
}
