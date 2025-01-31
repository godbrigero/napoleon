use kiddo::{KdTree, NearestNeighbour, SquaredEuclidean};
use nalgebra::{Matrix, Vector, Vector3};
use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::num::NonZeroUsize;
use std::ops::Deref;

pub struct DynamicObject {
    center_position: Vector3<f64>,
    velocity: Vector3<f64>,
    acceleration: Vector3<f64>,
    variance_coef: f64, // defines how far you should avoid this object. Lower speed = lower min distance.
}

impl DynamicObject {
    pub fn new(
        center_position: Vector3<f64>,
        velocity: Vector3<f64>,     // m/ms
        acceleration: Vector3<f64>, // m/ms^2
        variance_coef: f64,
    ) -> DynamicObject {
        Self {
            center_position,
            velocity,
            acceleration,
            variance_coef,
        }
    }

    pub fn is_point_in_front(&self, position: Vector3<f64>) -> bool {
        let velocity_direction = self.velocity.normalize();

        // Create a perpendicular plane by using the point-normal form
        // where the velocity is the normal vector and center_position is the point
        let signed_distance = velocity_direction.dot(&(position - self.center_position));

        // Calculate the projection of the point onto the plane
        let projection = position - signed_distance * velocity_direction;

        // Calculate the distance from the projection to the center position on the plane
        let lateral_distance = (projection - self.center_position).norm();

        // The plane width is proportional to the minimum distance
        let plane_width = self.variance_coef * self.velocity.norm();

        // Check if point is:
        // 1. in front of the perpendicular plane
        // 2. within the minimum distance
        // 3. within the lateral bounds of the plane
        let distance = (self.center_position - position).norm();
        let min_distance = self.variance_coef * self.velocity.norm();

        signed_distance > 0.0 && distance < min_distance && lateral_distance < plane_width
    }

    pub fn is_point_in_range(&self, position: Vector3<f64>) -> bool {
        let distance = (self.center_position - position).norm();
        distance < self.variance_coef * self.velocity.norm()
    }

    /// The time should be in ms!
    pub fn estimate_position_time(&self, time: f64) -> Vector3<f64> {
        self.center_position + self.velocity * time + self.acceleration * time * time / 2.0
    }
}

#[derive(Debug, Clone)]
pub struct HashableVector3(Vector3<f64>);

impl PartialEq for HashableVector3 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl From<Vector3<f64>> for HashableVector3 {
    fn from(value: Vector3<f64>) -> Self {
        Self(value)
    }
}

impl Eq for HashableVector3 {}

impl Hash for HashableVector3 {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.0.x.to_bits().hash(state);
        self.0.y.to_bits().hash(state);
        self.0.z.to_bits().hash(state);
    }
}

impl Deref for HashableVector3 {
    type Target = Vector3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub struct HybridGrid<'a> {
    filled_points: HashSet<HashableVector3>,
    kd_tree: KdTree<f64, 3>,
    dynamic_objects: Vec<&'a DynamicObject>,
}

impl<'a> HybridGrid<'a> {
    pub fn new(
        filled_points: HashSet<HashableVector3>,
        dynamic_objects: Vec<&'a DynamicObject>,
    ) -> HybridGrid<'a> {
        let mut kd_tree: KdTree<f64, 3> = KdTree::new();
        for (index, point) in filled_points.iter().enumerate() {
            let vec = &point.0;
            kd_tree.add(&[vec.x, vec.y, vec.z], 100);
        }

        HybridGrid {
            filled_points,
            kd_tree,
            dynamic_objects,
        }
    }

    pub fn contains(&self, point: &Vector3<f64>) -> bool {
        self.filled_points.contains(&HashableVector3(point.clone()))
    }

    pub fn remove(&mut self, point: &Vector3<f64>) {
        self.filled_points.remove(&HashableVector3(point.clone()));
    }

    pub fn get_nearest_in_range(
        &self,
        center_point: &Vector3<f64>,
        range: f64,
    ) -> Vec<NearestNeighbour<f64, u64>> {
        self.kd_tree.nearest_n_within::<SquaredEuclidean>(
            &[center_point.x, center_point.y, center_point.z],
            range,
            NonZeroUsize::new(100).unwrap(),
            true,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hybrid_grid_creation() {
        let points = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            Vector3::new(7.0, 8.0, 9.0),
        ];
        let filled_points: HashSet<HashableVector3> =
            points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points.clone(), vec![]);

        assert_eq!(grid.filled_points.len(), filled_points.len());
    }

    #[test]
    fn test_contains_point() {
        let points = vec![Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0)];
        let filled_points: HashSet<HashableVector3> =
            points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points, vec![]);

        assert!(grid.contains(&Vector3::new(1.0, 2.0, 3.0)));
        assert!(!grid.contains(&Vector3::new(7.0, 8.0, 9.0)));
    }

    #[test]
    fn test_remove_point() {
        let points = vec![Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0)];
        let mut filled_points: HashSet<HashableVector3> =
            points.into_iter().map(HashableVector3).collect();
        let mut grid = HybridGrid::new(filled_points.clone(), vec![]);

        grid.remove(&Vector3::new(1.0, 2.0, 3.0));

        assert!(!grid.contains(&Vector3::new(1.0, 2.0, 3.0)));
        assert!(grid.contains(&Vector3::new(4.0, 5.0, 6.0)));
    }

    #[test]
    fn test_nearest_in_range() {
        let points = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            Vector3::new(7.0, 8.0, 9.0),
        ];
        let filled_points: HashSet<HashableVector3> =
            points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points, vec![]);

        let center = Vector3::new(2.0, 3.0, 4.0);
        let nearest = grid.get_nearest_in_range(&center, 5.0);

        assert_eq!(nearest.len(), 1);
        assert!(nearest[0].distance < 5.0);
    }
}
