use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::num::NonZeroUsize;
use nalgebra::Vector3;
use kiddo::{KdTree, NearestNeighbour, SquaredEuclidean};

#[derive(Debug, Clone)]
pub struct HashableVector3(Vector3<f64>);

impl PartialEq for HashableVector3 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
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

pub struct HybridGrid {
    filled_points: HashSet<HashableVector3>,
    kd_tree: KdTree<f64, 3>,
}

impl HybridGrid {
    pub fn new(filled_points: HashSet<HashableVector3>) -> HybridGrid {
        let mut kd_tree: KdTree<f64, 3> = KdTree::new();
        for (index, point) in filled_points.iter().enumerate() {
            let vec = &point.0;
            kd_tree.add(&[vec.x, vec.y, vec.z], 100);
        }

        HybridGrid { filled_points, kd_tree }
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
        self.kd_tree
            .nearest_n_within::<SquaredEuclidean>(
                &[center_point.x, center_point.y, center_point.z],
                range,
                NonZeroUsize::new(100).unwrap(), true,
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
        let filled_points: HashSet<HashableVector3> = points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points.clone());

        assert_eq!(grid.filled_points.len(), filled_points.len());
    }

    #[test]
    fn test_contains_point() {
        let points = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
        ];
        let filled_points: HashSet<HashableVector3> = points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points);

        assert!(grid.contains(&Vector3::new(1.0, 2.0, 3.0)));
        assert!(!grid.contains(&Vector3::new(7.0, 8.0, 9.0)));
    }

    #[test]
    fn test_remove_point() {
        let points = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
        ];
        let mut filled_points: HashSet<HashableVector3> = points.into_iter().map(HashableVector3).collect();
        let mut grid = HybridGrid::new(filled_points.clone());

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
        let filled_points: HashSet<HashableVector3> = points.into_iter().map(HashableVector3).collect();
        let grid = HybridGrid::new(filled_points);

        let center = Vector3::new(2.0, 3.0, 4.0);
        let nearest = grid.get_nearest_in_range(&center, 5.0);

        assert_eq!(nearest.len(), 1);
        assert!(nearest[0].distance < 5.0);
    }
}
