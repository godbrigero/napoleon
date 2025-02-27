use nalgebra::Matrix3;

use crate::hybrid_grid::GenericDynamicObject;

use super::ObjectDimensions;

pub struct DynamicObject<D: ObjectDimensions> {
    transformation_matrix: Matrix3<f64>,
    dimensions: D,
}

impl<D: ObjectDimensions> DynamicObject<D> {}

impl<D: ObjectDimensions> GenericDynamicObject for DynamicObject<D> {
    fn calculate_transformation_matrix_at(&self, time_ms: f64) -> Matrix3<f64> {
        self.transformation_matrix
    }
}
