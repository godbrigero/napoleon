use nalgebra::{Matrix3, Matrix4, Rotation3, Unit, Vector2, Vector3};

use super::math;

pub trait ObjectDimensions {
    /// Returns true if the point is inside the object.
    /// Note: the point is in the local coordinate system of the object. (T_point_object = T_object_world * T_point_world)
    fn contains_point(&self, point: Vector2<f64>) -> bool;
}

pub struct DynamicObject<D: ObjectDimensions> {
    transformation_matrix: Matrix3<f64>,
    velocity: Vector2<f64>,
    variance_coef: f64,
    dimensions: D,
}

impl<D: ObjectDimensions> DynamicObject<D> {
    pub fn new(
        direction_vector: Vector2<f64>,
        position_center: Vector2<f64>,
        velocity: Vector2<f64>,
        variance_coef: f64,
        dimensions: D,
    ) -> Self {
        Self {
            transformation_matrix: math::construct_transformation_matrix(
                direction_vector,
                position_center,
            ),
            velocity,
            variance_coef,
            dimensions,
        }
    }

    pub fn get_position(&self) -> Vector2<f64> {
        self.transformation_matrix.column(3).xy()
    }

    pub fn get_velocity(&self) -> Vector2<f64> {
        self.velocity
    }

    pub fn get_variance_coef(&self) -> f64 {
        self.variance_coef
    }

    pub fn get_dimensions(&self) -> &D {
        &self.dimensions
    }

    pub fn calculate_transformation_matrix_at(&self, time: f64) -> Matrix3<f64> {
        let mut cloned_transformation_matrix = self.transformation_matrix.clone();
        let position = self.get_position() + self.get_velocity() * time;
        cloned_transformation_matrix.set_column(2, &position.push(1.0));

        cloned_transformation_matrix
    }
}
