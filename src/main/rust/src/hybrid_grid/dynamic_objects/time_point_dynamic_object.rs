use nalgebra::{Matrix3, Matrix4, Rotation3, Unit, Vector2, Vector3};

use crate::hybrid_grid::{math, GenericDynamicObject};

use super::ObjectDimensions;

pub struct DynamicObject<D: ObjectDimensions> {
    transformation_matrix: Matrix3<f64>,
    velocity: Vector2<f64>,
    acceleration: Vector2<f64>,
    variance_coef: f64,
    dimensions: D,
}

impl<D: ObjectDimensions> DynamicObject<D> {
    pub fn new(
        direction_vector: Vector2<f64>,
        position_center: Vector2<f64>,
        velocity: Vector2<f64>,
        acceleration: Vector2<f64>,
        variance_coef: f64,
        dimensions: D,
    ) -> Self {
        Self {
            transformation_matrix: math::construct_transformation_matrix(
                direction_vector,
                position_center,
            ),
            velocity,
            acceleration,
            variance_coef,
            dimensions,
        }
    }

    pub fn get_position(&self) -> Vector2<f64> {
        self.transformation_matrix.column(2).xy()
    }

    pub fn get_velocity(&self) -> Vector2<f64> {
        self.velocity
    }

    pub fn get_acceleration(&self) -> Vector2<f64> {
        self.acceleration
    }

    pub fn get_variance_coef(&self) -> f64 {
        self.variance_coef
    }

    pub fn get_dimensions(&self) -> &D {
        &self.dimensions
    }
}

impl<D: ObjectDimensions> GenericDynamicObject for DynamicObject<D> {
    fn calculate_transformation_matrix_at(&self, time: f64) -> Matrix3<f64> {
        let mut cloned_transformation_matrix = self.transformation_matrix.clone();

        let position = self.get_position()
            + self.get_velocity() * (time / 1000.0)
            + self.get_acceleration() * (0.5 * (time / 1000.0) * (time / 1000.0));

        cloned_transformation_matrix.set_column(2, &position.push(1.0));

        cloned_transformation_matrix
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestObjectDimensions;

    impl TestObjectDimensions {
        pub fn new() -> Self {
            return Self {};
        }
    }
    impl ObjectDimensions for TestObjectDimensions {
        fn contains_point(&self, point: Vector2<f64>) -> bool {
            true
        }
    }

    #[test]
    fn test_calculate_transformation_matrix_at() {
        let object = DynamicObject::new(
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 0.0),
            0.0,
            TestObjectDimensions::new(),
        );

        let transformation_matrix = object.calculate_transformation_matrix_at(1000.0);
        assert_eq!(transformation_matrix.column(2).xy(), Vector2::new(1.0, 0.0));
    }
}
