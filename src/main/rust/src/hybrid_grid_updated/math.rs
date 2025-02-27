use nalgebra::{Matrix3, Rotation2, Unit, Vector2, Vector3};

pub fn get_distance_between_points(point1: Vector2<f64>, point2: Vector2<f64>) -> f64 {
    (point1 - point2).magnitude()
}

pub fn get_distance_in_front(transformation_matrix: &Matrix3<f64>, point: Vector2<f64>) -> f64 {
    let point_in_local = transformation_matrix * point.push(1.0);
    let forward = transformation_matrix.fixed_view::<2, 1>(0, 1).into_owned();
    forward.dot(&point_in_local.xy())
}

pub fn construct_transformation_matrix(
    direction_vector: Vector2<f64>,
    position_center: Vector2<f64>,
) -> Matrix3<f64> {
    let angle = direction_vector.y.atan2(direction_vector.x); // Compute rotation angle

    Matrix3::from_columns(&[
        Vector3::new(f64::cos(angle), f64::sin(angle), 0.0), // Right direction (+x)
        Vector3::new(-f64::sin(angle), f64::cos(angle), 0.0), // Forward direction (+y)
        Vector3::new(position_center.x, position_center.y, 1.0), // Translation
    ])
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    #[test]
    fn test_construct_transformation_matrix() {
        let direction_vector = Vector2::new(1.0, 0.0); // Facing right (+x)
        let position_center = Vector2::new(0.0, 0.0);
        let transformation_matrix =
            construct_transformation_matrix(direction_vector, position_center);

        let correct_matrix = Matrix3::from_columns(&[
            Vector3::new(1.0, 0.0, 0.0), // Right (+x)
            Vector3::new(0.0, 1.0, 0.0), // Forward (+y)
            Vector3::new(0.0, 0.0, 1.0), // Translation (origin)
        ]);

        assert_eq!(transformation_matrix, correct_matrix);
    }
}
