use nalgebra::{Matrix2, Matrix3, OVector, RowVector3, Vector2, U2};

/// # Returns
/// The distance between the two points
/// # Arguments
/// * `point1` - The first point
/// * `point2` - The second point
pub fn get_distance_between_points(point1: &OVector<f64, U2>, point2: &OVector<f64, U2>) -> f64 {
    (point1 - point2).magnitude()
}

pub fn get_distance_in_front(
    object_transformation_matrix_2d: &Matrix3<f64>,
    point: Vector2<f64>,
) -> f64 {
    let forward_vector = get_global_direction_vector(object_transformation_matrix_2d);
    forward_vector.dot(&(point - object_transformation_matrix_2d.column(2).xy()))
}

pub fn get_global_direction_vector(transformation_matrix: &Matrix3<f64>) -> Vector2<f64> {
    get_rotation_matrix(transformation_matrix) * transformation_matrix.column(0).xy()
}

pub fn get_rotation_matrix(transformation_matrix: &Matrix3<f64>) -> Matrix2<f64> {
    Matrix2::new(
        transformation_matrix.column(0).xy().x,
        transformation_matrix.column(0).xy().y,
        transformation_matrix.column(1).xy().x,
        transformation_matrix.column(1).xy().y,
    )
}

pub fn construct_transformation_matrix(
    direction_vector: Vector2<f64>,
    position_center: Vector2<f64>,
) -> Matrix3<f64> {
    if direction_vector.magnitude() > 0.0 {
        let forward = direction_vector.normalize();
        let right = Vector2::new(forward.y, -forward.x);

        Matrix3::from_rows(&[
            RowVector3::new(right.x, right.y, position_center.x),
            RowVector3::new(forward.x, forward.y, position_center.y),
            RowVector3::new(0.0, 0.0, 1.0),
        ])
    } else {
        Matrix3::identity()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::RowVector3;

    use super::*;

    #[test]
    fn test_construct_transformation_matrix() {
        let direction_vector = Vector2::new(1.0, 0.0); // Facing right (+x)
        let position_center = Vector2::new(0.0, 0.0);
        let transformation_matrix =
            construct_transformation_matrix(direction_vector, position_center);

        let correct_matrix = Matrix3::from_rows(&[
            RowVector3::new(0.0, -1.0, 0.0),
            RowVector3::new(1.0, 0.0, 0.0),
            RowVector3::new(0.0, 0.0, 1.0),
        ]);

        assert_eq!(transformation_matrix, correct_matrix);
    }

    #[test]
    fn test_distance_in_front() {
        let direction_vector = Vector2::new(1.0, 0.0); // Facing right (+x)
        let position_center = Vector2::new(0.0, 0.0);
        let transformation_matrix =
            construct_transformation_matrix(direction_vector, position_center);

        let point = Vector2::new(2.0, 0.0);
        let distance = get_distance_in_front(&transformation_matrix, point);
        assert_eq!(distance, 2.0);
    }
}
