use nalgebra::Vector2;

pub mod precalculated_dynamic_object;
pub mod time_point_dynamic_object;
pub mod trajectory_state;

pub trait ObjectDimensions: Send + Sync {
    /// Returns true if the point is inside the object.
    /// Note: the point is in the local coordinate system of the object. (T_point_object = T_object_world * T_point_world)
    fn contains_point(&self, point: Vector2<f64>) -> bool;
}
