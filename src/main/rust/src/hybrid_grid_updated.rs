use nalgebra::Vector3;

pub mod dynamic_objects;
pub mod math;

pub trait DynamicObject {
    fn get_position_f64(&self, time_ms_since_initial: f64) -> Vector3<f64>;
}

pub struct HybridGrid<D: DynamicObject> {
    size_x: i32,
    size_y: i32,
    square_size_meters: f32,

    static_obstacles: Vec<(i32, i32)>,
    dynamic_objects: Vec<D>,
}
