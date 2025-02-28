use std::sync::Arc;

use hybrid_grid::{GenericDynamicObject, HybridGrid};
use j4rs::jni_sys::{jfloat, jfloatArray, jint, jintArray};
use jni::{
    objects::{JClass, JFloatArray, JIntArray},
    JNIEnv,
};
use nalgebra::Vector2;
use pathfinding::{a_star::AStar, Pathfinding};

pub mod hybrid_grid;
pub mod jni_util_extended;
pub mod pathfinding;
pub mod time_structures;
pub mod trajectory_maker;

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_calculate<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    static_obstacles: JIntArray<'a>,
    size_x_y: JIntArray<'a>,
    center_x_y: JIntArray<'a>,
    square_size_meters: jfloat,
    robot_avg_velocity_x_m_s: JFloatArray<'a>,
    start_x_y: JIntArray<'a>,
    end_x_y: JIntArray<'a>,
    finder_relative_w_h: JFloatArray<'a>,
) -> JIntArray<'a> {
    let parsed_static_bstacles = jni_util_extended::jintarray_to_vec(&env, static_obstacles);
    let mut static_obstacles_transformed = Vec::new();
    for i in (0..parsed_static_bstacles.len()).step_by(2) {
        static_obstacles_transformed.push(Vector2::new(
            parsed_static_bstacles[i],
            parsed_static_bstacles[i + 1],
        ));
    }

    let field_dimensions: Vector2<i32> =
        jni_util_extended::from_jint_array_to_vector2_int(&env, size_x_y);
    let field_center: Vector2<i32> =
        jni_util_extended::from_jint_array_to_vector2_int(&env, center_x_y);
    let sq_size_meters = square_size_meters as f32;
    let robot_avg_velocity: Vector2<f32> =
        jni_util_extended::from_jfloat_array_to_vector2_float(&env, robot_avg_velocity_x_m_s);

    let start = jni_util_extended::from_jint_array_to_vector2_int(&env, start_x_y);
    let end = jni_util_extended::from_jint_array_to_vector2_int(&env, end_x_y);
    let robot_dimensions = jni_util_extended::jfloatarray_to_vec(&env, finder_relative_w_h);

    let hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>> = HybridGrid::new(
        field_dimensions.x,
        field_dimensions.y,
        sq_size_meters,
        static_obstacles_transformed,
        field_center.x,
        field_center.y,
    );

    let a_star_pathfinder = AStar::new(hybrid_grid);
    let calculated_path = a_star_pathfinder.calculate_path(start, end);
    if calculated_path.is_none() {
        return env.new_int_array(0).unwrap();
    }

    let calculated_path = calculated_path.unwrap();
    let mut position_vec = Vec::new();
    for i in 0..calculated_path.len() {
        let node_pos = calculated_path[i] as Vector2<i32>;
        position_vec.push(node_pos.x);
        position_vec.push(node_pos.y);
    }

    let mut jint_array = env
        .new_int_array(position_vec.len() as i32)
        .expect("Failed to create Java int array");
    env.set_int_array_region(&jint_array, 0, &position_vec)
        .expect("Failed to copy values into Java int array");

    return jint_array;
}
