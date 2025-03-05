use std::sync::{Arc, Mutex};

use hybrid_grid::{GenericDynamicObject, HybridGrid};
use jni::{
    objects::{JClass, JFloatArray, JIntArray},
    sys::{jfloat, jint},
    JNIEnv,
};
use nalgebra::Vector2;
use pathfinding::{
    a_star::{node::NodePickStyle, AStar},
    Pathfinding,
};

pub mod hybrid_grid;
pub mod jni_util_extended;
pub mod pathfinding;
pub mod time_structures;
pub mod trajectory_maker;

lazy_static::lazy_static! {
    static ref STORED_RUST_INSTANCE: Mutex<Option<AStar>> = Mutex::new(None);
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_calculate<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    start_x_y: JIntArray<'a>,
    end_x_y: JIntArray<'a>,
) -> JIntArray<'a> {
    let start = jni_util_extended::from_jint_array_to_vector2_int(&env, start_x_y);
    let end = jni_util_extended::from_jint_array_to_vector2_int(&env, end_x_y);

    let stored_instance = STORED_RUST_INSTANCE.lock().unwrap();
    let stored_instance = stored_instance
        .as_ref()
        .expect("You have not yet initialized the pathfinder!");

    let path = stored_instance.calculate_path(start, end);
    if path.is_none() {
        return env
            .new_int_array(0)
            .expect("Failed to create Java int array");
    }

    let path = path.unwrap();

    let mut position_vec = Vec::new();
    for i in 0..path.len() {
        let node_pos = path[i] as Vector2<i32>;
        position_vec.push(node_pos.x);
        position_vec.push(node_pos.y);
    }

    let jint_array = env
        .new_int_array(position_vec.len() as i32)
        .expect("Failed to create Java int array");
    env.set_int_array_region(&jint_array, 0, &position_vec)
        .expect("Failed to copy values into Java int array");

    return jint_array;
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_HybridGrid_addHybridObjects<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    objects: JFloatArray<'a>,
) {
    let mut astar = STORED_RUST_INSTANCE.lock().unwrap();
    let mut hybrid_grid = astar.as_mut().unwrap().get_grid().lock().unwrap();

    let objects = jni_util_extended::jfloatarray_to_vec(&env, objects);
    for i in (0..objects.len()).step_by(2) {
        hybrid_grid.add_hybrid_object(&[objects[i], objects[i + 1]]);
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_HybridGrid_clearHybridObjects<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
) {
    let mut astar = STORED_RUST_INSTANCE.lock().unwrap();
    let mut hybrid_grid = astar.as_mut().unwrap().get_grid().lock().unwrap();
    hybrid_grid.clear_hybrid_objects();
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_HybridGrid_clearUncertentyFields<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
) {
    let mut astar = STORED_RUST_INSTANCE.lock().unwrap();
    let mut hybrid_grid = astar.as_mut().unwrap().get_grid().lock().unwrap();
    hybrid_grid.clear_uncertenty_fields();
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_HybridGrid_addUncertentyField<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    center: JFloatArray<'a>,
    radius: jfloat,
    intensity: jfloat,
) {
    let mut astar = STORED_RUST_INSTANCE.lock().unwrap();
    let mut hybrid_grid = astar.as_mut().unwrap().get_grid().lock().unwrap();
    let field_center: Vector2<f32> =
        jni_util_extended::from_jfloat_array_to_vector2_float(&env, center);
    hybrid_grid.add_uncertenty_field(field_center, radius, intensity);
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_initialize<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    static_obstacles: JIntArray<'a>,
    size_x_y: JIntArray<'a>,
    center_x_y: JIntArray<'a>,
    square_size_meters: jfloat,
    node_pick_style: jint,
    max_nodes_in_range: jint,
    finder_relative_w_h: JFloatArray<'a>,
) {
    let node_pick_style = if node_pick_style == 0 {
        NodePickStyle::ALL
    } else {
        NodePickStyle::SIDES
    };

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

    let hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>> = HybridGrid::new(
        field_dimensions.x,
        field_dimensions.y,
        sq_size_meters,
        static_obstacles_transformed,
        field_center.x,
        field_center.y,
    );

    let robot_dimensions =
        jni_util_extended::from_jfloat_array_to_vector2_float(&env, finder_relative_w_h);
    let distance = robot_dimensions.magnitude();

    let mut stored_instance = STORED_RUST_INSTANCE.lock().unwrap();
    if stored_instance.is_none() {
        *stored_instance = Some(AStar::build(
            hybrid_grid,
            node_pick_style,
            1,
            max_nodes_in_range.into(),
            (distance * distance + 0.2).into(),
        ));
    }
}
