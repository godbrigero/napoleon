use hybrid_grid::HybridGrid;
use jni::{
    objects::{JClass, JFloatArray, JIntArray},
    sys::{jfloat, jint, jlong},
    JNIEnv,
};
use nalgebra::Vector2;
use pathfinding::{
    a_star::{node::NodePickStyle, AStar},
    NodeRadiusSearch, Pathfinding,
};

pub mod hybrid_grid;
pub mod jni_util_extended;
pub mod pathfinding;
pub mod time_structures;
pub mod trajectory_maker;

fn get_astar<'a>(env: &mut JNIEnv<'a>, obj: JClass<'a>) -> &'a mut AStar {
    let ptr = env
        .get_field(obj, "nativePtr", "J")
        .expect("Field not found")
        .j()
        .unwrap();

    let astar = unsafe { &mut *(ptr as *mut AStar) };
    astar
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_calculate<'a>(
    mut env: JNIEnv<'a>,
    obj: JClass<'a>,
    start_x_y: JIntArray<'a>,
    end_x_y: JIntArray<'a>,
) -> JIntArray<'a> {
    let start = jni_util_extended::from_jint_array_to_vector2_int(&env, start_x_y);
    let end = jni_util_extended::from_jint_array_to_vector2_int(&env, end_x_y);

    let astar = get_astar(&mut env, obj);

    let path = astar.calculate_path(start, end);
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
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_addHybridObjects<'a>(
    mut env: JNIEnv<'a>,
    obj: JClass<'a>,
    objects: JFloatArray<'a>,
) {
    let astar = get_astar(&mut env, obj);
    let hybrid_grid = astar.get_grid();

    let objects = jni_util_extended::jfloatarray_to_vec(&env, objects);
    for i in (0..objects.len()).step_by(2) {
        hybrid_grid.add_hybrid_object(&[objects[i], objects[i + 1]]);
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_clearHybridObjects<'a>(
    mut env: JNIEnv<'a>,
    obj: JClass<'a>,
) {
    let astar = get_astar(&mut env, obj);
    let hybrid_grid = astar.get_grid();
    hybrid_grid.clear_hybrid_objects();
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_clearUncertentyFields<'a>(
    mut env: JNIEnv<'a>,
    obj: JClass<'a>,
) {
    let astar = get_astar(&mut env, obj);
    let hybrid_grid = astar.get_grid();
    hybrid_grid.clear_uncertenty_fields();
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_addUncertentyField<'a>(
    mut env: JNIEnv<'a>,
    obj: JClass<'a>,
    center: JFloatArray<'a>,
    radius: jfloat,
    intensity: jfloat,
) {
    let astar = get_astar(&mut env, obj);
    let hybrid_grid = astar.get_grid();
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
    finder_relative_w_h: JFloatArray<'a>,
    do_absolute_discard: jint,
    avg_distance_min_discard_threshold: jfloat,
    avg_distance_cost: jfloat,
) -> jlong {
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

    let hybrid_grid = HybridGrid::new(
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

    let astar = AStar::build(
        hybrid_grid,
        node_pick_style,
        NodeRadiusSearch {
            node_radius_search_radius_squared: distance * distance,
            do_absolute_discard: do_absolute_discard != 0,
            avg_distance_min_discard_threshold: avg_distance_min_discard_threshold as f32,
            avg_distance_cost: avg_distance_cost as f32,
        },
    );

    let boxed_astar = Box::new(astar);
    let ptr = Box::into_raw(boxed_astar);

    ptr as jlong
}
