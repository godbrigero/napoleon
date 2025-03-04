use jni::{
    objects::{GlobalRef, JClass, JIntArray, JObject},
    sys::{jdouble, jint, jintArray, jobject},
    JNIEnv,
};
use nalgebra::{Vector2, Vector3};
use std::{
    collections::HashMap,
    rc::Rc,
    sync::atomic::{AtomicUsize, Ordering},
    sync::{Arc, Mutex},
};

use crate::jni_util_extended;

lazy_static::lazy_static! {
    static ref STORED_NODES: Mutex<HashMap<usize, (Node, GlobalRef)>> = Mutex::new(HashMap::new());
    static ref NODE_COUNTER: AtomicUsize = AtomicUsize::new(0);
}

#[derive(Clone)]
pub enum NodePickStyle {
    ALL,
    SIDES,
}

impl NodePickStyle {
    pub fn get_offsets(&self, step_size: i32) -> Vec<Vector3<i32>> {
        match self {
            NodePickStyle::ALL => {
                let mut offsets = Vec::new();
                for x in -1..=1 {
                    for y in -1..=1 {
                        if x == 0 && y == 0 {
                            continue;
                        }
                        offsets.push(Vector3::new(x * step_size, y * step_size, 0));
                    }
                }
                offsets
            }
            NodePickStyle::SIDES => {
                let mut offsets = Vec::new();
                for (x, y) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                    offsets.push(Vector3::new(x * step_size, y * step_size, 0));
                }
                offsets
            }
        }
    }
}

#[derive(Clone)]
pub struct Node {
    position: Vector2<i32>,
    cost: f64,
    parent: Option<Arc<Node>>,
    time_ms_since_initial: f64, // TODO: IMPLEMENT THIS
}

impl std::fmt::Display for Node {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Node {{ position: {:?}, cost: {} }}",
            self.position, self.cost
        )
    }
}

impl Node {
    pub fn new(position: Vector2<i32>, parent: Option<Arc<Node>>) -> Self {
        Self {
            position,
            cost: 0.0,
            parent,
            time_ms_since_initial: 0.0,
        }
    }

    pub fn get_position(&self) -> Vector2<i32> {
        return self.position;
    }

    pub fn get_parent(&self) -> Option<Arc<Node>> {
        return self.parent.clone();
    }

    pub fn get_positions_around(&self, pick_style: &NodePickStyle, step_size: i32) -> Vec<Node> {
        let mut return_vec = Vec::new();
        let self_rc = Arc::new(Node::new(self.position, self.parent.clone()));

        for i in pick_style.get_offsets(step_size) {
            return_vec.push(Node::new(self.position + i.xy(), Some(self_rc.clone())));
        }
        return_vec
    }

    pub fn distance_to(&self, other: &Node) -> f64 {
        let diff = self.position - other.position;
        ((diff.x as f64).powi(2) + (diff.y as f64).powi(2)).sqrt()
    }

    pub fn set_cost(&mut self, new_cost: f64) {
        self.cost = new_cost;
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.cost.partial_cmp(&self.cost).unwrap()
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

impl Eq for Node {}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_createNode<'a>(
    env: JNIEnv<'a>,
    _: JClass<'a>,
    position: JIntArray<'a>,
    java_node: JObject<'a>,
) -> jint {
    let pos = jni_util_extended::from_jint_array_to_vector2_int(&env, position);
    let node = Node::new(pos, None);

    // Create a global reference to the Java object
    let global_ref = env
        .new_global_ref(java_node)
        .expect("Failed to create global reference");

    let id = NODE_COUNTER.fetch_add(1, Ordering::SeqCst);
    let mut nodes = STORED_NODES.lock().unwrap();
    nodes.insert(id, (node, global_ref));

    id as jint
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_cleanupNode(
    _env: JNIEnv,
    _: JClass,
    node_id: jint,
) {
    let mut nodes = STORED_NODES.lock().unwrap();
    nodes.remove(&(node_id as usize));
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_connectNodes(
    _env: JNIEnv,
    _: JClass,
    parent_id: jint,
    child_id: jint,
) {
    let mut nodes = STORED_NODES.lock().unwrap();

    let parent_arc = nodes
        .get(&(parent_id as usize))
        .map(|(node, _)| Arc::new(node.clone()));

    if let (Some(parent), Some((child_node, _))) = (parent_arc, nodes.get_mut(&(child_id as usize)))
    {
        child_node.parent = Some(parent);
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_getNodeCost(
    _env: JNIEnv,
    _: JClass,
    node_id: jint,
) -> jdouble {
    let nodes = STORED_NODES.lock().unwrap();
    nodes
        .get(&(node_id as usize))
        .map(|(node, _)| node.cost)
        .unwrap_or(0.0)
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_setNodeCost(
    _env: JNIEnv,
    _: JClass,
    node_id: jint,
    cost: jdouble,
) {
    let mut nodes = STORED_NODES.lock().unwrap();
    if let Some((node, _)) = nodes.get_mut(&(node_id as usize)) {
        node.set_cost(cost);
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_getNodePosition(
    env: JNIEnv,
    _: JClass,
    node_id: jint,
) -> jintArray {
    let nodes = STORED_NODES.lock().unwrap();
    if let Some((node, _)) = nodes.get(&(node_id as usize)) {
        let pos = node.get_position();
        let arr = env.new_int_array(2).unwrap();
        env.set_int_array_region(&arr, 0, &[pos.x, pos.y]).unwrap();
        **arr
    } else {
        **env.new_int_array(0).unwrap()
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_getPositionsAround(
    env: JNIEnv,
    _: JClass,
    node_id: jint,
    pick_style: jint,
    step_size: jint,
) -> jintArray {
    let nodes = STORED_NODES.lock().unwrap();
    if let Some((node, _)) = nodes.get(&(node_id as usize)) {
        let style = if pick_style == 0 {
            NodePickStyle::ALL
        } else {
            NodePickStyle::SIDES
        };
        let positions = node.get_positions_around(&style, step_size);
        let mut pos_array = Vec::new();
        for pos in positions {
            pos_array.push(pos.get_position().x);
            pos_array.push(pos.get_position().y);
        }
        let arr = env.new_int_array(pos_array.len() as i32).unwrap();
        env.set_int_array_region(&arr, 0, &pos_array).unwrap();
        **arr
    } else {
        **env.new_int_array(0).unwrap()
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_AStarPathfinder_distanceBetweenNodes(
    _env: JNIEnv,
    _: JClass,
    node_id1: jint,
    node_id2: jint,
) -> jdouble {
    let nodes = STORED_NODES.lock().unwrap();
    if let (Some((node1, _)), Some((node2, _))) = (
        nodes.get(&(node_id1 as usize)),
        nodes.get(&(node_id2 as usize)),
    ) {
        node1.distance_to(node2)
    } else {
        0.0
    }
}

#[no_mangle]
pub extern "system" fn Java_org_pwrup_napoleon_bridge_Node_getParent<'a>(
    mut env: JNIEnv<'a>,
    _: JClass<'a>,
    node_id: jint,
) -> JObject<'a> {
    let nodes = STORED_NODES.lock().unwrap();
    if let Some((node, _)) = nodes.get(&(node_id as usize)) {
        if let Some(parent) = &node.parent {
            // Find the parent's ID and create a new Java Node object
            for (id, (potential_parent, _)) in nodes.iter() {
                if Arc::ptr_eq(parent, &Arc::new(potential_parent.clone())) {
                    let node_class = env.find_class("org/pwrup/napoleon/bridge/Node").unwrap();
                    let pos = potential_parent.get_position();
                    let pos_arr = env.new_int_array(2).unwrap();
                    env.set_int_array_region(&pos_arr, 0, &[pos.x, pos.y])
                        .unwrap();

                    return env
                        .new_object(node_class, "(II)V", &[pos.x.into(), pos.y.into()])
                        .unwrap();
                }
            }
        }
    }

    JObject::null()
}
