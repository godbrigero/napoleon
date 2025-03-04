use super::Pathfinding;
use crate::hybrid_grid::{GenericDynamicObject, HybridGrid};
use core::f64;
use jni::objects::{JClass, JFloatArray, JIntArray, JObject};
use jni::JNIEnv;
use nalgebra::{Vector2, Vector3};
use node::{Node, NodePickStyle};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;
use std::sync::Mutex;

pub mod node;

pub struct AStar {
    grid: Arc<Mutex<HybridGrid<Arc<dyn GenericDynamicObject>>>>,
    pick_style: NodePickStyle,
    node_step_size: i32,
    max_nodes_in_radius: i32,
    node_radius_search_radius: f64,
}

impl Pathfinding for AStar {
    fn new(hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>>) -> Self {
        Self {
            grid: Arc::new(Mutex::new(hybrid_grid)),
            pick_style: NodePickStyle::ALL,
            node_step_size: 1,
            max_nodes_in_radius: 1,
            node_radius_search_radius: 2.0,
        }
    }

    fn calculate_path(&self, start: Vector2<i32>, end: Vector2<i32>) -> Option<Vec<Vector2<i32>>> {
        let end_node = Node::new(end, None);
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut g_scores = HashMap::new();

        open_set.push(Node::new(start, None));
        closed_set.insert(start);
        g_scores.insert(start, 0.0);

        while let Some(current) = open_set.pop() {
            if current.get_position() == end_node.get_position() {
                return Some(self.reconstruct_path(current));
            }

            closed_set.remove(&current.get_position());

            for mut neighbor in current.get_positions_around(&self.pick_style, self.node_step_size)
            {
                let grid = self.grid.lock().unwrap();

                // Skip if neighbor is invalid
                if closed_set.contains(&neighbor.get_position())
                    || grid.is_outside_grid(neighbor.get_position())
                    || grid.is_obstructed(neighbor.get_position())
                    || grid.is_obstruction_in_radius(
                        neighbor.get_position(),
                        self.node_radius_search_radius as i32,
                    )
                {
                    continue;
                }

                // Only apply radius checks if we're not near the goal
                if neighbor.get_position() != end_node.get_position() {
                    let nearby_nodes = grid.get_nearest(
                        neighbor.get_position(),
                        self.node_radius_search_radius as f32,
                    );

                    if nearby_nodes.len() > self.max_nodes_in_radius as usize {
                        continue;
                    }
                }

                let g_score = *g_scores.get(&current.get_position()).unwrap_or(&f64::MIN);
                let tentative_g_cost = g_score + current.distance_to(&neighbor);
                let neighbor_g_cost = g_scores
                    .get(&neighbor.get_position())
                    .copied()
                    .unwrap_or(f64::INFINITY);

                if tentative_g_cost < neighbor_g_cost {
                    neighbor.set_cost(tentative_g_cost + end_node.distance_to(&neighbor));
                    g_scores.insert(neighbor.get_position(), tentative_g_cost);
                    open_set.push(neighbor);
                }
            }
        }

        None
    }
}

impl AStar {
    fn reconstruct_path(&self, head_node: Node) -> Vec<Vector2<i32>> {
        let mut output: Vec<Vector2<i32>> = Vec::new();
        output.push(head_node.get_position());

        let mut current_node: Option<Arc<Node>> = head_node.get_parent();
        while current_node.is_some() {
            let node = current_node.unwrap();
            output.push(node.get_position());
            current_node = node.get_parent()
        }

        output.reverse();
        output
    }

    pub fn get_grid(&mut self) -> &mut Arc<Mutex<HybridGrid<Arc<dyn GenericDynamicObject>>>> {
        return &mut self.grid;
    }

    pub fn build(
        hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>>,
        pick_style: NodePickStyle,
        node_step_size: i32,
        max_nodes_in_range: i32,
        node_radius_search_radius: f64,
    ) -> Self {
        Self {
            grid: Arc::new(Mutex::new(hybrid_grid)),
            pick_style,
            node_step_size,
            max_nodes_in_radius: max_nodes_in_range,
            node_radius_search_radius,
        }
    }
}
