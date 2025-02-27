use super::Pathfinding;
use crate::hybrid_grid::{GenericDynamicObject, HybridGrid};
use nalgebra::{Vector2, Vector3};
use node::{Node, NodePickStyle};
use std::sync::Arc;
use std::time::Instant;
use std::{
    collections::{BinaryHeap, HashMap, HashSet},
    rc::Rc,
};

mod node;

pub struct AStar {
    grid: HybridGrid<Arc<dyn GenericDynamicObject>>,
    pick_style: NodePickStyle,
    node_step_size: i32,
    max_nodes_in_radius: i32,
    node_radius_search_radius: f64,
}

impl Pathfinding for AStar {
    fn new(hybrid_grid: HybridGrid<Arc<dyn GenericDynamicObject>>) -> Self {
        Self {
            grid: hybrid_grid,
            pick_style: NodePickStyle::ALL,
            node_step_size: 1,
            max_nodes_in_radius: 1,
            node_radius_search_radius: 2.0,
        }
    }

    fn calculate_path(&self, start: Vector2<i32>, end: Vector2<i32>) -> Option<Vec<Vector2<i32>>> {
        let end_node = Node::new(end, None);
        let mut cur = 0;

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashSet<Vector2<i32>> = HashSet::new();
        let mut g_scores: HashMap<Vector2<i32>, f64> = HashMap::new();

        open_set.push(Node::new(start, None));
        closed_set.insert(start);
        g_scores.insert(start, 0.0);

        while !open_set.is_empty() {
            let current = open_set.pop().unwrap();
            if current.get_position() == end {
                return Some(self.reconstruct_path(current));
            }

            closed_set.remove(&current.get_position());
            for mut neighbor in current.get_positions_around(&self.pick_style, self.node_step_size)
            {
                if closed_set.contains(&neighbor.get_position())
                    || self.grid.is_outside_grid(neighbor.get_position())
                    || self.grid.is_obstructed(neighbor.get_position())
                {
                    continue;
                }

                let dynamic_object_list =
                    self.grid.get_dynamic_object_transformation_matrices_at(0.0);

                let tentative_g_cost = *g_scores.get(&current.get_position()).unwrap()
                    + current.distance_to(&neighbor);
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

            cur += 1;
        }

        None
    }
}

impl AStar {
    fn reconstruct_path(&self, head_node: Node) -> Vec<Vector2<i32>> {
        let mut output: Vec<Vector2<i32>> = Vec::new();
        output.push(head_node.get_position());

        let mut current_node: Option<Rc<Node>> = head_node.get_parent();
        while current_node.is_some() {
            let node = current_node.unwrap();
            output.push(node.get_position());
            current_node = node.get_parent()
        }

        output.reverse();
        output
    }
}
