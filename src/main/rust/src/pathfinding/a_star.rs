use super::Pathfinding;
use crate::hybrid_grid::{DynamicObject, HashableVector3, HybridGrid};
use nalgebra::Vector3;
use node::{Node, NodePickStyle};
use std::{
    collections::{BinaryHeap, HashMap, HashSet},
    rc::Rc,
};
use std::time::Instant;

mod node;

pub struct AStar<'a> {
    grid: HybridGrid<'a>,
    pick_style: NodePickStyle,
    node_step_size: i32,
    max_nodes_in_radius: i32,
    node_radius_search_radius: f64,
}

impl<'a> Pathfinding<'a> for AStar<'a> {
    fn new(
        dynamic_objects: Vec<&'a DynamicObject>,
        filled_points: HashSet<HashableVector3>,
    ) -> Self {
        Self {
            grid: HybridGrid::new(filled_points.clone(), dynamic_objects),
            pick_style: NodePickStyle::ALL,
            node_step_size: 1,
            max_nodes_in_radius: 1,
            node_radius_search_radius: 2.0,
        }
    }

    fn calculate_path(&self, start: Vector3<i32>, end: Vector3<i32>) -> Option<Vec<Vector3<i32>>> {
        let end_node = Node::new(end, None);
        let mut cur = 0;

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashSet<Vector3<i32>> = HashSet::new();
        let mut g_scores: HashMap<Vector3<i32>, f64> = HashMap::new();

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
                    || self.grid.contains(&neighbor.get_position())
                    || self
                        .grid
                        .get_nearest_in_range(
                            &neighbor.get_position(),
                            self.node_radius_search_radius,
                        )
                        .len()
                        > self.max_nodes_in_radius as usize
                {
                    continue;
                }

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

impl<'a> AStar<'a> {
    fn reconstruct_path(&self, head_node: Node) -> Vec<Vector3<i32>> {
        let mut output: Vec<Vector3<i32>> = Vec::new();
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
