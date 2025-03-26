use nalgebra::{Vector2, Vector3};
use std::sync::Arc;

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
    g_score: f64,
    parent: Option<Arc<Node>>,
    time_ms_since_initial: f64,
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
            g_score: 0.0,
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

    pub fn get_positions_around(&self, pick_style: &NodePickStyle) -> Vec<Node> {
        let mut return_vec = Vec::new();
        let self_rc = Arc::new(Node::new(self.position, self.parent.clone()));

        for i in pick_style.get_offsets(1) {
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

    pub fn get_cost(&self) -> f64 {
        return self.cost;
    }

    pub fn set_g_score(&mut self, new_score: f64) {
        self.g_score = new_score;
    }

    pub fn get_g_cost(&self) -> f64 {
        return self.g_score;
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
