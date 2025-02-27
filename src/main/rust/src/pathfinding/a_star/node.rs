use nalgebra::Vector3;
use std::rc::Rc;

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

pub struct Node {
    position: Vector3<i32>,
    cost: f64,
    parent: Option<Rc<Node>>,
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
    pub fn new(position: Vector3<i32>, parent: Option<Rc<Node>>) -> Self {
        Self {
            position,
            cost: 0.0,
            parent,
        }
    }

    pub fn get_position(&self) -> Vector3<i32> {
        return self.position;
    }

    pub fn get_parent(&self) -> Option<Rc<Node>> {
        return self.parent.clone();
    }

    pub fn get_positions_around(&self, pick_style: &NodePickStyle, step_size: i32) -> Vec<Node> {
        let mut return_vec = Vec::new();
        let self_rc = Rc::new(Node::new(self.position, self.parent.clone()));

        for i in pick_style.get_offsets(step_size) {
            return_vec.push(Node::new(self.position + i, Some(self_rc.clone())));
        }
        return_vec
    }

    pub fn distance_to(&self, other: &Node) -> f64 {
        let diff = self.position - other.position;
        ((diff.x as f64).powi(2) + (diff.y as f64).powi(2) + (diff.z as f64).powi(2)).sqrt()
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
