use super::{NodeRadiusSearch, Pathfinding};
use crate::hybrid_grid::HybridGrid;
use core::f64;
use kiddo::NearestNeighbour;
use nalgebra::Vector2;
use node::{Node, NodePickStyle};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;

pub mod node;

fn average_distance(nodes: &Vec<f32>) -> f32 {
    if nodes.len() == 0 {
        return 0.0;
    }

    nodes.iter().sum::<f32>() / nodes.len() as f32
}

pub struct AStar {
    grid: HybridGrid,
    pick_style: NodePickStyle,
    node_radius_search_config: NodeRadiusSearch,
}

impl Pathfinding for AStar {
    fn new(hybrid_grid: HybridGrid) -> Self {
        Self {
            grid: hybrid_grid,
            pick_style: NodePickStyle::ALL,
            node_radius_search_config: NodeRadiusSearch {
                node_radius_search_radius: 1.0,
                do_absolute_discard: false,
                avg_distance_min_discard_threshold: 1.0,
                avg_distance_cost: 1.0,
            },
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

            let position = current.get_position();
            closed_set.remove(&position);

            for mut neighbor in current.get_positions_around(&self.pick_style) {
                if closed_set.contains(&neighbor.get_position())
                    || self.grid.is_outside_grid(neighbor.get_position())
                    || self.grid.is_obstructed(neighbor.get_position())
                {
                    continue;
                }

                let all_nodes_in_radius = self.grid.get_all_obstructions_in_radius(
                    neighbor.get_position(),
                    self.node_radius_search_config.node_radius_search_radius as i32,
                );
                let avg_distance = average_distance(
                    &all_nodes_in_radius
                        .iter()
                        .map(|node| node.x as f32)
                        .collect::<Vec<f32>>(),
                );

                if neighbor.get_position() != end_node.get_position() {
                    if self.node_radius_search_config.do_absolute_discard
                        && avg_distance != 0.0
                        && avg_distance
                            <= self
                                .node_radius_search_config
                                .avg_distance_min_discard_threshold
                    {
                        continue;
                    }
                }

                let g_score = *g_scores.get(&position).unwrap_or(&f64::MIN);
                let neighbor_position = neighbor.get_position();
                let tentative_g_cost = g_score + current.distance_to(&neighbor);
                let neighbor_g_cost = g_scores
                    .get(&neighbor_position)
                    .copied()
                    .unwrap_or(f64::INFINITY);

                if tentative_g_cost < neighbor_g_cost {
                    let mut extra_cost =
                        if let Some((field, distance)) = self.grid.get_uncertenty_field(
                            Vector2::new(neighbor_position.x as f32, neighbor_position.y as f32),
                        ) {
                            self.grid.uncertenty_field_cost_ramping(
                                distance,
                                field.radius,
                                field.intensity,
                            )
                        } else {
                            0.0
                        };
                    extra_cost += avg_distance * self.node_radius_search_config.avg_distance_cost;

                    neighbor.set_cost(
                        tentative_g_cost + end_node.distance_to(&neighbor) + extra_cost as f64,
                    );

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

    pub fn get_grid(&mut self) -> &mut HybridGrid {
        return &mut self.grid;
    }

    pub fn build(
        hybrid_grid: HybridGrid,
        pick_style: NodePickStyle,
        node_radius_search_config: NodeRadiusSearch,
    ) -> Self {
        Self {
            grid: hybrid_grid,
            pick_style,
            node_radius_search_config,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    fn create_empty_grid() -> HybridGrid {
        HybridGrid::new(10, 10, 1.0, vec![], 5, 5)
    }

    fn create_blocked_grid() -> HybridGrid {
        HybridGrid::new(
            10,
            10,
            1.0,
            vec![
                Vector2::new(2, 2),
                Vector2::new(2, 3),
                Vector2::new(2, 4),
                Vector2::new(3, 2),
                Vector2::new(4, 2),
            ],
            5,
            5,
        )
    }

    #[test]
    fn test_direct_path() {
        let grid = create_empty_grid();
        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(2, 2));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.first(), Some(&Vector2::new(0, 0)));
        assert_eq!(path.last(), Some(&Vector2::new(2, 2)));
    }

    #[test]
    fn test_blocked_path() {
        let grid = create_blocked_grid();
        let astar = AStar::build(
            grid,
            NodePickStyle::SIDES,
            NodeRadiusSearch {
                node_radius_search_radius: 2.0,
                do_absolute_discard: false,
                avg_distance_min_discard_threshold: 0.5,
                avg_distance_cost: 2.0,
            },
        );

        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(5, 5));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.contains(&Vector2::new(2, 2)));
    }

    #[test]
    fn test_impossible_path() {
        let mut grid = create_empty_grid();
        for i in 0..10 {
            grid.push_static_obstacle(Vector2::new(2, i));
        }
        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(5, 5));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_with_uncertainty() {
        let mut grid = create_empty_grid();
        grid.add_uncertenty_field(Vector2::new(2.0, 2.0), 2.0, 10.0);
        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.contains(&Vector2::new(2, 2)));
    }

    #[test]
    fn test_path_with_hybrid_obstacles() {
        let mut grid = create_empty_grid();
        grid.add_hybrid_object(&[2.0, 2.0]);
        let astar = AStar::build(
            grid,
            NodePickStyle::ALL,
            NodeRadiusSearch {
                node_radius_search_radius: 1.0,
                do_absolute_discard: false,
                avg_distance_min_discard_threshold: 0.5,
                avg_distance_cost: 2.0,
            },
        );
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(4, 4));
        assert!(path.is_some());
    }

    #[test]
    fn test_start_equals_end() {
        let grid = create_empty_grid();
        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(0, 0));
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 1);
    }

    #[test]
    fn test_out_of_bounds() {
        let grid = create_empty_grid();
        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(20, 20));
        assert!(path.is_none());
    }

    #[test]
    fn test_different_node_styles() {
        let grid = create_empty_grid();
        let astar = AStar::build(
            grid,
            NodePickStyle::ALL,
            NodeRadiusSearch {
                node_radius_search_radius: 1.0,
                do_absolute_discard: false,
                avg_distance_min_discard_threshold: 0.5,
                avg_distance_cost: 2.0,
            },
        );
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(2, 2));
        assert!(path.is_some());
        assert!(path.unwrap().len() <= 3);
    }

    #[test]
    fn test_node_radius_search_discard() {
        let mut grid = create_empty_grid();
        // Add several hybrid objects close together to create a high-density area
        grid.push_static_obstacle(Vector2::new(2, 2));

        let astar = AStar::build(
            grid,
            NodePickStyle::ALL,
            NodeRadiusSearch {
                node_radius_search_radius: 2.0,
                do_absolute_discard: true,
                avg_distance_min_discard_threshold: 1.0,
                avg_distance_cost: 2.0,
            },
        );

        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        // Path should avoid the dense area around (2,2)
        assert!(!path.contains(&Vector2::new(2, 2)));
    }

    #[test]
    fn test_diagonal_movement() {
        let grid = create_empty_grid();
        let astar = AStar::build(
            grid,
            NodePickStyle::ALL,
            NodeRadiusSearch {
                node_radius_search_radius: 1.0,
                do_absolute_discard: false,
                avg_distance_min_discard_threshold: 0.5,
                avg_distance_cost: 1.0,
            },
        );

        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(3, 3));
        assert!(path.is_some());
        let path = path.unwrap();
        // With diagonal movement, should be a direct diagonal line
        assert_eq!(path.len(), 4); // Should include start, end, and 2 diagonal moves
    }

    #[test]
    fn test_multiple_uncertainty_fields() {
        let mut grid = create_empty_grid();
        grid.add_uncertenty_field(Vector2::new(2.0, 2.0), 2.0, 10.0);
        grid.add_uncertenty_field(Vector2::new(3.0, 3.0), 2.0, 10.0);

        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(5, 5));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.contains(&Vector2::new(2, 2)));
        assert!(!path.contains(&Vector2::new(3, 3)));
    }

    #[test]
    fn test_path_along_border() {
        let mut grid = create_empty_grid();
        // Create obstacles forcing path along the border
        for i in 1..9 {
            grid.push_static_obstacle(Vector2::new(1, i));
        }

        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(0, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        // All x coordinates should be 0
        assert!(path.iter().all(|p| p.x == 0));
    }

    #[test]
    fn test_high_intensity_uncertainty() {
        let mut grid = create_empty_grid();
        // Add a very high intensity uncertainty field
        grid.add_uncertenty_field(Vector2::new(2.0, 2.0), 2.0, 100.0);

        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        // Path should take a longer route to avoid the high-intensity area
        assert!(path.len() > 5); // Should take a longer path than direct diagonal
    }
}
