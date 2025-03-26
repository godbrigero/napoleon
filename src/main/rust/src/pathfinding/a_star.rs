use super::{NodeRadiusSearch, Pathfinding};
use crate::hybrid_grid::HybridGrid;
use core::f64;
use kiddo::NearestNeighbour;
use nalgebra::{distance, Vector2};
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

fn get_distance_squared(node: &Vector2<i32>, other: &Vector2<i32>) -> f32 {
    return (node.x as f32 - other.x as f32).powi(2) + (node.y as f32 - other.y as f32).powi(2);
}

fn get_distance(node: &Vector2<i32>, other: &Vector2<i32>) -> f32 {
    get_distance_squared(node, other).sqrt()
}

fn get_avg_distances_nearest_nodes(grid: &HybridGrid, radius: f32, center: Vector2<i32>) -> f32 {
    let static_nodes_in_radius = grid.get_all_obstructions_in_radius(center, radius as i32);
    let hybrid_nodes_in_radius = grid.get_nearest_hybrid(center, radius);

    if static_nodes_in_radius.len() == 0 && hybrid_nodes_in_radius.len() == 0 {
        return f32::MAX;
    }

    let mut all_distances = static_nodes_in_radius
        .iter()
        .map(|node| get_distance(&center, &node) * grid.get_square_size_meters())
        .collect::<Vec<f32>>();

    all_distances.extend(
        hybrid_nodes_in_radius
            .iter()
            .map(|node| node.distance.sqrt() * grid.get_square_size_meters()),
    );

    return average_distance(&all_distances);
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
        g_scores.insert(start, 0.0);
        open_set.push(Node::new(start, None));

        while let Some(current) = open_set.pop() {
            let position = current.get_position();

            if position == end_node.get_position() {
                println!("Found path {:?}", closed_set.len());
                return Some(self.reconstruct_path(current));
            }

            closed_set.insert(position);

            for mut neighbor in current.get_positions_around(&self.pick_style) {
                let neighbor_position = neighbor.get_position();

                if self.grid.is_outside_grid(neighbor_position)
                    || self.grid.is_obstructed(neighbor_position)
                    || closed_set.contains(&neighbor_position)
                {
                    continue;
                }

                let tentative_g_cost =
                    current.distance_to(&neighbor) + g_scores.get(&position).unwrap();

                let best_g_score = g_scores.get(&neighbor_position).unwrap_or(&f64::MAX);

                if tentative_g_cost >= *best_g_score {
                    continue;
                }

                g_scores.insert(neighbor_position, tentative_g_cost);

                let mut extra_cost = 0.0;
                if neighbor_position != end_node.get_position() {
                    let scaled_radius = self.node_radius_search_config.node_radius_search_radius;

                    let avg_distance = if self.node_radius_search_config.do_absolute_discard
                        || self.node_radius_search_config.avg_distance_cost != 0.0
                    {
                        get_avg_distances_nearest_nodes(
                            &self.grid,
                            scaled_radius,
                            neighbor_position,
                        )
                    } else {
                        f32::MAX
                    };

                    let threshold = self
                        .node_radius_search_config
                        .avg_distance_min_discard_threshold;

                    /*
                    if self.node_radius_search_config.do_absolute_discard {
                        println!(
                            "Position: {:?}, Avg dist: {:.2}, Threshold: {:.2}",
                            neighbor_position, avg_distance, threshold
                        );
                    } */

                    if self.node_radius_search_config.do_absolute_discard
                        && avg_distance < threshold
                        && avg_distance != f32::MAX
                    {
                        continue;
                    }

                    if avg_distance != f32::MAX
                        && self.node_radius_search_config.avg_distance_cost != 0.0
                    {
                        let proximity_cost = if avg_distance < 0.001 {
                            self.node_radius_search_config.avg_distance_cost * 1000.0
                        } else {
                            self.node_radius_search_config.avg_distance_cost
                                * (scaled_radius / avg_distance)
                        };

                        extra_cost += proximity_cost;
                    }

                    if let Some((field, distance)) = self.grid.get_nearest_uncertainty_field(
                        Vector2::new(neighbor_position.x as f32, neighbor_position.y as f32),
                    ) {
                        extra_cost += self.grid.uncertainty_field_cost_ramping(
                            distance,
                            field.radius,
                            field.intensity,
                        );
                    }
                }

                let f_cost = tentative_g_cost + end_node.distance_to(&neighbor) + extra_cost as f64;
                neighbor.set_cost(f_cost);
                open_set.push(neighbor);
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
        grid.add_uncertainty_field(Vector2::new(2.0, 2.0), 2.0, 10.0);
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
        assert_eq!(path.len(), 4);
    }

    #[test]
    fn test_multiple_uncertainty_fields() {
        let mut grid = create_empty_grid();
        grid.add_uncertainty_field(Vector2::new(2.0, 2.0), 2.0, 10.0);
        grid.add_uncertainty_field(Vector2::new(3.0, 3.0), 2.0, 10.0);

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
        for i in 1..9 {
            grid.push_static_obstacle(Vector2::new(1, i));
        }

        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(0, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(path.iter().all(|p| p.x == 0));
    }

    #[test]
    fn test_high_intensity_uncertainty() {
        let mut grid = create_empty_grid();
        grid.add_uncertainty_field(Vector2::new(2.0, 2.0), 2.0, 100.0);

        let astar = AStar::new(grid);
        let path = astar.calculate_path(Vector2::new(0, 0), Vector2::new(4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(path.len() > 5);
    }
}
