use std::sync::Arc;

use nalgebra::Vector2;
use napoleon_core::hybrid_grid::{GenericDynamicObject, HybridGrid};
use napoleon_core::pathfinding::a_star::AStar;
use napoleon_core::pathfinding::Pathfinding;
use plotters::prelude::*;

fn visualize_path(
    grid: &HybridGrid<Arc<dyn GenericDynamicObject>>,
    path: &Option<Vec<Vector2<i32>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Create a drawing area
    let root = BitMapBackend::new("path_visualization.png", (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // Find the bounds of our visualization
    let x_range = -1.0..10.0;
    let y_range = -1.0..6.0;

    // Create the chart
    let mut chart = ChartBuilder::on(&root)
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(x_range, y_range)?;

    // Configure the chart
    chart.configure_mesh().draw()?;

    // Draw obstacles (static objects from the grid)
    for obstacle in grid.get_static_obstacles() {
        let x = obstacle[0];
        let y = obstacle[1];
        chart.draw_series(std::iter::once(Rectangle::new(
            [
                (x as f64 - 0.4, y as f64 - 0.4),
                (x as f64 + 0.4, y as f64 + 0.4),
            ],
            BLACK.filled(),
        )))?;
    }

    // Draw start and end points
    chart.draw_series(std::iter::once(Circle::new((0.0, 0.0), 10, GREEN.filled())))?;
    chart.draw_series(std::iter::once(Circle::new((8.0, 4.0), 10, RED.filled())))?;

    // Draw path if it exists
    if let Some(path) = path {
        // Draw path lines
        chart.draw_series(LineSeries::new(
            path.iter().map(|p| (p.x as f64, p.y as f64)),
            BLUE.stroke_width(2),
        ))?;

        // Draw path points
        chart.draw_series(
            path.iter()
                .map(|p| Circle::new((p.x as f64, p.y as f64), 5, BLUE.filled())),
        )?;
    }

    root.present()?;
    println!("Visualization saved as 'path_visualization.png'");
    Ok(())
}

fn main() {
    // Create a hybrid grid with some obstacles
    let mut grid = HybridGrid::new(10, 6, 1.0, vec![], 5, 3); // 10x6 grid

    // Add some static obstacles (walls)
    for y in 0..5 {
        grid.push_static_obstacle(Vector2::new(5, y)); // Vertical wall
    }
    for x in 0..4 {
        grid.push_static_obstacle(Vector2::new(x, 3)); // Horizontal wall
    }

    // Create our A* pathfinder
    let pathfinder = AStar::new(grid.clone());

    // Define start and end points
    let start = Vector2::new(0, 0);
    let end = Vector2::new(8, 4);

    // Calculate the path
    let path = pathfinder.calculate_path(start, end);

    // Visualize the path
    if let Err(e) = visualize_path(&grid, &path) {
        eprintln!("Failed to create visualization: {}", e);
    }

    // Print the path details
    match &path {
        Some(path) => {
            println!("Path found!");
            println!("Path points:");
            for (i, point) in path.iter().enumerate() {
                println!("Step {}: ({}, {})", i, point.x, point.y);
            }
        }
        None => println!("No path found!"),
    }
}
