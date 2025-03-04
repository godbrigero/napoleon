use std::sync::Arc;
use std::time::Instant;

use nalgebra::Vector2;
use napoleon_core::hybrid_grid::{GenericDynamicObject, HybridGrid};
use napoleon_core::pathfinding::a_star::AStar;
use napoleon_core::pathfinding::Pathfinding;
use plotters::prelude::*;

fn visualize_map(
    grid: &HybridGrid<Arc<dyn GenericDynamicObject>>,
    start: Vector2<i32>,
    end: Vector2<i32>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Create a drawing area
    let root = BitMapBackend::new("maze_map.png", (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // Find the bounds of our visualization
    let x_range = -1.0..21.0;
    let y_range = -1.0..16.0;

    // Create the chart
    let mut chart = ChartBuilder::on(&root)
        .caption(
            "Maze Map (Before Pathfinding)",
            ("sans-serif", 20).into_font(),
        )
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
    chart.draw_series(std::iter::once(Circle::new(
        (start.x as f64, start.y as f64),
        10,
        GREEN.filled(),
    )))?;
    chart.draw_series(std::iter::once(Circle::new(
        (end.x as f64, end.y as f64),
        10,
        RED.filled(),
    )))?;

    // Add legend
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;
    println!("Initial maze map saved as 'maze_map.png'");
    Ok(())
}

fn visualize_path(
    grid: &HybridGrid<Arc<dyn GenericDynamicObject>>,
    path: &Option<Vec<Vector2<i32>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Create a drawing area
    let root = BitMapBackend::new("path_visualization.png", (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // Find the bounds of our visualization
    let x_range = -1.0..21.0;
    let y_range = -1.0..16.0;

    // Create the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("Path Solution", ("sans-serif", 20).into_font())
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
    chart.draw_series(std::iter::once(Circle::new((18.0, 13.0), 10, RED.filled())))?;

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
    println!("Path visualization saved as 'path_visualization.png'");
    Ok(())
}

fn main() {
    println!("Starting pathfinding demo with timing statistics...\n");
    let setup_start = Instant::now();

    // Create a larger hybrid grid with more space for complex paths
    let mut grid = HybridGrid::new(20, 15, 1.0, vec![], -1, -1); // 20x15 grid

    // Add complex maze-like obstacles
    // Vertical walls
    for y in 0..10 {
        grid.push_static_obstacle(Vector2::new(5, y)); // First vertical wall
        grid.push_static_obstacle(Vector2::new(10, y)); // Second vertical wall
        grid.push_static_obstacle(Vector2::new(15, y)); // Third vertical wall
    }

    // Horizontal walls with gaps
    for x in 5..15 {
        if x != 7 && x != 12 {
            // Leave gaps for passage
            grid.push_static_obstacle(Vector2::new(x, 4)); // First horizontal wall
            grid.push_static_obstacle(Vector2::new(x, 8)); // Second horizontal wall
        }
    }

    // Additional obstacles to create a maze-like pattern
    for x in 5..8 {
        grid.push_static_obstacle(Vector2::new(x, 12)); // Top horizontal wall
    }

    for y in 8..12 {
        grid.push_static_obstacle(Vector2::new(8, y)); // Additional vertical segment
    }

    // Add some diagonal barriers
    for i in 2..3 {
        grid.push_static_obstacle(Vector2::new(12 + i, 10 + i));
    }

    // Add some scattered obstacles
    let scattered_obstacles = [
        (3, 2),
        (4, 2),
        (2, 6),
        (3, 6),
        (17, 3),
        (17, 4),
        (18, 4),
        (7, 13),
        (8, 13),
        (9, 13),
    ];

    for (x, y) in scattered_obstacles.iter() {
        grid.push_static_obstacle(Vector2::new(*x, *y));
    }

    let setup_duration = setup_start.elapsed();
    println!("Grid setup time: {:?}", setup_duration);

    // Define start and end points with a more challenging path
    let start = Vector2::new(0, 0);
    let end = Vector2::new(0, 3);

    // First visualize just the map
    println!("\nGenerating initial maze map...");
    let map_viz_start = Instant::now();
    if let Err(e) = visualize_map(&grid, start, end) {
        eprintln!("Failed to create maze map visualization: {}", e);
    }
    let map_viz_duration = map_viz_start.elapsed();
    println!("Initial map visualization time: {:?}", map_viz_duration);

    println!("\nPress Enter to start pathfinding...");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input).unwrap();

    // Create our A* pathfinder
    let pathfinder = AStar::new(grid.clone());

    // Calculate the path with timing
    let path_calc_start = Instant::now();
    let path = pathfinder.calculate_path(start, end);
    let path_calc_duration = path_calc_start.elapsed();

    println!("\nPathfinding Statistics:");
    println!("----------------------");
    println!("Path calculation time: {:?}", path_calc_duration);

    // Visualize the path with timing
    let viz_start = Instant::now();
    if let Err(e) = visualize_path(&grid, &path) {
        eprintln!("Failed to create path visualization: {}", e);
    }
    let viz_duration = viz_start.elapsed();
    println!("Path visualization time: {:?}", viz_duration);

    // Print the path details
    match &path {
        Some(path) => {
            println!("\nPath Details:");
            println!("-------------");
            println!("Path found: Yes");
            println!("Path length (steps): {}", path.len());

            // Calculate Manhattan distance of path
            let manhattan_distance: i32 = path
                .windows(2)
                .map(|window| (window[1].x - window[0].x).abs() + (window[1].y - window[0].y).abs())
                .sum();
            println!("Total Manhattan distance: {}", manhattan_distance);

            println!("\nPath points:");
            for (i, point) in path.iter().enumerate() {
                println!("Step {}: ({}, {})", i, point.x, point.y);
            }
        }
        None => {
            println!("\nPath Details:");
            println!("-------------");
            println!("Path found: No");
            println!("Reason: No valid path exists between start and end points");
        }
    }

    // Print total execution time
    let total_duration = setup_duration + path_calc_duration + viz_duration;
    println!("\nExecution Summary:");
    println!("-----------------");
    println!("Total execution time: {:?}", total_duration);
    println!(
        "- Setup time: {:?} ({:.1}%)",
        setup_duration,
        (setup_duration.as_nanos() as f64 / total_duration.as_nanos() as f64) * 100.0
    );
    println!(
        "- Pathfinding time: {:?} ({:.1}%)",
        path_calc_duration,
        (path_calc_duration.as_nanos() as f64 / total_duration.as_nanos() as f64) * 100.0
    );
    println!(
        "- Visualization time: {:?} ({:.1}%)",
        viz_duration,
        (viz_duration.as_nanos() as f64 / total_duration.as_nanos() as f64) * 100.0
    );
}
