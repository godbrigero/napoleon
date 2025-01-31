use nalgebra::Vector2;
use napoleon::time_structures::TimedPath;
use plotters::prelude::*;
use std::fs::create_dir_all;

fn main() {
    // Create a TimedPath with max speed 2.0 and acceleration 1.0
    let mut path = TimedPath::new(2.0, 1.0);

    // Add key features (positions & directions)
    path.add_key_feature(Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0));
    path.add_key_feature(Vector2::new(2.0, 1.0), Vector2::new(1.0, 1.0));
    path.add_key_feature(Vector2::new(4.0, 1.5), Vector2::new(1.0, 0.5));
    path.add_key_feature(Vector2::new(6.0, 3.0), Vector2::new(1.0, 0.0));
    path.add_key_feature(Vector2::new(8.0, 5.0), Vector2::new(0.0, 1.0));

    // Compute the splines
    path.compute_spline();

    // Sample points along the interpolated path
    let mut sampled_positions = Vec::new();
    let time_step = 0.1;
    for t in (0..(path.total_time * 100.0) as usize).map(|x| x as f64 * time_step) {
        if let Some(pos) = path.get_position_at(t) {
            sampled_positions.push(pos);
        }
    }

    // Plot the results
    create_dir_all("plots").unwrap(); // Ensure directory exists
    let root = BitMapBackend::new("plots/path_plot.png", (800, 600)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption(
            "Smoothed Path with Spline Interpolation",
            ("sans-serif", 30),
        )
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(-1.0..9.0, -1.0..6.0)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    // Draw the original key feature points
    chart
        .draw_series(
            path.key_feature_path
                .iter()
                .map(|p| Circle::new((p.position[0], p.position[1]), 5, RED.filled())),
        )
        .unwrap();

    // Draw the interpolated path
    chart
        .draw_series(LineSeries::new(
            sampled_positions.iter().map(|p| (p[0], p[1])),
            &BLUE,
        ))
        .unwrap();

    root.present().unwrap();
    println!("Path plot saved to plots/path_plot.png");
}
