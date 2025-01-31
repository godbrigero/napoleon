use nalgebra::Vector2;
use splines::{Interpolation, Key, Spline};

#[derive(Debug)]
pub struct TimedVector2 {
    pub position: Vector2<f64>,
    pub direction: Vector2<f64>,
    pub time_to_reach: f64,
}

#[derive(Debug)]
pub struct TimedPath {
    pub key_feature_path: Vec<TimedVector2>,
    pub total_time: f64,
    spline_x: Option<Spline<f64, f64>>,
    spline_y: Option<Spline<f64, f64>>,
    max_speed: f64,
    accel: f64,
}

impl TimedPath {
    /// Creates a new empty TimedPath with max speed and acceleration
    pub fn new(max_speed: f64, accel: f64) -> Self {
        Self {
            key_feature_path: vec![],
            total_time: 0.0,
            spline_x: None,
            spline_y: None,
            max_speed,
            accel,
        }
    }

    /// Adds a new key feature point to the path
    pub fn add_key_feature(&mut self, position: Vector2<f64>, direction: Vector2<f64>) {
        let last_time = self
            .key_feature_path
            .last()
            .map(|p| p.time_to_reach)
            .unwrap_or(0.0);

        if let Some(last_pos) = self.key_feature_path.last().map(|p| p.position) {
            let distance = (position - last_pos).norm();
            let time_delta = self.compute_time_to_reach(distance);
            let new_time = last_time + time_delta;

            self.key_feature_path.push(TimedVector2 {
                position,
                direction,
                time_to_reach: new_time,
            });
        } else {
            // First point, assume time 0
            self.key_feature_path.push(TimedVector2 {
                position,
                direction,
                time_to_reach: 0.0,
            });
        }
    }

    /// Computes the time required to reach a point using acceleration curve
    fn compute_time_to_reach(&self, distance: f64) -> f64 {
        let t_accel = self.max_speed / self.accel; // Time to reach max speed
        let d_accel = 0.5 * self.accel * t_accel.powi(2); // Distance covered during acceleration

        if distance < 2.0 * d_accel {
            // If total distance is too short, adjust using a parabola equation
            return 2.0 * (distance / self.accel).sqrt();
        } else {
            // Full trapezoidal profile: accel + cruise + decel
            let d_cruise = distance - 2.0 * d_accel;
            let t_cruise = d_cruise / self.max_speed;
            return 2.0 * t_accel + t_cruise;
        }
    }

    /// Computes the spline interpolation through all key feature points
    pub fn compute_spline(&mut self) {
        if self.key_feature_path.len() < 2 {
            return;
        }

        let keys_x: Vec<Key<f64, f64>> = self
            .key_feature_path
            .iter()
            .map(|tv| Key::new(tv.time_to_reach, tv.position[0], Interpolation::CatmullRom))
            .collect();

        let keys_y: Vec<Key<f64, f64>> = self
            .key_feature_path
            .iter()
            .map(|tv| Key::new(tv.time_to_reach, tv.position[1], Interpolation::CatmullRom))
            .collect();

        self.spline_x = Some(Spline::from_vec(keys_x));
        self.spline_y = Some(Spline::from_vec(keys_y));
        self.total_time = self
            .key_feature_path
            .last()
            .map(|p| p.time_to_reach)
            .unwrap_or(0.0);
    }

    /// Returns the interpolated position at the given time
    pub fn get_position_at(&self, time: f64) -> Option<Vector2<f64>> {
        if time < 0.0 || time > self.total_time {
            return None;
        }
        let x = self
            .spline_x
            .as_ref()
            .and_then(|spline| spline.sample(time))?;
        let y = self
            .spline_y
            .as_ref()
            .and_then(|spline| spline.sample(time))?;
        Some(Vector2::new(x, y))
    }

    /// Returns the approximate time at a given position by finding the closest point on the path
    pub fn get_time_at_position(&self, pos: Vector2<f64>) -> Option<f64> {
        if self.spline_x.is_none() || self.spline_y.is_none() {
            return None;
        }

        // Sample points along the path to find the closest one
        let samples = 100;
        let dt = self.total_time / samples as f64;

        let mut min_dist = f64::MAX;
        let mut best_time = 0.0;

        for i in 0..=samples {
            let t = i as f64 * dt;
            if let Some(path_pos) = self.get_position_at(t) {
                let dist = (path_pos - pos).magnitude();
                if dist < min_dist {
                    min_dist = dist;
                    best_time = t;
                }
            }
        }

        Some(best_time)
    }
}
