/// Utility for debug
pub fn show_ascii_map(map: &grid_map::GridMap<u8>, scale: f32) {
    use grid_map::Cell;
    for i in 0..map.height() {
        for j in 0..map.width() {
            let letter = match map.cells()[i * map.width() + j] {
                Cell::Value(v) => {
                    let v = (v as f32 * scale) as u8;
                    if v <= 9 {
                        format!("{v}")
                    } else {
                        "9".to_owned()
                    }
                }
                Cell::Obstacle => "x".to_owned(),
                Cell::Uninitialized => "u".to_owned(),
                Cell::Unknown => "?".to_owned(),
            };
            print!("{letter:}");
        }
        println!();
    }
}

pub fn nearest_path_point(
    global_path: Vec<Vec<f64>>,
    current_pose: [f64; 2],
) -> Option<(usize, Vec<f64>)> {
    if global_path.is_empty() {
        return None;
    } else {
        let mut nearest = (0, f64::MAX);
        for (i, pose) in global_path.iter().enumerate() {
            let dist =
                ((pose[0] - current_pose[0]).powi(2) + (pose[1] - current_pose[1]).powi(2)).sqrt();
            if dist < nearest.1 {
                nearest.0 = i;
                nearest.1 = dist;
            }
        }
        Some((nearest.0, global_path[nearest.0].clone()))
    }
}
