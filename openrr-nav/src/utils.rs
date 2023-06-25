use nalgebra::Isometry2;
use std::time::Duration;

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

#[derive(Debug, Clone)]
pub struct Isometry2TimeStamp {
    pose: Option<Isometry2<f64>>,
    time: std::time::Instant,
}

impl Isometry2TimeStamp {
    pub fn pose(&self) -> Isometry2<f64> {
        self.pose.unwrap()
    }

    pub fn set_pose(&mut self, pose: Isometry2<f64>) {
        self.pose = Some(pose);
        self.time = std::time::Instant::now();
    }

    pub fn elapsed(&self) -> Duration {
        self.time.elapsed()
    }

    pub fn is_initialized(&self) -> bool {
        self.pose.is_some()
    }
}

impl Default for Isometry2TimeStamp {
    fn default() -> Self {
        Self {
            pose: Default::default(),
            time: std::time::Instant::now(),
        }
    }
}
