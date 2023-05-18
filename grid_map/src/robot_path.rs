use arci::Isometry2;

use crate::{Indices, Position};

#[derive(Debug, Clone, Default)]
pub struct RobotPathGrid {
    path: Vec<Indices>,
    resolution: f64,
    min_point: Position,
    max_point: Position,
}

impl RobotPathGrid {
    pub fn new(min_point: Position, max_point: Position, resolution: f64) -> Self {
        Self {
            path: Vec::new(),
            resolution,
            min_point,
            max_point,
        }
    }

    pub fn update_by_path_isometry(&mut self, robot_path: &RobotPath) {
        self.path.clear();

        // Parse Isometry path to indices path
        let grid_path: Vec<Indices> = robot_path
            .0
            .iter()
            .filter(|pose| {
                pose.translation.x > self.min_point.x && pose.translation.y > self.min_point.y
            })
            .map(|pose| {
                let idx = ((pose.translation.x - self.min_point.x) / self.resolution) as usize;
                let idy = ((pose.translation.y - self.min_point.y) / self.resolution) as usize;

                Indices::new(idx, idy)
            })
            .collect();

        let mut last_element = Indices::new(usize::MAX, usize::MAX);
        // Remove duplicated elements
        let grid_path: Vec<Indices> = grid_path
            .iter()
            .cloned()
            .filter(|&ids| {
                if ids == last_element {
                    false
                } else {
                    last_element = ids.clone();
                    true
                }
            })
            .collect();

        let mut last_element = grid_path.first().unwrap().clone();
        // Interpolate non-contiguous elements.
        let grid_path: Vec<Indices> = grid_path
            .iter()
            .cloned()
            .skip(1)
            .map(|ids| {
                if ids.neighbors4().contains(&last_element) {
                    ids
                } else {
                    last_element = ids;
                    todo!()
                }
            })
            .collect();

        self.path = grid_path;
    }
}

#[derive(Debug, Clone, Default)]
pub struct RobotPath(Vec<Isometry2<f64>>);

impl RobotPath {
    pub fn new() -> Self {
        Self(Vec::new())
    }

    pub fn get_vec(&self) -> &Vec<Isometry2<f64>> {
        &self.0
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn push(&mut self, pose: Isometry2<f64>) {
        self.0.push(pose);
    }
}

pub struct NavigationRobotPath {
    local_path: RobotPath,
    global_path: RobotPath,
}

impl NavigationRobotPath {
    pub fn new(local_path: RobotPath, global_path: RobotPath) -> Self {
        Self {
            local_path,
            global_path,
        }
    }

    pub fn set_local_path(&mut self, local_path: RobotPath) {
        self.local_path = local_path;
    }

    pub fn set_global_path(&mut self, global_path: RobotPath) {
        self.global_path = global_path;
    }

    pub fn local_path(&self) -> &RobotPath {
        &self.local_path
    }

    pub fn local_path_mut(&mut self) -> &mut RobotPath {
        &mut self.local_path
    }

    pub fn global_path(&self) -> &RobotPath {
        &self.global_path
    }

    pub fn global_path_mut(&mut self) -> &mut RobotPath {
        &mut self.global_path
    }
}
