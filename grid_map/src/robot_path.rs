use arci::Isometry2;

#[derive(Clone, Debug, Default)]
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
