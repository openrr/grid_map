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
