use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct AngleSpace {
    angles: HashMap<String, f64>,
}

impl AngleSpace {
    pub fn new(angles: HashMap<String, f64>) -> Self {
        Self { angles }
    }
    pub fn add_space(&mut self, name: String, angle: f64) {
        self.angles.insert(name, angle);
    }
    pub fn space(&self, name: &str) -> Option<&f64> {
        self.angles.get(name)
    }
    pub fn space_mut(&mut self, name: &str) -> Option<&mut f64> {
        self.angles.get_mut(name)
    }
}
