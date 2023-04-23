use crate::grid_map::GridMap;

#[derive(Clone, Debug, Default)]
pub struct LayeredGridMap<T>
where
    T: Clone,
{
    layers: Vec<GridMap<T>>,
}

impl<T> LayeredGridMap<T>
where
    T: Clone,
{
    pub fn add_layer(&mut self, layer: GridMap<T>) {
        self.layers.push(layer);
    }
    pub fn layer(&self, name: &str) -> Option<&GridMap<T>> {
        self.layers.iter().find(|l| l.name() == name)
    }
    pub fn layer_mut(&mut self, name: &str) -> Option<&mut GridMap<T>> {
        self.layers.iter_mut().find(|l| l.name() == name)
    }
}
