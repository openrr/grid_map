use crate::layer::Layer;

#[derive(Clone, Debug, Default)]
pub struct GridMap<T>
where
    T: Clone,
{
    layers: Vec<Layer<T>>,
}

impl<T> GridMap<T>
where
    T: Clone,
{
    pub fn add_layer(&mut self, layer: Layer<T>) {
        self.layers.push(layer);
    }
    pub fn layer(&self, name: &str) -> Option<&Layer<T>> {
        self.layers.iter().find(|l| l.name() == name)
    }
    pub fn layer_mut(&mut self, name: &str) -> Option<&mut Layer<T>> {
        self.layers.iter_mut().find(|l| l.name() == name)
    }
}
