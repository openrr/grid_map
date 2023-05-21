use crate::grid_map::GridMap;
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct LayeredGridMap<T>
where
    T: Clone,
{
    maps: HashMap<String, GridMap<T>>,
}

impl<T> LayeredGridMap<T>
where
    T: Clone,
{
    /// Initialize with all maps
    pub fn new(maps: HashMap<String, GridMap<T>>) -> Self {
        Self { maps }
    }
    /// Add a map as a layer
    pub fn add_layer(&mut self, name: String, map: GridMap<T>) {
        self.maps.insert(name, map);
    }
    /// Accessor for a map with name
    pub fn layer(&self, name: &str) -> Option<&GridMap<T>> {
        self.maps.get(name)
    }
    /// Mutator for a map with name
    pub fn layer_mut(&mut self, name: &str) -> Option<&mut GridMap<T>> {
        self.maps.get_mut(name)
    }
}
