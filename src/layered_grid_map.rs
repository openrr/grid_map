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
    pub fn add_layer(&mut self, name: String, map: GridMap<T>) {
        self.maps.insert(name, map);
    }
    pub fn layer(&self, name: &str) -> Option<&GridMap<T>> {
        self.maps.get(name)
    }
    pub fn layer_mut(&mut self, name: &str) -> Option<&mut GridMap<T>> {
        self.maps.get_mut(name)
    }
}
