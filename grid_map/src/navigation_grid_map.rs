use crate::{GridMap, LayeredGridMap};

#[derive(Clone, Debug)]
pub struct NavigationGridMap<T>
where
    T: Clone,
{
    local_map: LayeredGridMap<T>,
    global_map: LayeredGridMap<T>,
}

impl<T> NavigationGridMap<T>
where
    T: Clone,
{
    pub fn new(local_map: LayeredGridMap<T>, global_map: LayeredGridMap<T>) -> Self {
        NavigationGridMap {
            local_map,
            global_map,
        }
    }

    pub fn local_map(&self) -> &LayeredGridMap<T> {
        &self.local_map
    }

    pub fn global_map(&self) -> &LayeredGridMap<T> {
        &self.global_map
    }

    pub fn update_local_map(&mut self, name: String, map: GridMap<T>) {
        match self.local_map.layer_mut(&name) {
            Some(layer) => *layer = map,
            None => self.local_map.add_layer(name, map),
        }
    }

    pub fn update_global_map(&mut self, name: String, map: GridMap<T>) {
        match self.global_map.layer_mut(&name) {
            Some(layer) => *layer = map,
            None => self.global_map.add_layer(name, map),
        }
    }
}
