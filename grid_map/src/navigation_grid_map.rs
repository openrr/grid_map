use crate::LayeredGridMap;

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
}
