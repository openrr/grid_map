use crate::GridMap;
use crate::Indices;

pub fn path_distance_map(map: &GridMap<u8>, path: &[Indices]) -> GridMap<u8> {
    let mut path_distance_map = map.copy_without_value();
    let mut updated = vec![false; map.len()];
    for ind in path {
        path_distance_map.set_value_by_indices(ind, 0).unwrap();
        updated[path_distance_map.to_index_by_indices(ind).unwrap()] = true;
    }
    path_distance_map_internal(&mut path_distance_map, &path, 0, &mut updated);
    path_distance_map
}

pub fn path_distance_map_internal(map: &mut GridMap<u8>, 
    previous_indices: &[Indices],
    previous_value: u8,
    updated: &mut [bool]) -> bool {
    // all is true, finish.
    if updated.iter().any(|v| v == false) {
        return true;
    }
    for ind in previous_indices {
        let index = map.to_index_by_indices(ind).unwrap();
        // search +/-
        if !updated[index] {
            map.cell_by_indices_mut(ind)
            update[index] = true;
        }
}