use crate::{Indices, GridMap, Cell};

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
    if updated.iter().any(|v| *v == false) {
        return true;
    }
    let current_value = previous_value + 1;
    let mut current_indices = vec![];
    for ind in previous_indices {
        // search +/-
        let opt_index = map.to_index_by_indices(ind);
        if opt_index.is_none() {
            continue;
        }
        if updated[opt_index.unwrap()] {
            continue;
        }
            for neighbor in ind.neighbors4() {
                let mut opt_cell = map.cell_by_indices_mut(&neighbor);
                if opt_cell.is_some() {
                    let mut cell = opt_cell.unwrap();
                    cell = Cell::Value(current_value);
                    let index = map.to_index_by_indices(ind).unwrap();
                    updated[index] = true;
                    current_indices.push(neighbor);
                }
            }
    }
    path_distance_map_internal(map, &current_indices, current_value, updated)
}