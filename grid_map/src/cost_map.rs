use crate::{Cell, GridMap, Indices};

pub fn path_distance_map(map: &GridMap<u8>, path: &[Indices]) -> GridMap<u8> {
    let mut path_distance_map = map.copy_without_value();
    for ind in path {
        path_distance_map.set_value_by_indices(ind, 0).unwrap();
    }
    expand_distance_map_internal(&mut path_distance_map, &path, 0, |v| {
        if v == u8::MAX {
            u8::MAX
        } else {
            v + 1
        }
    });
    path_distance_map
}

pub fn goal_distance_map(map: &GridMap<u8>, goal: &Indices) -> GridMap<u8> {
    let mut goal_distance_map = map.copy_without_value();
    goal_distance_map.set_value_by_indices(goal, 0).unwrap();
    expand_distance_map_internal(&mut goal_distance_map, &[goal.to_owned()], 0, |v| {
        if v == u8::MAX {
            u8::MAX
        } else {
            v + 1
        }
    });
    goal_distance_map
}

pub fn obstacle_distance_map(map: &GridMap<u8>) -> GridMap<u8> {
    let mut distance_map = map.copy_without_value();
    let mut obstacle_indices = vec![];
    for y in 0..distance_map.height() {
        for x in 0..distance_map.width() {
            let indices = Indices { x, y };
            if distance_map
                .cell_by_indices(&indices)
                .unwrap()
                .is_obstacle()
            {
                obstacle_indices.push(indices);
            }
        }
    }
    const REDUCE: u8 = 10;
    expand_distance_map_internal(&mut distance_map, &obstacle_indices, u8::MAX, |v| {
        if v < REDUCE {
            0
        } else {
            v - REDUCE
        }
    });
    distance_map
}

pub fn expand_distance_map_internal<F>(
    map: &mut GridMap<u8>,
    previous_indices: &[Indices],
    previous_value: u8,
    increment_func: F,
) -> bool
where
    F: Fn(u8) -> u8,
{
    if !map.cells().iter().any(|c| c.is_uninitialized()) {
        return true;
    }
    let current_value = increment_func(previous_value);
    let mut current_indices = vec![];
    for ind in previous_indices {
        // search +/-
        let opt_index = map.to_index_by_indices(ind);
        if opt_index.is_none() {
            continue;
        }
        for neighbor in ind.neighbors4() {
            if let Some(index) = map.to_index_by_indices(&neighbor) {
                if !map.cells()[index].is_uninitialized() {
                    //if updated[index] {
                    continue;
                }
                let opt_cell = map.cell_by_indices_mut(&neighbor);
                if opt_cell.is_none() {
                    continue;
                }
                let cell = opt_cell.unwrap();
                //if cell.has_value() || cell.is_uninitialized() {
                if cell.is_uninitialized() {
                    *cell = Cell::Value(current_value);
                }
                //updated[index] = true;
                current_indices.push(neighbor);
            }
        }
    }
    //expand_distance_map_internal(map, &current_indices, current_value, increment_func, updated)
    expand_distance_map_internal(map, &current_indices, current_value, increment_func)
}

#[cfg(test)]
mod tests {
    use crate::*;

    fn show_ascii_map(map: &GridMap<u8>, scale: f32) {
        for i in 0..map.height() {
            for j in 0..map.width() {
                let letter = match map.cells()[i * map.width() + j] {
                    Cell::Value(v) => {
                        let v = (v as f32 * scale) as u8;
                        if v <= 9 {
                            format!("{v}")
                        } else {
                            "9".to_owned()
                        }
                    }
                    Cell::Obstacle => "x".to_owned(),
                    Cell::Uninitialized => "u".to_owned(),
                    Cell::Unknown => "?".to_owned(),
                };
                print!("{letter:}");
            }
            println!("");
        }
    }
    #[test]
    fn path_distance_map_test() {
        use rand::distributions::{Distribution, Uniform};
        use rrt;
        let mut map = grid_map::GridMap::<u8>::new(
            Position::new(-1.05, -1.05),
            Position::new(3.05, 1.05),
            0.1,
        );
        for i in 0..20 {
            map.set_obstacle_by_position(&Position::new(0.2 + 0.1 * i as f64, -0.5))
                .unwrap();
            for j in 0..10 {
                map.set_obstacle_by_position(&Position::new(0.1 * i as f64, -0.2 + 0.1 * j as f64))
                    .unwrap();
            }
        }
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let goal = [2.5, 0.5];
        let result = rrt::dual_rrt_connect(
            &[0.5, -0.8],
            &goal,
            |p: &[f64]| {
                !matches!(
                    map.cell_by_position(&Position::new(p[0], p[1])).unwrap(),
                    Cell::Obstacle
                )
            },
            || {
                let mut rng = rand::thread_rng();
                vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
            },
            0.05,
            1000,
        )
        .unwrap();

        let path_indices = result
            .iter()
            .map(|p| {
                map.to_index_by_position(&Position::new(p[0], p[1]))
                    .unwrap()
            })
            .map(|index| map.to_indices_from_index(index).unwrap())
            .collect::<Vec<_>>();
        for p in result {
            map.set_value_by_position(&Position::new(p[0], p[1]), 0)
                .unwrap();
        }

        show_ascii_map(&path_distance_map(&map, &path_indices), 1.0);
        println!("=======================");
        let goal_indices = map
            .position_to_indices(&Position::new(goal[0], goal[1]))
            .unwrap();
        show_ascii_map(&goal_distance_map(&map, &goal_indices), 1.0);
        println!("=======================");
        show_ascii_map(&obstacle_distance_map(&map), 0.03);
    }
}
