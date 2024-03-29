use grid_map::{Cell, Error, Grid, GridMap, Position, Result};

use crate::utils::nearest_path_point;

/// Create path distance map
pub fn path_distance_map(map: &GridMap<u8>, path: &[Grid]) -> Result<GridMap<u8>> {
    let mut path_distance_map = map.copy_without_value();
    for ind in path {
        path_distance_map
            .set_value(ind, 0)
            .ok_or_else(|| Error::OutOfRangeGrid(*ind))?;
    }
    expand_distance_map_internal(&mut path_distance_map, path, 0, |v| {
        if v == u8::MAX {
            u8::MAX
        } else {
            v + 1
        }
    });
    Ok(path_distance_map)
}

/// Create goal distance map
pub fn goal_distance_map(map: &GridMap<u8>, goal: &Grid) -> Result<GridMap<u8>> {
    let mut goal_distance_map = map.copy_without_value();
    goal_distance_map
        .set_value(goal, 0)
        .ok_or_else(|| Error::OutOfRangeGrid(*goal))?;
    expand_distance_map_internal(&mut goal_distance_map, &[goal.to_owned()], 0, |v| {
        if v == u8::MAX {
            u8::MAX
        } else {
            v + 1
        }
    });
    Ok(goal_distance_map)
}

/// Create obstacle distance map
pub fn obstacle_distance_map(map: &GridMap<u8>) -> Result<GridMap<u8>> {
    let mut distance_map = map.copy_without_value();
    let mut obstacle_grid = vec![];
    for y in 0..distance_map.height() {
        for x in 0..distance_map.width() {
            let grid = Grid { x, y };
            if distance_map
                .cell(&grid)
                .ok_or_else(|| Error::OutOfRangeGrid(grid))?
                .is_obstacle()
            {
                obstacle_grid.push(grid);
            }
        }
    }
    const REDUCE: u8 = 10;
    expand_distance_map_internal(&mut distance_map, &obstacle_grid, 50, |v| {
        if v < REDUCE {
            0
        } else {
            v - REDUCE
        }
    });
    Ok(distance_map)
}

/// Create local goal distance map
pub fn local_goal_distance_map(
    map: &GridMap<u8>,
    global_path: &[Vec<f64>],
    current_pose: [f64; 2],
) -> Result<GridMap<u8>> {
    let len = global_path.len();
    let nearest = nearest_path_point(global_path, current_pose).unwrap();

    const LOCAL_GOAL_FORWARD_OFFSET: usize = 20;
    let local_goal = global_path[(nearest.0 + LOCAL_GOAL_FORWARD_OFFSET).min(len - 1)].clone();

    let local_width = (2. * (local_goal[0] - current_pose[0]).abs()).max(1.);
    let local_height = (2. * (local_goal[1] - current_pose[1]).abs()).max(1.);

    let resolution = map.resolution();

    let min_point = Position::new(
        current_pose[0] - local_width * 0.5 - resolution,
        current_pose[1] - local_height * 0.5 - resolution,
    );
    let max_point = Position::new(
        current_pose[0] + local_width * 0.5 + resolution,
        current_pose[1] + local_height * 0.5 + resolution,
    );

    let local_map = GridMap::<u8>::new(min_point, max_point, resolution);
    let grid = local_map.to_grid(local_goal[0], local_goal[1]).unwrap();

    goal_distance_map(&local_map, &grid)
}

pub fn expand_distance_map_internal<F>(
    map: &mut GridMap<u8>,
    previous_grids: &[Grid],
    previous_value: u8,
    increment_func: F,
) -> bool
where
    F: Fn(u8) -> u8,
{
    if !map.cells().iter().any(|c| c.is_uninitialized()) || previous_grids.is_empty() {
        return true;
    }
    let current_value = increment_func(previous_value);
    let mut current_grid = vec![];
    for ind in previous_grids {
        // search +/-
        if map.cell(ind).is_none() {
            continue;
        }
        for neighbor in ind.neighbors4() {
            if let Some(cell) = map.cell_mut(&neighbor) {
                if !cell.is_uninitialized() {
                    continue;
                }
                *cell = Cell::Value(current_value);
                current_grid.push(neighbor);
            }
        }
    }
    expand_distance_map_internal(map, &current_grid, current_value, increment_func)
}

#[cfg(test)]
mod tests {
    use crate::utils::show_ascii_map;
    use crate::*;
    use grid_map::*;
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
            map.set_obstacle(&map.to_grid(0.2 + 0.1 * i as f64, -0.5).unwrap());
            for j in 0..10 {
                map.set_obstacle(&map.to_grid(0.1 * i as f64, -0.2 + 0.1 * j as f64).unwrap());
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
                    map.cell(&map.to_grid(p[0], p[1]).unwrap()).unwrap(),
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

        let path_grid = result
            .iter()
            .map(|p| map.to_grid(p[0], p[1]).unwrap())
            .collect::<Vec<_>>();
        for p in result {
            map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
        }

        show_ascii_map(&path_distance_map(&map, &path_grid).unwrap(), 1.0);
        println!("=======================");
        let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();
        show_ascii_map(&goal_distance_map(&map, &goal_grid).unwrap(), 1.0);
        println!("=======================");
        show_ascii_map(&obstacle_distance_map(&map).unwrap(), 0.1);
    }
}
