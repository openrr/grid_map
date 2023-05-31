use grid_map::*;
use nalgebra as na;
use openrr_nav::*;
use openrr_nav_viewer::*;
use parking_lot::Mutex;
use rand::distributions::{Distribution, Uniform};

use std::{collections::HashMap, sync::Arc};

fn new_sample_map() -> GridMap<u8> {
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-1.05, -1.05), Position::new(3.05, 1.05), 0.05);
    for i in 10..50 {
        map.set_obstacle(&Grid::new(i + 10, 5)).unwrap();
        map.set_obstacle(&Grid::new(i + 10, 6)).unwrap();
        for j in 20..30 {
            map.set_obstacle(&Grid::new(i, j)).unwrap();
        }
    }
    map
}

fn robot_path_from_vec_vec(path: Vec<Vec<f64>>) -> RobotPath {
    let mut robot_path_inner = vec![];
    for p in path {
        let pose = na::Isometry2::new(na::Vector2::new(p[0], p[1]), 0.);

        robot_path_inner.push(pose);
    }
    RobotPath(robot_path_inner)
}

fn main() {
    let layered_grid_map = Arc::new(Mutex::new(LayeredGridMap::default()));
    let navigator = Arc::new(Mutex::new(Navigator::new()));

    let cloned_layered_grid_map = layered_grid_map.clone();
    let cloned_navigator = navigator.clone();

    std::thread::spawn(move || {
        let mut map = new_sample_map();
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start = [-0.8, -0.9];
        let goal = [0.8, 0.6];
        let result = rrt::dual_rrt_connect(
            &start,
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

        {
            let mut locked_navigator = cloned_navigator.lock();
            locked_navigator
                .nav_path
                .set_global_path(robot_path_from_vec_vec(result.clone()));
        }
        let path_grid = result
            .iter()
            .map(|p| map.to_grid(p[0], p[1]).unwrap())
            .collect::<Vec<_>>();

        for p in result {
            map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
        }

        let path_distance_map = path_distance_map(&map, &path_grid).unwrap();

        let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();
        let goal_distance_map = goal_distance_map(&map, &goal_grid).unwrap();

        let obstacle_distance_map = obstacle_distance_map(&map).unwrap();

        {
            let mut locked_layered_grid_map = cloned_layered_grid_map.lock();
            locked_layered_grid_map.add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
            locked_layered_grid_map.add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
            locked_layered_grid_map
                .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
        }
        let mut weights = HashMap::new();
        weights.insert(PATH_DISTANCE_MAP_NAME.to_owned(), 0.8);
        weights.insert(GOAL_DISTANCE_MAP_NAME.to_owned(), 0.9);
        weights.insert(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), 0.3);

        let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), 0.0);
        let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), 0.0);

        let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
        let mut plan_map = map.clone();

        for i in 0..100 {
            let plan;

            {
                let mut locked_navigator = cloned_navigator.lock();
                locked_navigator.set_local_path_from_global_path();
                plan = Plan {
                    velocity: Default::default(),
                    cost: Default::default(),
                    path: locked_navigator.nav_path.local_path().0.clone(),
                };
            }

            current_velocity = plan.velocity;
            current_pose = plan.path[0];

            {
                let mut locked_navigator = cloned_navigator.lock();
                locked_navigator.current_pose = current_pose;
            }
            std::thread::sleep(std::time::Duration::from_millis(400));

            if let Some(grid) =
                plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
            {
                let _ = plan_map.set_value(&grid, 9);
            } else {
                println!("OUT OF MAP!");
                return;
            }
            const GOAL_THRESHOLD: f64 = 0.1;
            if (goal_pose.translation.vector - current_pose.translation.vector).norm()
                < GOAL_THRESHOLD
            {
                println!("GOAL! count = {i}");
                break;
            }
        }
    });

    let bevy_cloned_layered_grid_map = Arc::clone(&layered_grid_map);
    let bevy_cloned_navigator = Arc::clone(&navigator);

    // Setup for Bevy app.
    let res_layered_grid_map = ResLayeredGridMap(bevy_cloned_layered_grid_map);
    let res_navigator = ResNavigator(bevy_cloned_navigator);

    let mut app = BevyAppNav::new();

    app.setup_as_navigator(res_layered_grid_map, res_navigator);

    app.run();
}
