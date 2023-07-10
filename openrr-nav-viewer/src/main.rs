use std::collections::HashMap;

use grid_map::*;
use nalgebra as na;
use openrr_nav::*;
use openrr_nav_viewer::*;
use rand::distributions::{Distribution, Uniform};

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

fn robot_path_from_vec_vec(path: &[Vec<f64>]) -> RobotPath {
    let mut robot_path_inner = vec![];
    for p in path {
        let pose = na::Isometry2::new(na::Vector2::new(p[0], p[1]), 0.);

        robot_path_inner.push(pose);
    }
    RobotPath(robot_path_inner)
}

fn linear_interpolate_path(path: Vec<Vec<f64>>, extend_length: f64) -> Vec<Vec<f64>> {
    if path.len() < 2 {
        return path;
    }
    let mut interpolated_path = vec![];
    interpolated_path.push(path.first().unwrap().clone());
    for (p0, p1) in path.iter().zip(path.iter().skip(1)) {
        let diff_x = p1[0] - p0[0];
        let diff_y = p1[1] - p0[1];
        let diff = (diff_x.powi(2) + diff_y.powi(2)).sqrt();
        let interpolate_num = (diff / extend_length) as usize;
        if interpolate_num > 0 {
            let unit_diff_x = diff_x / interpolate_num as f64;
            let unit_diff_y = diff_y / interpolate_num as f64;
            for j in 1..interpolate_num {
                interpolated_path.push(vec![
                    p0[0] + unit_diff_x * j as f64,
                    p0[1] + unit_diff_y * j as f64,
                ]);
            }
        } else {
            interpolated_path.push(p0.to_owned());
        }
    }
    interpolated_path.push(path.last().unwrap().clone());
    interpolated_path
}

mod api {
    use super::*;
    pub struct Api(NavigationViz);
    impl Api {
        pub fn new(nav: NavigationViz) -> Self {
            Self(nav)
        }
        pub fn get_start_position(&self) -> [f64; 2] {
            let start = self.0.start_position.lock();
            [start.x, start.y]
        }
        pub fn get_goal_position(&self) -> [f64; 2] {
            let goal = self.0.goal_position.lock();
            [goal.x, goal.y]
        }
        pub fn get_weights(&self) -> HashMap<String, f64> {
            self.0.weights.lock().clone()
        }
        pub fn get_layered_grid_map(&self) -> LayeredGridMap<u8> {
            self.0.layered_grid_map.lock().clone()
        }
        pub fn get_is_run(&self) -> bool {
            *self.0.is_run.lock()
        }
        pub fn set_global_path(&self, path: RobotPath) {
            self.0.robot_path.lock().set_global_path(path);
        }
        pub fn set_local_path_and_candidates(&self, path: RobotPath, candidates: Vec<Plan>) {
            let mut robot_path = self.0.robot_path.lock();
            robot_path.set_local_path(path);
            for (i, candidate) in candidates.iter().enumerate() {
                robot_path.add_user_defined_path(
                    &format!("candidate_{}", i),
                    RobotPath(candidate.path.clone()),
                );
            }
        }
        pub fn set_grid_maps(
            &self,
            path_distance_map: GridMap<u8>,
            goal_distance_map: GridMap<u8>,
            obstacle_distance_map: GridMap<u8>,
        ) {
            let mut layered_grid_map = self.0.layered_grid_map.lock();
            layered_grid_map.add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
            layered_grid_map.add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
            layered_grid_map
                .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
        }
        pub fn set_current_pose(&self, current_pose: Pose) {
            *self.0.robot_pose.lock() = current_pose;
        }
        pub fn set_is_run(&self, is_run: bool) {
            *self.0.is_run.lock() = is_run;
        }
    }
}

fn controller(api: &api::Api) {
    if api.get_is_run() {
        let mut map = new_sample_map();
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start = api.get_start_position();
        let goal = api.get_goal_position();
        let is_free = |p: &[f64]| {
            !matches!(
                map.cell(&map.to_grid(p[0], p[1]).unwrap()).unwrap(),
                Cell::Obstacle
            )
        };
        const EXTEND_LENGTH: f64 = 0.05;
        let mut result = rrt::dual_rrt_connect(
            &start,
            &goal,
            is_free,
            || {
                let mut rng = rand::thread_rng();
                vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
            },
            0.05,
            1000,
        )
        .unwrap();
        rrt::smooth_path(&mut result, is_free, EXTEND_LENGTH, 1000);
        let result = linear_interpolate_path(result, EXTEND_LENGTH);
        api.set_global_path(robot_path_from_vec_vec(&result));
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

        api.set_grid_maps(path_distance_map, goal_distance_map, obstacle_distance_map);
        let weights = api.get_weights();

        let planner = DwaPlanner::new(
            Limits {
                max_velocity: Velocity { x: 0.5, theta: 2.0 },
                max_accel: Acceleration { x: 2.0, theta: 5.0 },
                min_velocity: Velocity {
                    x: 0.0,
                    theta: -2.0,
                },
                min_accel: Acceleration {
                    x: -2.0,
                    theta: -5.0,
                },
            },
            weights,
            0.1,
            1.0,
            5,
        );

        let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), 0.0);
        let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), 0.0);

        let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
        let mut plan_map = map.clone();

        for i in 0..100 {
            let (plan, candidates) = {
                let layered_grid_map = api.get_layered_grid_map();
                (
                    planner.plan_local_path(&current_pose, &current_velocity, &layered_grid_map),
                    planner.predicted_plan_candidates(&current_pose, &current_velocity),
                )
            };
            api.set_local_path_and_candidates(RobotPath(plan.path.clone()), candidates);

            current_velocity = plan.velocity;
            current_pose = plan.path[0];

            api.set_current_pose(current_pose);
            std::thread::sleep(std::time::Duration::from_millis(50));

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
        api.set_is_run(false);
    }
}

fn main() {
    let nav = NavigationViz::default();

    let api = api::Api::new(nav.clone());

    std::thread::spawn(move || loop {
        controller(&api);
    });

    let bevy_cloned_nav = nav.clone();
    let mut app = BevyAppNav::new();
    app.setup(bevy_cloned_nav);
    app.run();
}
