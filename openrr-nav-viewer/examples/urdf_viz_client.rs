use arci::Localization;
use arci_urdf_viz::UrdfVizWebClient;
use grid_map::*;
use openrr_nav::*;
use openrr_nav_viewer::*;
use parking_lot::Mutex;
use rand::distributions::{Distribution, Uniform};
use std::{collections::HashMap, sync::Arc};

fn new_square_map() -> GridMap<u8> {
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-5.0, -5.0), Position::new(5.0, 5.0), 0.05);

    for i in 0..200 {
        map.set_obstacle(&Grid { x: 0, y: i });
        map.set_obstacle(&Grid { x: 199, y: i });
        map.set_obstacle(&Grid { x: i, y: 0 });
        map.set_obstacle(&Grid { x: i, y: 199 });
    }

    map
}

fn main() {
    let move_base_client = Arc::new(Mutex::new({
        let client = UrdfVizWebClient::default();
        client.run_send_velocity_thread();
        client
    }));
    let localization_client = Arc::new(Mutex::new(UrdfVizWebClient::default()));

    let mut nav = NavigationVizLite::new(move_base_client, localization_client);
    nav.grid_map = Arc::new(Mutex::new(new_square_map()));

    let cloned_nav = nav.clone();

    std::thread::spawn(move || loop {
        if cloned_nav.is_run.lock().clone() {
            let (x_range, y_range) = {
                let map = cloned_nav.grid_map.lock();
                (
                    Uniform::new(map.min_point().x, map.max_point().x),
                    Uniform::new(map.min_point().y, map.max_point().y),
                )
            };

            let goal = {
                let locked_goal = cloned_nav.goal.lock();
                [locked_goal.x, locked_goal.y]
            };

            let start = {
                let pose = cloned_nav.localization.lock().current_pose("").unwrap();
                [pose.translation.x, pose.translation.y]
            };

            let result = rrt::dual_rrt_connect(
                &start,
                &goal,
                |p: &[f64]| {
                    let map = cloned_nav.grid_map.lock();
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
                .map(|p| {
                    let map = cloned_nav.grid_map.lock();
                    map.to_grid(p[0], p[1]).unwrap()
                })
                .collect::<Vec<_>>();

            {
                let mut grid_map = cloned_nav.grid_map.lock();
                *grid_map = new_square_map();
            }

            for p in result {
                let mut locked_map = cloned_nav.grid_map.lock();
                let grid = locked_map.to_grid(p[0], p[1]).unwrap();
                locked_map.set_value(&grid, 0).unwrap();
            }

            let (path_distance_map, goal_distance_map, obstacle_distance_map) = {
                let map = cloned_nav.grid_map.lock();

                let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();

                (
                    path_distance_map(&map, &path_grid).unwrap(),
                    goal_distance_map(&map, &goal_grid).unwrap(),
                    obstacle_distance_map(&map).unwrap(),
                )
            };

            let mut layered_grid_map = LayeredGridMap::default();
            layered_grid_map.add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
            layered_grid_map.add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
            layered_grid_map
                .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);

            let mut weights = HashMap::new();
            weights.insert(
                PATH_DISTANCE_MAP_NAME.to_owned(),
                DEFAULT_PATH_DISTANCE_WEIGHT,
            );
            weights.insert(
                GOAL_DISTANCE_MAP_NAME.to_owned(),
                DEFAULT_GOAL_DISTANCE_WEIGHT,
            );
            weights.insert(
                OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                DEFAULT_OBSTACLE_DISTANCE_WEIGHT,
            );

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

            let mut current_pose;
            let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), 0.0);

            let move_base_client = cloned_nav.move_base.clone();
            let localization_client = cloned_nav.localization.clone();

            let mut executor = LocalPlanExecutor::new(
                move_base_client,
                localization_client,
                "".to_owned(),
                planner,
                layered_grid_map,
            );

            for i in 0..1000 {
                executor.exec_once().unwrap();

                current_pose = executor.current_pose();

                std::thread::sleep(std::time::Duration::from_millis(50));
                const GOAL_THRESHOLD: f64 = 0.1;
                if (goal_pose.translation.vector - current_pose.translation.vector).norm()
                    < GOAL_THRESHOLD
                {
                    println!("GOAL! count = {i}");
                    break;
                }
            }
            executor.stop();
            {
                let mut is_run = cloned_nav.is_run.lock();
                *is_run = false;
            }
        }
    });

    let bevy_cloned_nav = nav.clone();
    let mut app = BevyAppLite::new();
    app.setup(bevy_cloned_nav);
    app.run();
}
