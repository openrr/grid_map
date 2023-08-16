use grid_map::*;
use nalgebra as na;
use openrr_nav::*;
use openrr_nav_viewer::*;
use rand::distributions::{Distribution, Uniform};

fn new_sample_map() -> GridMap<u8> {
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-2.05, -2.05), Position::new(6.05, 2.05), 0.05);
    for i in 20..100 {
        for j in 10..14 {
            map.set_obstacle(&Grid::new(i + 20, j)).unwrap();
        }
        for j in 40..60 {
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

fn linear_interpolate_path(path: Vec<Vec<f64>>, extend_length: f64) -> Vec<Vec<f64>> {
    if path.len() < 2 {
        return path;
    }
    let mut interpolated_path = vec![];
    for (p0, p1) in path.iter().zip(path.iter().skip(1)) {
        let diff_x = p1[0] - p0[0];
        let diff_y = p1[1] - p0[1];
        let diff = (diff_x.powi(2) + diff_y.powi(2)).sqrt();
        let direction = diff_y.atan2(diff_x);
        let interpolate_num = (diff / extend_length) as usize;
        if interpolate_num > 0 {
            let unit_diff_x = diff_x / interpolate_num as f64;
            let unit_diff_y = diff_y / interpolate_num as f64;
            for j in 1..interpolate_num {
                interpolated_path.push(vec![
                    p0[0] + unit_diff_x * j as f64,
                    p0[1] + unit_diff_y * j as f64,
                    direction,
                ]);
            }
        } else {
            interpolated_path.push({
                let mut p = p0.to_owned();
                p.push(direction);
                p
            });
        }
    }
    let last_point_angle = interpolated_path.last().unwrap()[2];
    interpolated_path.push({
        let mut end_path = path.last().unwrap().clone();
        end_path.push(last_point_angle);
        end_path
    });
    interpolated_path
}

fn add_target_position_to_path(path: Vec<Vec<f64>>, target_pose: &Pose) -> Vec<Vec<f64>> {
    let mut p = path.clone();
    p.push(vec![
        target_pose.translation.x,
        target_pose.translation.y,
        target_pose.rotation.angle(),
    ]);
    p
}

fn main() {
    let nav = NavigationViz::default();

    let cloned_nav = nav.clone();

    let planner = DwaPlanner::new_from_config(format!(
        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
        env!("CARGO_MANIFEST_DIR")
    ));

    {
        let mut locked_planner = cloned_nav.planner.lock();
        *locked_planner = planner.unwrap();
    }

    std::thread::spawn(move || loop {
        if cloned_nav.is_run.lock().to_owned() {
            let mut map = new_sample_map();
            let x_range = Uniform::new(map.min_point().x, map.max_point().x);
            let y_range = Uniform::new(map.min_point().y, map.max_point().y);
            let start;
            let goal;
            {
                let locked_start = cloned_nav.start_position.lock();
                start = [
                    locked_start.translation.x,
                    locked_start.translation.y,
                    locked_start.rotation.angle(),
                ];
                let locked_goal = cloned_nav.goal_position.lock();
                goal = [
                    locked_goal.translation.x,
                    locked_goal.translation.y,
                    locked_goal.rotation.angle(),
                ];
            }
            let is_free = |p: &[f64]| {
                !matches!(
                    map.cell(&map.to_grid(p[0], p[1]).unwrap()).unwrap(),
                    Cell::Obstacle
                )
            };
            const EXTEND_LENGTH: f64 = 0.05;
            let mut result = rrt::dual_rrt_connect(
                &[start[0], start[1]],
                &[goal[0], goal[1]],
                is_free,
                || {
                    let mut rng = rand::thread_rng();
                    vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
                },
                EXTEND_LENGTH,
                4000,
            )
            .unwrap();
            rrt::smooth_path(&mut result, is_free, EXTEND_LENGTH, 1000);
            let result = linear_interpolate_path(result, EXTEND_LENGTH);
            let result = add_target_position_to_path(
                result,
                &Pose::new(Vector2::new(goal[0], goal[1]), goal[2]),
            );
            {
                let mut locked_robot_path = cloned_nav.robot_path.lock();
                locked_robot_path.set_global_path(robot_path_from_vec_vec(result.clone()));
            }
            let path_grid = result
                .iter()
                .map(|p| map.to_grid(p[0], p[1]).unwrap())
                .collect::<Vec<_>>();

            for p in result.iter() {
                map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
            }

            let path_distance_map = path_distance_map(&map, &path_grid).unwrap();

            let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();
            let goal_distance_map = goal_distance_map(&map, &goal_grid).unwrap();

            let obstacle_distance_map = obstacle_distance_map(&map).unwrap();

            let local_goal_disrance_map =
                local_goal_distance_map(&map, &result, [start[0], start[1]]).unwrap();

            {
                let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                locked_layered_grid_map
                    .add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
                locked_layered_grid_map
                    .add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
                locked_layered_grid_map
                    .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
                locked_layered_grid_map.add_layer(
                    LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                    local_goal_disrance_map,
                );
            }

            {
                let mut locked_angle_space = cloned_nav.angle_space.lock();
                locked_angle_space.add_space(ROTATION_COST_NAME.to_owned(), start[2]);
                locked_angle_space.add_space(GOAL_DIRECTION_COST_NAME.to_owned(), goal[2]);
            }

            let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), start[2]);
            let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

            let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
            let mut plan_map = map.clone();

            for i in 0..300 {
                // let dynamic_map = new_dynamic_sample_map(i);
                let dynamic_map = new_sample_map();
                let path_distance_map =
                    openrr_nav::path_distance_map(&dynamic_map, &path_grid).unwrap();

                let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();
                let goal_distance_map =
                    openrr_nav::goal_distance_map(&dynamic_map, &goal_grid).unwrap();

                let obstacle_distance_map =
                    openrr_nav::obstacle_distance_map(&dynamic_map).unwrap();

                let local_goal_disrance_map = openrr_nav::local_goal_distance_map(
                    &map,
                    &result,
                    [current_pose.translation.x, current_pose.translation.y],
                )
                .unwrap();

                {
                    let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                    locked_layered_grid_map
                        .add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
                    locked_layered_grid_map
                        .add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
                    locked_layered_grid_map
                        .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
                    locked_layered_grid_map.add_layer(
                        LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                        local_goal_disrance_map,
                    );
                }

                {
                    let mut locked_angle_space = cloned_nav.angle_space.lock();
                    locked_angle_space
                        .add_space(ROTATION_COST_NAME.to_owned(), current_pose.rotation.angle());
                }

                let (plan, candidates) = {
                    let locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                    let locked_angle_space = cloned_nav.angle_space.lock();
                    let locked_planner = cloned_nav.planner.lock();
                    (
                        locked_planner.plan_local_path(
                            &current_pose,
                            &current_velocity,
                            &locked_layered_grid_map,
                            &locked_angle_space,
                        ),
                        locked_planner.predicted_plan_candidates(&current_pose, &current_velocity),
                    )
                };
                {
                    let mut locked_robot_path = cloned_nav.robot_path.lock();
                    locked_robot_path.set_local_path(RobotPath(plan.path.clone()));
                    for (i, candidate) in candidates.iter().enumerate() {
                        locked_robot_path.add_user_defined_path(
                            &format!("candidate_{}", i),
                            RobotPath(candidate.path.clone()),
                        );
                    }
                }

                current_velocity = plan.velocity;
                current_pose = plan.path[0];

                {
                    let mut locked_robot_pose = cloned_nav.robot_pose.lock();
                    *locked_robot_pose = current_pose;
                }
                std::thread::sleep(std::time::Duration::from_millis(50));

                if let Some(grid) =
                    plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
                {
                    let _ = plan_map.set_value(&grid, 9);
                } else {
                    println!("OUT OF MAP!");
                    return;
                }
                const GOAL_THRESHOLD_DISTANCE: f64 = 0.1;
                const GOAL_THRESHOLD_ANGLE_DIFFERENCE: f64 = 0.4;
                if (goal_pose.translation.vector - current_pose.translation.vector).norm()
                    < GOAL_THRESHOLD_DISTANCE
                    && (goal_pose.rotation.angle() - current_pose.rotation.angle()).abs()
                        < GOAL_THRESHOLD_ANGLE_DIFFERENCE
                {
                    println!("GOAL! count = {i}");
                    break;
                }
            }
            {
                let mut is_run = cloned_nav.is_run.lock();
                *is_run = false;
            }
        }
    });

    let bevy_cloned_nav = nav.clone();
    let mut app = BevyAppNav::new();
    app.setup(bevy_cloned_nav);
    app.run();
}
