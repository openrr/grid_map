use grid_map::*;
use nalgebra as na;
use openrr_nav::{utils::nearest_path_point, *};
use openrr_nav_viewer::*;

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

            let mut global_plan = GlobalPlan::new(map.clone(), start, goal);

            let result = global_plan.global_plan();
            {
                let mut locked_robot_path = cloned_nav.robot_path.lock();
                locked_robot_path.set_global_path(robot_path_from_vec_vec(result.clone()));
            }

            for p in result.iter() {
                map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
            }

            let mut cost_maps = CostMaps::new(&map, &result, &start, &goal);
            {
                let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                *locked_layered_grid_map = cost_maps.layered_grid_map();
            }

            let mut angle_table = AngleTable::new(start[2], goal[2]);
            {
                let mut locked_angle_table = cloned_nav.angle_table.lock();
                *locked_angle_table = angle_table.angle_table();
            }

            let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), start[2]);
            let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

            let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
            let mut plan_map = map.clone();

            for i in 0..300 {
                cost_maps.update(
                    &None,
                    &result,
                    &[current_pose.translation.x, current_pose.translation.y],
                    &[],
                );
                {
                    let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                    *locked_layered_grid_map = cost_maps.layered_grid_map();
                }

                angle_table.update(Some(current_pose), &result);
                {
                    let nearest_path_point = nearest_path_point(
                        &result,
                        [current_pose.translation.x, current_pose.translation.y],
                    );
                    let len = result.len();
                    let mut locked_angle_table = cloned_nav.angle_table.lock();
                    locked_angle_table
                        .insert(ROTATION_COST_NAME.to_owned(), current_pose.rotation.angle());
                    match nearest_path_point {
                        Some((idx, _)) => {
                            locked_angle_table.insert(
                                PATH_DIRECTION_COST_NAME.to_owned(),
                                result[(idx + 20).min(len - 1)][2],
                            );
                        }
                        None => {}
                    }
                }

                let (plan, candidates) = {
                    let locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                    let locked_angle_table = cloned_nav.angle_table.lock();
                    let locked_planner = cloned_nav.planner.lock();
                    (
                        locked_planner.plan_local_path(
                            &current_pose,
                            &current_velocity,
                            &locked_layered_grid_map,
                            &locked_angle_table,
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
