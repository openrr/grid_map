use arci_urdf_viz::UrdfVizWebClient;
use grid_map::*;
use nalgebra as na;
use nalgebra::Vector2;
use openrr_nav::*;
use openrr_nav_viewer::{BevyAppNav, NavigationViz};
use parking_lot::Mutex;
use std::sync::Arc;

fn new_sample_map() -> GridMap<u8> {
    let mut map = grid_map::GridMap::<u8>::new(
        Position::new(-1.05, -1.05),
        Position::new(10.05, 10.05),
        0.1,
    );

    for i in 0..40 {
        for j in 0..20 {
            map.set_obstacle(&Grid {
                x: 40 + i,
                y: 30 + j,
            });
        }
    }
    for i in 0..20 {
        for j in 0..30 {
            map.set_obstacle(&Grid {
                x: 60 + i,
                y: 65 + j,
            });
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
    let client = UrdfVizWebClient::default();
    client.run_send_velocity_thread();

    let nav = NavigationViz::default();

    let start = [0.0, 0.0, 0.0];
    let goal = [9.0, 8.0, std::f64::consts::FRAC_PI_3];
    {
        let mut locked_start = nav.start_position.lock();
        *locked_start = Pose::new(Vector2::new(start[0], start[1]), start[2]);
        let mut locked_goal = nav.goal_position.lock();
        *locked_goal = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);
    }

    let cloned_nav = nav.clone();

    let planner = DwaPlanner::new_from_config(format!(
        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();

    {
        let mut locked_planner = cloned_nav.planner.lock();
        *locked_planner = planner.clone();
    }

    let mut local_plan_executor = LocalPlanExecutor::new(
        Arc::new(Mutex::new(client.clone())),
        Arc::new(Mutex::new(client)),
        "".to_owned(),
        planner,
        0.1,
    );

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

            let mut cost_maps = CostMaps::new(&map, &result, &start, &goal);
            let mut angle_table = AngleTable::new(start[2], goal[2]);

            for p in result.iter() {
                map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
            }

            local_plan_executor.set_cost_maps(cost_maps.layered_grid_map());
            {
                let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                *locked_layered_grid_map = cost_maps.layered_grid_map();
            }

            local_plan_executor.set_angle_table(angle_table.angle_table());
            {
                let mut locked_angle_table = cloned_nav.angle_table.lock();
                *locked_angle_table = angle_table.angle_table();
            }

            let mut current_pose;
            let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

            const STEP: usize = 2000;
            for i in 0..STEP {
                current_pose = local_plan_executor.current_pose().unwrap();

                cost_maps.update(
                    &None,
                    &result,
                    &[current_pose.translation.x, current_pose.translation.y],
                    &[],
                );

                angle_table.update(Some(current_pose), &result);

                local_plan_executor.set_cost_maps(cost_maps.layered_grid_map());
                {
                    let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock();
                    *locked_layered_grid_map = cost_maps.layered_grid_map();
                }

                local_plan_executor.set_angle_table(angle_table.angle_table());
                {
                    let mut locked_angle_table = cloned_nav.angle_table.lock();
                    *locked_angle_table = angle_table.angle_table();
                }

                local_plan_executor.exec_once().unwrap();

                {
                    let mut locked_robot_pose = cloned_nav.robot_pose.lock();
                    *locked_robot_pose = current_pose;
                }

                println!(
                    "[ {:4} / {} ] X: {:.3}, Y: {:.3}, THETA: {:.3}",
                    i + 1,
                    STEP,
                    current_pose.translation.x,
                    current_pose.translation.y,
                    current_pose.rotation.angle()
                );
                std::thread::sleep(std::time::Duration::from_millis(5));

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
            local_plan_executor.stop().unwrap();
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
