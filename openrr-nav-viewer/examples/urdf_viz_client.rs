use arci_urdf_viz::UrdfVizWebClient;
use grid_map::*;
use nalgebra::Vector2;
use openrr_nav::{utils::nearest_path_point, *};
use parking_lot::Mutex;
use rand::{distributions::Uniform, prelude::Distribution};
use std::{collections::HashMap, sync::Arc};

const PATH_DISTANCE_MAP_NAME: &str = "path";
const GOAL_DISTANCE_MAP_NAME: &str = "goal";
const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";
const LOCAL_GOAL_DISTANCE_MAP_NAME: &str = "local_goal";
const ROTATION_COST_NAME: &str = "rotation";
const PATH_DIRECTION_COST_NAME: &str = "path_direction";
const GOAL_DIRECTION_COST_NAME: &str = "goal_direction";

fn new_sample_map() -> GridMap<u8> {
    let mut map = grid_map::GridMap::<u8>::new(
        Position::new(-10.05, -10.05),
        Position::new(30.05, 30.05),
        0.1,
    );

    let height = map.height();
    let width = map.width();
    for h in 0..height {
        map.set_obstacle(&Grid { x: 0, y: h });
        map.set_obstacle(&Grid { x: width, y: h });
    }
    for w in 0..width {
        map.set_obstacle(&Grid { x: w, y: 0 });
        map.set_obstacle(&Grid { x: w, y: height });
    }

    map
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
    let target_pose_vec = vec![
        target_pose.translation.x,
        target_pose.translation.y,
        target_pose.rotation.angle(),
    ];
    match p.last_mut() {
        Some(v) => {
            *v = target_pose_vec;
        }
        None => {
            p.push(target_pose_vec);
        }
    }
    p
}

fn main() {
    let client = UrdfVizWebClient::default();
    client.run_send_velocity_thread();

    let planner = DwaPlanner::new_from_config(format!(
        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();

    let mut local_plan_executor = LocalPlanExecutor::new(
        Arc::new(Mutex::new(client.clone())),
        Arc::new(Mutex::new(client)),
        "".to_owned(),
        planner,
        0.1,
    );

    std::thread::spawn(move || loop {
        let mut map = new_sample_map();
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start = [0.0, 0.0, 0.0];
        let goal = [25.0, 25.0, 2.0];

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
            10000,
        )
        .unwrap();
        rrt::smooth_path(&mut result, is_free, EXTEND_LENGTH, 1000);
        let result = linear_interpolate_path(result, EXTEND_LENGTH);
        let result = add_target_position_to_path(
            result,
            &Pose::new(Vector2::new(goal[0], goal[1]), goal[2]),
        );

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

        let local_goal_distance_map =
            local_goal_distance_map(&map, &result, [start[0], start[1]]).unwrap();

        let mut layered_grid_map = LayeredGridMap::default();
        layered_grid_map.add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
        layered_grid_map.add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
        layered_grid_map.add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
        layered_grid_map.add_layer(
            LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
            local_goal_distance_map,
        );

        let mut angle_table = HashMap::new();
        angle_table.insert(ROTATION_COST_NAME.to_owned(), start[2]);
        angle_table.insert(PATH_DIRECTION_COST_NAME.to_owned(), start[2]);
        angle_table.insert(GOAL_DIRECTION_COST_NAME.to_owned(), goal[2]);

        local_plan_executor.set_cost_maps(layered_grid_map);
        local_plan_executor.set_angle_table(angle_table);

        let mut current_pose;
        let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

        for i in 0..10000 {
            current_pose = local_plan_executor.current_pose().unwrap();

            let local_goal_distance_map = openrr_nav::local_goal_distance_map(
                &map,
                &result,
                [current_pose.translation.x, current_pose.translation.y],
            )
            .unwrap();

            local_plan_executor.set_cost_map_element(
                LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                local_goal_distance_map,
            );

            let nearest_path_point = nearest_path_point(
                &result,
                [current_pose.translation.x, current_pose.translation.y],
            );
            local_plan_executor.set_angle_table_element(
                ROTATION_COST_NAME.to_owned(),
                current_pose.rotation.angle(),
            );
            match nearest_path_point {
                Some(p) => local_plan_executor
                    .set_angle_table_element(PATH_DIRECTION_COST_NAME.to_owned(), p.1[2]),
                None => {}
            }

            local_plan_executor.exec_once().unwrap();

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
    });

    for i in 1..121 {
        std::thread::sleep(std::time::Duration::from_millis(1000));
        println!("{}sec", i);
    }
}
