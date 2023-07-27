// How to run:
// ```sh
// # start viewer
// cargo run -p openrr-nav-viewer
// # start controller example
// cargo run -p openrr-nav-viewer --example controller
// ```

use anyhow::Result;
use grid_map::*;
use nalgebra as na;
use openrr_nav::*;
use openrr_nav_viewer::pb;
use rand::distributions::{Distribution, Uniform};

const ENDPOINT: &str = "http://[::1]:50101";

pub const PATH_DISTANCE_MAP_NAME: &str = "path";
pub const GOAL_DISTANCE_MAP_NAME: &str = "goal";
pub const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";
pub const DEFAULT_PATH_DISTANCE_WEIGHT: f64 = 0.8;
pub const DEFAULT_GOAL_DISTANCE_WEIGHT: f64 = 0.9;
pub const DEFAULT_OBSTACLE_DISTANCE_WEIGHT: f64 = 0.3;

#[tokio::main]
async fn main() -> Result<()> {
    let mut c = pb::api_client::ApiClient::connect(ENDPOINT).await?;
    loop {
        controller(&mut c).await?
    }
}

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

async fn controller(
    api: &mut openrr_nav_viewer::pb::api_client::ApiClient<tonic::transport::Channel>,
) -> Result<()> {
    if !api.get_is_run(()).await?.into_inner().is_run {
        return Ok(());
    }
    let mut map = new_sample_map();
    let x_range = Uniform::new(map.min_point().x, map.max_point().x);
    let y_range = Uniform::new(map.min_point().y, map.max_point().y);
    let start = api.get_start_position(()).await?.into_inner();
    let start = [start.x, start.y];
    let goal = api.get_goal_position(()).await?.into_inner();
    let goal = [goal.x, goal.y];
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
    api.set_global_path(pb::RobotPath::from(robot_path_from_vec_vec(&result)))
        .await?;
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

    api.set_layered_grid_map(pb::SetLayeredGridMapRequest {
        maps: vec![
            pb::NamedGridMap {
                name: PATH_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&path_distance_map).into()),
            },
            pb::NamedGridMap {
                name: GOAL_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&goal_distance_map).into()),
            },
            pb::NamedGridMap {
                name: OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&obstacle_distance_map).into()),
            },
        ],
    })
    .await?;
    let planner = api.get_planner(()).await?.into_inner();
    let planner = DwaPlanner::new(
        planner.limits.unwrap().into(),
        planner.map_name_weight,
        planner.controller_dt,
        planner.simulation_duration,
        planner.num_vel_sample,
    );

    let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), 0.0);
    let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), 0.0);

    let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
    let mut plan_map = map.clone();

    for i in 0..100 {
        let (plan, candidates) = {
            let layered_grid_map = api
                .get_layered_grid_map(pb::GetLayeredGridMapRequest {
                    names: planner.map_names().cloned().collect(),
                })
                .await?
                .into_inner()
                .maps
                .into_iter()
                .map(|(k, v)| (k, v.into()))
                .collect();
            (
                planner.plan_local_path(
                    &current_pose,
                    &current_velocity,
                    &LayeredGridMap::new(layered_grid_map),
                ),
                planner.predicted_plan_candidates(&current_pose, &current_velocity),
            )
        };
        api.set_local_path_and_candidates(pb::PathAndCandidates {
            path: Some(RobotPath(plan.path.clone()).into()),
            candidates: candidates.into_iter().map(Into::into).collect(),
        })
        .await?;

        current_velocity = plan.velocity;
        current_pose = plan.path[0];

        api.set_current_pose(pb::Isometry2::from(current_pose))
            .await?;
        std::thread::sleep(std::time::Duration::from_millis(50));

        if let Some(grid) = plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
        {
            let _ = plan_map.set_value(&grid, 9);
        } else {
            println!("OUT OF MAP!");
            return Ok(());
        }
        const GOAL_THRESHOLD: f64 = 0.1;
        if (goal_pose.translation.vector - current_pose.translation.vector).norm() < GOAL_THRESHOLD
        {
            println!("GOAL! count = {i}");
            break;
        }
    }
    api.set_is_run(pb::IsRun { is_run: false }).await?;
    Ok(())
}
