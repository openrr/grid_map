use arci::{Localization, MoveBase};
use grid_map::LayeredGridMap;
use parking_lot::Mutex;
use std::{collections::HashMap, fs, sync::Arc};
use yaml_rust::YamlLoader;

use crate::*;

pub fn local_plan_executor_from_yaml_config<M: MoveBase, L: Localization>(
    path: &str,
    move_base: Arc<Mutex<M>>,
    localization: Arc<Mutex<L>>,
) -> Result<LocalPlanExecutor<M, L>> {
    let yaml_file = match fs::read_to_string(path) {
        Ok(s) => s,
        Err(_) => return Err(Error::FailedToLoadConfigFile("No such file".to_owned())),
    };
    let config_vec = YamlLoader::load_from_str(&yaml_file).unwrap();
    let config = &config_vec[0];

    let local_plane_executor_config = &config["local_plan_executor_config"];
    let local_planner;
    match local_plane_executor_config["planner_type"]
        .as_str()
        .unwrap()
    {
        "dwa" => {
            let dwa_config = &config["dwa_planner_config"];

            let max_vel = dwa_config["max_velocity"].as_vec().unwrap();
            let max_accel = dwa_config["max_acceleration"].as_vec().unwrap();
            let min_vel = dwa_config["min_velocity"].as_vec().unwrap();
            let min_accel = dwa_config["min_acceleration"].as_vec().unwrap();

            let limits = Limits {
                max_velocity: Velocity {
                    x: max_vel[0].as_f64().unwrap(),
                    theta: max_vel[1].as_f64().unwrap(),
                },
                max_accel: Acceleration {
                    x: max_accel[0].as_f64().unwrap(),
                    theta: max_accel[1].as_f64().unwrap(),
                },
                min_velocity: Velocity {
                    x: min_vel[0].as_f64().unwrap(),
                    theta: min_vel[1].as_f64().unwrap(),
                },
                min_accel: Acceleration {
                    x: min_accel[0].as_f64().unwrap(),
                    theta: min_accel[1].as_f64().unwrap(),
                },
            };

            let mut map_name_weight = HashMap::new();
            map_name_weight.insert(
                PATH_DISTANCE_MAP_NAME.to_owned(),
                dwa_config[PATH_DISTANCE_MAP_NAME].as_f64().unwrap(),
            );
            map_name_weight.insert(
                GOAL_DISTANCE_MAP_NAME.to_owned(),
                dwa_config[GOAL_DISTANCE_MAP_NAME].as_f64().unwrap(),
            );
            map_name_weight.insert(
                OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                dwa_config[OBSTACLE_DISTANCE_MAP_NAME].as_f64().unwrap(),
            );

            local_planner = DwaPlanner::new(
                limits,
                map_name_weight,
                dwa_config["controller_dt"].as_f64().unwrap(),
                dwa_config["simulation_duration"].as_f64().unwrap(),
                dwa_config["number_of_velocity_sample"].as_i64().unwrap() as i32,
            );
        }
        _ => {
            todo!()
        }
    }

    let local_plan_executor = LocalPlanExecutor::new(
        move_base,
        localization,
        local_plane_executor_config["frame_id"]
            .as_str()
            .unwrap()
            .to_owned(),
        local_planner,
        LayeredGridMap::default(),
    );

    Ok(local_plan_executor)
}
