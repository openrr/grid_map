use bevy::prelude::*;
use grid_map::*;
use openrr_nav::*;
use parking_lot::Mutex;
use std::{collections::HashMap, sync::Arc};

use crate::*;

#[derive(Clone, Resource)]
pub struct NavigationViz {
    pub layered_grid_map: Arc<Mutex<LayeredGridMap<u8>>>,
    pub robot_path: Arc<Mutex<NavigationRobotPath>>,
    pub robot_pose: Arc<Mutex<Pose>>,
    pub is_run: Arc<Mutex<bool>>,
    pub start_position: Arc<Mutex<Position>>,
    pub goal_position: Arc<Mutex<Position>>,
    pub weights: Arc<Mutex<HashMap<String, f64>>>,
}

impl Default for NavigationViz {
    fn default() -> Self {
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

        Self {
            layered_grid_map: Default::default(),
            robot_path: Default::default(),
            robot_pose: Default::default(),
            is_run: Arc::new(Mutex::new(true)),
            start_position: Arc::new(Mutex::new(Position::new(-0.8, -0.9))),
            goal_position: Arc::new(Mutex::new(Position::new(2.5, 0.5))),
            weights: Arc::new(Mutex::new(weights)),
        }
    }
}
