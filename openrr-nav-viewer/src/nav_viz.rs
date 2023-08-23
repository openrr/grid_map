use bevy::prelude::*;
use grid_map::*;
use openrr_nav::*;
use parking_lot::Mutex;
use std::{collections::HashMap, sync::Arc};

#[derive(Clone, Resource)]
pub struct NavigationViz {
    pub layered_grid_map: Arc<Mutex<LayeredGridMap<u8>>>,
    pub angle_table: Arc<Mutex<HashMap<String, f64>>>,
    pub robot_path: Arc<Mutex<NavigationRobotPath>>,
    pub robot_pose: Arc<Mutex<Pose>>,
    pub is_run: Arc<Mutex<bool>>,
    pub start_position: Arc<Mutex<Pose>>,
    pub goal_position: Arc<Mutex<Pose>>,
    pub planner: Arc<Mutex<DwaPlanner>>,
    pub empty_map: Arc<Mutex<GridMap<u8>>>,
}

impl Default for NavigationViz {
    fn default() -> Self {
        Self {
            layered_grid_map: Default::default(),
            angle_table: Default::default(),
            robot_path: Default::default(),
            robot_pose: Default::default(),
            is_run: Arc::new(Mutex::new(true)),
            start_position: Arc::new(Mutex::new(Pose::new(Vector2::new(-1.6, -1.8), 0.0))),
            goal_position: Arc::new(Mutex::new(Pose::new(Vector2::new(5.0, 1.0), 0.0))),
            planner: Arc::new(Mutex::new(DwaPlanner::default())),
            empty_map: Arc::new(Mutex::new(GridMap::new(
                Position { x: -1., y: -1. },
                Position { x: 1., y: 1. },
                0.05,
            ))),
        }
    }
}
