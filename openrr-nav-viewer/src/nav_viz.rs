use bevy::prelude::*;
use grid_map::*;
use openrr_nav::*;
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Clone, Resource)]
pub struct NavigationViz {
    pub layered_grid_map: Arc<Mutex<LayeredGridMap<u8>>>,
    pub robot_path: Arc<Mutex<NavigationRobotPath>>,
    pub robot_pose: Arc<Mutex<Pose>>,
    pub is_run: Arc<Mutex<bool>>,
    pub start_position: Arc<Mutex<Position>>,
    pub goal_position: Arc<Mutex<Position>>,
    pub planner: Arc<Mutex<DwaPlanner>>,
}

impl Default for NavigationViz {
    fn default() -> Self {
        Self {
            layered_grid_map: Default::default(),
            robot_path: Default::default(),
            robot_pose: Default::default(),
            is_run: Arc::new(Mutex::new(true)),
            start_position: Arc::new(Mutex::new(Position::new(-1.6, -1.8))),
            goal_position: Arc::new(Mutex::new(Position::new(5.0, 1.0))),
            planner: Arc::new(Mutex::new(DwaPlanner::default())),
        }
    }
}
