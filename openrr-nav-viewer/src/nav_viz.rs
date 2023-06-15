use arci::{BaseVelocity, Localization, MoveBase};
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
            is_run: Arc::new(Mutex::new(false)),
            start_position: Arc::new(Mutex::new(Position::new(-0.8, -0.9))),
            goal_position: Arc::new(Mutex::new(Position::new(2.5, 0.5))),
            weights: Arc::new(Mutex::new(weights)),
        }
    }
}

#[derive(Resource)]
pub struct NavigationVizLite<M, L>
where
    M: MoveBase,
    L: Localization,
{
    pub grid_map: Arc<Mutex<GridMap<u8>>>,
    pub robot_velocity: Arc<Mutex<BaseVelocity>>,
    pub start: Arc<Mutex<Position>>,
    pub goal: Arc<Mutex<Position>>,
    pub is_run: Arc<Mutex<bool>>,
    pub move_base: Arc<Mutex<M>>,
    pub localization: Arc<Mutex<L>>,
}

impl<M, L> Clone for NavigationVizLite<M, L>
where
    M: MoveBase,
    L: Localization,
{
    fn clone(&self) -> Self {
        Self {
            grid_map: Arc::clone(&self.grid_map),
            robot_velocity: Arc::clone(&self.robot_velocity),
            start: Arc::clone(&self.start),
            goal: Arc::clone(&self.goal),
            is_run: Arc::clone(&self.is_run),
            move_base: Arc::clone(&self.move_base),
            localization: Arc::clone(&self.localization),
        }
    }
}

impl<M, L> NavigationVizLite<M, L>
where
    M: MoveBase,
    L: Localization,
{
    pub fn new(move_base: Arc<Mutex<M>>, localization: Arc<Mutex<L>>) -> Self {
        Self {
            grid_map: Arc::new(Mutex::new(GridMap::<u8>::new(
                Position { x: -0.8, y: -0.9 },
                Position { x: 2.5, y: 0.5 },
                0.05,
            ))),
            robot_velocity: Default::default(),
            start: Arc::new(Mutex::new(Position::new(-4.0, -4.0))),
            goal: Arc::new(Mutex::new(Position::new(4.0, 4.0))),
            is_run: Arc::new(Mutex::new(false)),
            move_base,
            localization,
        }
    }
}
