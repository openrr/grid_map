use crate::DwaPlanner;
use crate::{goal_distance_map, obstacle_distance_map, path_distance_map};
use arci::MoveBase;
use grid_map::{LayeredGridMap, Result};
use std::collections::HashMap;

pub struct LocalPlanExecutor<T>
where
    T: MoveBase,
{
    /// local planner
    /// TODO: Support more planners
    move_base: T,
    local_planner: DwaPlanner,
    cost_maps: LayeredGridMap<u8>,
    weights: HashMap<String, f64>,
    goal_threshold: f64,
}

impl<T> LocalPlanExecutor<T>
where
    T: MoveBase,
{
    pub fn new(
        move_base: T,
        local_planner: DwaPlanner,
        weights: HashMap<String, f64>,
        goal_threshold: f64,
    ) -> Self {
        Self {
            move_base,
            local_planner,
            weights,
            cost_maps: LayeredGridMap::default(),
            goal_threshold,
        }
    }
    pub fn exec_once() -> Result<()> {
        todo!()
    }
}
