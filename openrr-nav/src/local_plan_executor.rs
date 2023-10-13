use crate::utils::PoseTimeStamped;
use crate::{goal_distance_map, obstacle_distance_map, path_distance_map, Velocity};
use crate::{DwaPlanner, Result};
use arci::{BaseVelocity, Localization, MoveBase};
use grid_map::LayeredGridMap;
use std::collections::HashMap;

pub struct LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    /// local planner
    /// TODO: Support more planners
    move_base: M,
    localization: L,
    frame_id: String,
    local_planner: DwaPlanner,
    cost_maps: LayeredGridMap<u8>,
    angles_named: HashMap<String, f64>,
    weights: HashMap<String, f64>,
    goal_threshold: f64,
    last_pose: PoseTimeStamped,
}

impl<M, L> LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    pub fn new(
        move_base: M,
        localization: L,
        frame_id: String,
        local_planner: DwaPlanner,
        weights: HashMap<String, f64>,
        goal_threshold: f64,
    ) -> Self {
        Self {
            move_base,
            localization,
            frame_id,
            local_planner,
            cost_maps: LayeredGridMap::default(),
            angles_named: HashMap::default(),
            weights,
            goal_threshold,
            last_pose: PoseTimeStamped::default(),
        }
    }

    pub fn exec_once(&mut self) -> Result<()> {
        let current_pose = self.localization.current_pose(&self.frame_id).unwrap();

        let current_velocity = {
            let v = self.move_base.current_velocity().unwrap_or(BaseVelocity {
                x: 0.,
                y: 0.,
                theta: 0.,
            });
            Velocity {
                x: v.x,
                theta: v.theta,
            }
        };

        let plan = self.local_planner.plan_local_path(
            &current_pose,
            &current_velocity,
            &self.cost_maps,
            &self.angles_named,
        );

        self.move_base
            .send_velocity(&BaseVelocity {
                x: plan.velocity.x,
                y: 0.,
                theta: plan.velocity.theta,
            })
            .unwrap();

        self.last_pose.set_pose(current_pose);

        Ok(())
    }

    pub fn cost_maps(&self) -> &LayeredGridMap<u8> {
        &self.cost_maps
    }

    pub fn cost_maps_mut(&mut self) -> &mut LayeredGridMap<u8> {
        &mut self.cost_maps
    }

    pub fn angles_named(&self) -> &HashMap<String, f64> {
        &self.angles_named
    }

    pub fn angles_named_mut(&mut self) -> &mut HashMap<String, f64> {
        &mut self.angles_named
    }
}
