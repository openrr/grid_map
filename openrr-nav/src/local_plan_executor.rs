use std::sync::Arc;

use crate::*;
use arci::{BaseVelocity, Localization, MoveBase};
use grid_map::LayeredGridMap;
use nalgebra::Isometry2;
use parking_lot::Mutex;

pub struct LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    /// local planner
    /// TODO: Support more planners
    move_base: Arc<Mutex<M>>,
    localization: Arc<Mutex<L>>,
    frame_id: String,
    local_planner: DwaPlanner,
    cost_maps: LayeredGridMap<u8>,
}

impl<M, L> LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    pub fn new(
        move_base: Arc<Mutex<M>>,
        localization: Arc<Mutex<L>>,
        frame_id: String,
        local_planner: DwaPlanner,
        cost_maps: LayeredGridMap<u8>,
    ) -> Self {
        Self {
            move_base,
            localization,
            frame_id,
            local_planner,
            cost_maps,
        }
    }

    pub fn set_cost_maps(&mut self, cost_maps: LayeredGridMap<u8>) {
        self.cost_maps = cost_maps;
    }

    pub fn exec_once(&self) -> Result<()> {
        let current_pose = self
            .localization
            .lock()
            .current_pose(&self.frame_id)
            .unwrap();
        let base_velocity = self.move_base.lock().current_velocity().unwrap();

        let current_velocity = Velocity {
            x: base_velocity.x,
            theta: base_velocity.theta,
        };

        let plan =
            self.local_planner
                .plan_local_path(&current_pose, &current_velocity, &self.cost_maps);

        self.move_base
            .lock()
            .send_velocity(&BaseVelocity::new(plan.velocity.x, 0., plan.velocity.theta))
            .unwrap();

        Ok(())
    }

    pub fn current_velocity(&self) -> BaseVelocity {
        self.move_base.lock().current_velocity().unwrap()
    }

    pub fn current_pose(&self) -> Isometry2<f64> {
        self.localization.lock().current_pose("").unwrap()
    }

    pub fn stop(&self) {
        self.move_base
            .lock()
            .send_velocity(&BaseVelocity {
                x: 0.,
                y: 0.,
                theta: 0.,
            })
            .unwrap();
    }
}
