use std::sync::Arc;

use crate::{utils::Isometry2TimeStamp, *};
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
    last_pose: Isometry2TimeStamp,
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
            last_pose: Isometry2TimeStamp::default(),
        }
    }

    pub fn set_cost_maps(&mut self, cost_maps: LayeredGridMap<u8>) {
        self.cost_maps = cost_maps;
    }

    pub fn exec_once(&mut self) -> Result<()> {
        let current_pose = self
            .localization
            .lock()
            .current_pose(&self.frame_id)
            .unwrap();

        let base_velocity = self.current_velocity();

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

        self.last_pose.set_pose(current_pose);

        Ok(())
    }

    pub fn current_velocity(&self) -> BaseVelocity {
        let current_pose = self
            .localization
            .lock()
            .current_pose(&self.frame_id)
            .unwrap();

        match self.move_base.lock().current_velocity() {
            Ok(v) => v,
            Err(_) => {
                if self.last_pose.is_initialized() {
                    let dt = self.last_pose.elapsed().as_secs_f64();
                    let dx = current_pose.translation.x - self.last_pose.pose().translation.x;
                    let dy = current_pose.translation.y - self.last_pose.pose().translation.y;
                    let dtheta =
                        current_pose.rotation.angle() - self.last_pose.pose().rotation.angle();

                    BaseVelocity {
                        x: (dx.powi(2) + dy.powi(2)).sqrt() / dt,
                        y: 0.,
                        theta: dtheta / dt,
                    }
                } else {
                    BaseVelocity {
                        x: 0.,
                        y: 0.,
                        theta: 0.,
                    }
                }
            }
        }
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
