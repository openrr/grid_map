use crate::{Cell, GridMap, LayeredGridMap, Position};
use nalgebra as na;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity {
    pub x: f64,
    pub theta: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Acceleration {
    pub x: f64,
    pub theta: f64,
}

pub type Pose = na::Isometry2<f64>;

#[derive(Debug, Clone, Default)]
pub struct Plan {
    pub velocity: Velocity,
    pub path: Vec<Pose>,
}

#[derive(Debug, Clone, Default)]
pub struct DwaPlanner {
    max_velocity: Velocity,
    max_accel: Acceleration,
    min_velocity: Velocity,
    min_accel: Acceleration,

    map_name_weight: HashMap<String, f64>,
    controller_dt: f64,
    simulation_duration: f64,
    num_vel_sample: i32,
}

fn accumulate_values_by_positions(map: &GridMap<u8>, positions: &[Position]) -> f64 {
    let mut cost: f64 = 0.0;
    for p in positions {
        if let Some(opt) = map.cell_by_position(p) {
            match opt {
                Cell::Value(v) => cost += v as f64,
                Cell::Obstacle => cost = f64::MIN,
                _ => {}
            }
        } else {
            cost = f64::MIN;
        }
    }
    cost
}

impl DwaPlanner {
    pub fn new(
        max_velocity: Velocity,
        max_accel: Acceleration,
        min_velocity: Velocity,
        min_accel: Acceleration,
        map_name_weight: HashMap<String, f64>,
        controller_dt: f64,
        simulation_duration: f64,
        num_vel_sample: i32,
    ) -> Self {
        Self {
            max_velocity,
            max_accel,
            min_velocity,
            min_accel,
            map_name_weight,
            controller_dt,
            simulation_duration,
            num_vel_sample,
        }
    }

    pub(crate) fn sample_velocity(&self, current_velocity: &Velocity) -> Vec<Velocity> {
        let max_x_limit = (current_velocity.x + self.max_accel.x * self.controller_dt)
            .clamp(self.min_velocity.x, self.max_velocity.x);
        let min_x_limit = (current_velocity.x + self.min_accel.x * self.controller_dt)
            .clamp(self.min_velocity.x, self.max_velocity.x);
        let max_theta_limit = (current_velocity.theta + self.max_accel.theta * self.controller_dt)
            .clamp(self.min_velocity.theta, self.max_velocity.theta);
        let min_theta_limit = (current_velocity.theta + self.min_accel.theta * self.controller_dt)
            .clamp(self.min_velocity.theta, self.max_velocity.theta);
        let vel_dx = (max_x_limit - min_x_limit) / self.num_vel_sample as f64;
        let vel_dtheta = (max_theta_limit - min_theta_limit) / self.num_vel_sample as f64;
        let mut velocities = vec![];
        for i in 0..(self.num_vel_sample + 1) {
            for j in 0..(self.num_vel_sample + 1) {
                velocities.push(Velocity {
                    x: min_x_limit + vel_dx * j as f64,
                    theta: min_theta_limit + vel_dtheta * i as f64,
                });
            }
            velocities.push(Velocity {
                x: 0.0,
                theta: min_theta_limit + vel_dtheta * i as f64,
            });
        }
        velocities
    }
    fn forward_simulation(&self, current_pose: &Pose, target_velocity: &Velocity) -> Vec<Pose> {
        let diff = Pose::new(
            na::Vector2::new(target_velocity.x * self.controller_dt, 0.0),
            target_velocity.theta * self.controller_dt,
        );
        let mut poses = vec![];
        for _ in 0..(self.simulation_duration / self.controller_dt) as usize {
            poses.push(current_pose * diff);
        }
        poses
    }
    pub fn plan_local_path(
        &self,
        current_pose: &na::Isometry2<f64>,
        current_velocity: &Velocity,
        maps: &LayeredGridMap<u8>,
    ) -> Plan {
        let plans = self
            .sample_velocity(current_velocity)
            .into_iter()
            .map(|v| Plan {
                velocity: v.to_owned(),
                path: self.forward_simulation(current_pose, &v),
            })
            .collect::<Vec<_>>();
        let mut min_cost = f64::MAX;
        let mut selected_plan = Plan::default();
        for plan in plans {
            for (k, v) in &self.map_name_weight {
                let cost = v * accumulate_values_by_positions(
                    maps.layer(&k).unwrap(),
                    &plan
                        .path
                        .iter()
                        .map(|p| Position::new(p.translation.x, p.translation.y))
                        .collect::<Vec<_>>(),
                );
                if cost < min_cost {
                    min_cost = cost;
                    selected_plan = plan.clone();
                }
            }
        }
        selected_plan
    }
}
