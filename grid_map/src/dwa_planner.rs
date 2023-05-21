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

fn velocity_to_pose(velocity: &Velocity, dt: f64) -> Pose {
    Pose::new(
        na::Vector2::new(velocity.x * dt, 0.0),
        velocity.theta * dt,
    )
}

#[derive(Debug, Clone, Default)]
pub struct Plan {
    pub velocity: Velocity,
    pub path: Vec<Pose>,
}

#[derive(Debug, Clone, Default)]
/// DWA Planner
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
        let mut last_pose = current_pose.to_owned();
        let diff = velocity_to_pose(target_velocity, self.controller_dt);
        let mut poses = vec![];
        for _ in 0..(self.simulation_duration / self.controller_dt) as usize {
            let next_pose = last_pose * diff;
            poses.push(next_pose);
            last_pose = next_pose;
        }
        poses
    }
    pub fn plan_local_path(
        &self,
        current_pose: &Pose,
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


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use na::Vector2;

    use crate::*;
    use crate::dwa_planner::*;
    use crate::utils::show_ascii_map;
    #[test]
    fn path_distance_map_test() {
        use rand::distributions::{Distribution, Uniform};
        use rrt;
        let mut map = grid_map::GridMap::<u8>::new(
            Position::new(-1.05, -1.05),
            Position::new(3.05, 1.05),
            0.1,
        );
        for i in 0..20 {
            map.set_obstacle_by_position(&Position::new(0.2 + 0.1 * i as f64, -0.5))
                .unwrap();
            for j in 0..10 {
                map.set_obstacle_by_position(&Position::new(0.1 * i as f64, -0.2 + 0.1 * j as f64))
                    .unwrap();
            }
        }
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start = [0.5, -0.8];
        let goal = [2.5, 0.5];
        let result = rrt::dual_rrt_connect(
            &start,
            &goal,
            |p: &[f64]| {
                !matches!(
                    map.cell_by_position(&Position::new(p[0], p[1])).unwrap(),
                    Cell::Obstacle
                )
            },
            || {
                let mut rng = rand::thread_rng();
                vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
            },
            0.05,
            1000,
        )
        .unwrap();

        let path_indices = result
            .iter()
            .map(|p| {
                map.to_index_by_position(&Position::new(p[0], p[1]))
                    .unwrap()
            })
            .map(|index| map.to_indices_from_index(index).unwrap())
            .collect::<Vec<_>>();
        for p in result {
            map.set_value_by_position(&Position::new(p[0], p[1]), 0)
                .unwrap();
        }
        let path_distance_map = path_distance_map(&map, &path_indices);
        show_ascii_map(&path_distance_map, 1.0);
        println!("=======================");
        let goal_indices = map
            .position_to_indices(&Position::new(goal[0], goal[1]))
            .unwrap();
        let goal_distance_map = goal_distance_map(&map, &goal_indices);
        show_ascii_map(&goal_distance_map, 1.0);
        println!("=======================");
        let obstacle_distance_map = obstacle_distance_map(&map);
        show_ascii_map(&obstacle_distance_map, 0.03);
        let mut maps = HashMap::new();
        maps.insert("path".to_owned(), path_distance_map);
        maps.insert("goal".to_owned(), goal_distance_map);
        maps.insert("obstacle".to_owned(), obstacle_distance_map);
        let layered = LayeredGridMap::new(maps);
        let mut weights = HashMap::new();
        weights.insert("path".to_owned(), 0.1);
        weights.insert("goal".to_owned(), 0.1);
        weights.insert("obstacle".to_owned(), 0.03);
        
        let planner = DwaPlanner::new(Velocity { x: 0.1, theta: 0.3}, Acceleration { x: 0.2, theta: 0.6},
            Velocity { x: 0.0, theta: -0.3}, Acceleration { x: -0.2, theta: -0.6},
            weights, 0.1, 3.0, 5);
        let plan = planner.plan_local_path(&Pose::new(Vector2::new(start[0], start[1]),
         0.0), &Velocity { x: 0.0, theta: 0.0 }, &layered);
         println!("vel = {:?}", plan.velocity);

    }
}