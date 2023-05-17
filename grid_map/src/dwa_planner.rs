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

#[derive(Debug, Clone, Copy, Default)]
pub struct DwaPlanner {
    max_velocity: Velocity,
    max_accel: Acceleration,
    min_velocity: Velocity,
    min_accel: Acceleration,

    controller_dt: f64,

    num_vel_sample: i32,
}

impl DwaPlanner {
    pub fn new(
        max_velocity: Velocity,
        max_accel: Acceleration,
        min_velocity: Velocity,
        min_accel: Acceleration,
        controller_dt: f64,
        num_vel_sample: i32,
    ) -> Self {
        Self {
            max_velocity,
            max_accel,
            min_velocity,
            min_accel,
            controller_dt,
            num_vel_sample,
        }
    }

    pub(crate) fn sample_velocity(&self, current_velocity: &Velocity) -> Vec<Velocity> {
        let max_x_limit = current_velocity.x + self.max_accel.x * self.controller_dt;
        let min_x_limit = current_velocity.x + self.min_accel.x * self.controller_dt;
        let max_theta_limit = current_velocity.theta + self.max_accel.theta * self.controller_dt;
        let min_theta_limit = current_velocity.theta + self.min_accel.theta * self.controller_dt;
        let vel_dx = (max_x_limit - min_x_limit) / self.num_vel_sample as f64;
        let vel_dtheta = (max_theta_limit - min_theta_limit) / self.num_vel_sample as f64;
        let mut velocities = vec![];
        // TODO: Add zero velocity
        for i in 0..(self.num_vel_sample + 1) {
            velocities.push(Velocity {
                x: min_x_limit + vel_dx * i as f64,
                theta: min_theta_limit + vel_dtheta * i as f64,
            })
        }
        velocities
    }
}
