use crate::{NavigationRobotPath, Pose, RobotPath};

pub struct Navigator {
    pub nav_path: NavigationRobotPath,
    pub current_pose: Pose,
    local_area: [f64; 2],
}

impl Navigator {
    pub fn new() -> Self {
        Self {
            nav_path: NavigationRobotPath::default(),
            current_pose: Pose::default(),
            local_area: [1.0; 2],
        }
    }

    pub fn set_local_path_from_global_path(&mut self) {
        let local_area_rectangle = [
            [
                self.current_pose.translation.x - self.local_area[0],
                self.current_pose.translation.y - self.local_area[1],
            ],
            [
                self.current_pose.translation.x + self.local_area[0],
                self.current_pose.translation.y + self.local_area[1],
            ],
        ];

        let global_plan_as_local = self
            .nav_path
            .global_path()
            .0
            .clone()
            .into_iter()
            .filter(|&p| {
                p.translation.x > 0.5 * local_area_rectangle[0][0]
                    && p.translation.x < 0.5 * local_area_rectangle[1][0]
                    && p.translation.y > 0.5 * local_area_rectangle[0][1]
                    && p.translation.y < 0.5 * local_area_rectangle[1][1]
            })
            .collect::<Vec<_>>();

        *self.nav_path.local_path_mut() = RobotPath(global_plan_as_local);
    }
}
