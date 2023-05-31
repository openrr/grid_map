use crate::{NavigationRobotPath, Pose, RobotPath};

pub struct Navigator {
    pub nav_path: NavigationRobotPath,
    pub current_pose: Pose,
    local_area: [f64; 2],
    counter: usize,
}

impl Navigator {
    pub fn new() -> Self {
        Self {
            nav_path: NavigationRobotPath::default(),
            current_pose: Pose::default(),
            local_area: [0.5; 2],
            counter: 0,
        }
    }

    pub fn set_local_path_from_global_path(&mut self) {
        let rect = self.get_local_area_rectangle_points();

        let global_plan_as_local = self
            .nav_path
            .global_path()
            .0
            .clone()
            .into_iter()
            .skip(self.counter)
            .filter(|&p| {
                p.translation.x > rect[0][0]
                    && p.translation.x < rect[1][0]
                    && p.translation.y > rect[0][1]
                    && p.translation.y < rect[1][1]
            })
            .collect::<Vec<_>>();

        self.counter += 1;

        *self.nav_path.local_path_mut() = RobotPath(global_plan_as_local);
    }

    pub fn get_local_area_rectangle_points(&self) -> [[f64; 2]; 2] {
        [
            [
                self.current_pose.translation.x - self.local_area[0],
                self.current_pose.translation.y - self.local_area[1],
            ],
            [
                self.current_pose.translation.x + self.local_area[0],
                self.current_pose.translation.y + self.local_area[1],
            ],
        ]
    }
}
