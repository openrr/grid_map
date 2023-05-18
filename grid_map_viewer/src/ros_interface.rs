use arci::{nalgebra::Quaternion, Isometry2};
use grid_map::{Cell, GridMap, Indices, Position, RobotPath};
use rosrust_msg::{
    map_msgs::OccupancyGridUpdate,
    nav_msgs::{self, OccupancyGrid},
};

pub fn new_grid_map_with_ros_navigation_costmap(occupancy_grid: OccupancyGrid) -> GridMap<u8> {
    let resolution = occupancy_grid.info.resolution as f64;

    let width = occupancy_grid.info.width as usize;
    let height = occupancy_grid.info.height as usize;
    let origin = occupancy_grid.info.origin;

    let min_point = Position {
        x: origin.position.x,
        y: origin.position.y,
    };
    let max_point = Position {
        x: width as f64 * resolution + origin.position.x,
        y: height as f64 * resolution + origin.position.y,
    };

    let mut renew_grid_map = GridMap::<u8>::new(min_point, max_point, resolution);

    let _ = occupancy_grid.data.iter().enumerate().map(|(i, &d)| {
        let index = Indices::new(i % renew_grid_map.width(), i / renew_grid_map.width());
        println!("index: {:?}", index);
        if d == -1 {
            renew_grid_map.set_cell_by_indices(&index, Cell::Unknown);
        } else if (0..=100).contains(&d) {
            renew_grid_map.set_cell_by_indices(&index, Cell::Value(d as u8));
        } else {
            renew_grid_map.set_cell_by_indices(&index, Cell::Unknown);
        }
    });

    GridMap::new(min_point, max_point, resolution)
}

pub fn update_grid_map_with_ros_navigation_costmap_update(
    grid_map: &mut GridMap<u8>,
    occupancy_grid_update: OccupancyGridUpdate,
) {
    let _ = occupancy_grid_update
        .data
        .iter()
        .enumerate()
        .map(|(i, &d)| {
            let index = Indices::new(i % grid_map.width(), i / grid_map.width());
            if d == -1 {
                grid_map.set_cell_by_indices(&index, Cell::Unknown);
            } else if (0..=100).contains(&d) {
                grid_map.set_cell_by_indices(&index, Cell::Value(d as u8));
            } else {
                grid_map.set_cell_by_indices(&index, Cell::Unknown);
            }
        });
}

pub fn update_robot_path_with_ros_navigation_path(
    robot_path: &mut RobotPath,
    ros_msg_path: nav_msgs::Path,
) {
    robot_path.clear();
    for pose in ros_msg_path.poses {
        let translation = arci::Vector2::new(pose.pose.position.x, pose.pose.position.y);
        let o = pose.pose.orientation;
        let q = Quaternion::new(o.w, o.x, o.y, o.z);
        let angle = arci::UnitQuaternion::new_normalize(q).angle();

        robot_path.push(Isometry2::new(translation, angle));
    }
}
