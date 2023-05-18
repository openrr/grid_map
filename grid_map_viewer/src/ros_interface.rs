use arci::{nalgebra::Quaternion, Isometry2};
use grid_map::{Cell, GridMap, LayeredGridMap, Position, RobotPath};
use rosrust_msg::{
    map_msgs::OccupancyGridUpdate,
    nav_msgs::{self, OccupancyGrid},
};

pub fn update_grid_map_with_ros_navigation_costmap(
    grid_map: &mut GridMap<u8>,
    occupancy_grid: OccupancyGrid,
) {
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

    grid_map.renew(min_point, max_point, resolution);

    let cells: Vec<Cell<u8>> = occupancy_grid
        .data
        .iter()
        .map(|&d| {
            if d == -1 {
                Cell::Unknown
            } else if (0..=100).contains(&d) {
                Cell::Value(d as u8)
            } else {
                Cell::Unknown
            }
        })
        .collect();

    *grid_map.cells_mut() = cells;
}

pub fn update_grid_map_with_ros_navigation_costmap_update(
    grid_map: &mut GridMap<u8>,
    occupancy_grid_update: OccupancyGridUpdate,
) {
    let cells: Vec<Cell<u8>> = occupancy_grid_update
        .data
        .iter()
        .map(|&d| {
            if d == -1 {
                Cell::Unknown
            } else if (0..=100).contains(&d) {
                Cell::Value(d as u8)
            } else {
                Cell::Unknown
            }
        })
        .collect();

    *grid_map.cells_mut() = cells;
}

pub fn update_layered_grid_map_with_ros_navigation_costmap(
    grid_map: &mut LayeredGridMap<u8>,
    name: &str,
    occupancy_grid: OccupancyGrid,
) {
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

    grid_map.layer(name).unwrap().clone().renew(min_point, max_point, resolution);

    let cells: Vec<Cell<u8>> = occupancy_grid
        .data
        .iter()
        .map(|&d| {
            if d == -1 {
                Cell::Unknown
            } else if (0..=100).contains(&d) {
                Cell::Value(d as u8)
            } else {
                Cell::Unknown
            }
        })
        .collect();

    *grid_map.layer_mut(name).unwrap().cells_mut() = cells;
}

pub fn update_layered_grid_map_with_ros_navigation_costmap_update(
    grid_map: &mut LayeredGridMap<u8>,
    name: &str,
    occupancy_grid_update: OccupancyGridUpdate,
) {
    let cells: Vec<Cell<u8>> = occupancy_grid_update
        .data
        .iter()
        .map(|&d| {
            if d == -1 {
                Cell::Unknown
            } else if (0..=100).contains(&d) {
                Cell::Value(d as u8)
            } else {
                Cell::Unknown
            }
        })
        .collect();

    *grid_map.layer_mut(name).unwrap().cells_mut() = cells;
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
