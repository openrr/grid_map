#[cfg(feature = "ros")]
fn main() {
    use bevy::prelude::*;
    use bevy_egui::EguiPlugin;
    use grid_map::{GridMap, Position, RobotPath};
    use grid_map_viewer::*;
    use parking_lot::Mutex;
    use rosrust_msg::{map_msgs::OccupancyGridUpdate, nav_msgs::OccupancyGrid};
    use std::sync::Arc;

    rosrust::init("name");

    // Setup for GUI app by Bevy
    let default_plugins = DefaultPlugins.set(WindowPlugin {
        window: WindowDescriptor {
            title: "Rust GridMap".to_owned(),
            ..Default::default()
        },
        ..Default::default()
    });

    let min_point = Position { x: -0.5, y: -0.5 };
    let max_point = Position { x: 0.5, y: 0.5 };
    let resolution = 0.05;

    let grid_map = Arc::new(Mutex::new(GridMap::<u8>::new(
        min_point, max_point, resolution,
    )));
    let robot_path = Arc::new(Mutex::new(RobotPath::default()));

    // Local map subscriber
    let dynamic_grid_map = Arc::clone(&grid_map);
    let _grid_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap",
        2,
        move |msg: OccupancyGrid| {
            let mut updated_grid_map = dynamic_grid_map.lock();
            *updated_grid_map = new_grid_map_with_ros_navigation_costmap(msg);
        },
    )
    .unwrap();

    // Local map updated subscriber
    let dynamic_grid_map_updated = Arc::clone(&grid_map);
    let _grid_update_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap_updates",
        2,
        move |msg: OccupancyGridUpdate| {
            let mut updated_grid_map_updated = dynamic_grid_map_updated.lock();
            update_grid_map_with_ros_navigation_costmap_update(&mut updated_grid_map_updated, msg);
        },
    )
    .unwrap();

    // Local path subscriber
    let dynamic_robot_path = Arc::clone(&robot_path);
    let _robot_path_sub =
        rosrust::subscribe("/move_base/DWAPlannerROS/local_plan", 2, move |msg| {
            let mut updated_robot_path = dynamic_robot_path.lock();
            update_robot_path_with_ros_navigation_path(&mut updated_robot_path, msg);
        });

    let bevy_grid_map = Arc::clone(&grid_map);
    let bevy_robot_path = Arc::clone(&robot_path);

    let arc_grid_map = ArcGridMap(bevy_grid_map);
    let arc_robot_path = ArcRobotPath(bevy_robot_path);

    App::new()
        .insert_resource(arc_grid_map)
        .insert_resource(arc_robot_path)
        .add_plugins(default_plugins)
        .add_plugin(EguiPlugin)
        .add_system(bevy_ui_system_for_arc2)
        .run();
}

#[cfg(not(feature = "ros"))]
fn main() {
    println!("ROS is not featured.");
}
