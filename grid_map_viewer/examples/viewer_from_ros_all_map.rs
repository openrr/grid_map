#[cfg(feature = "ros")]
fn main() {
    use bevy::prelude::*;
    use bevy_egui::EguiPlugin;
    use grid_map::{
        GridMap, LayeredGridMap, NavigationGridMap, NavigationRobotPath, Position, RobotPath,
    };
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

    let local_cost_map = GridMap::<u8>::new(min_point, max_point, resolution);
    let mut local_maps = LayeredGridMap::default();
    local_maps.add_layer("costmap".to_owned(), local_cost_map);

    let global_cost_map = GridMap::<u8>::new(min_point, max_point, resolution);
    let mut global_maps = LayeredGridMap::default();
    global_maps.add_layer("costmap".to_owned(), global_cost_map);

    let local_robot_path = RobotPath::default();
    let global_robot_path = RobotPath::default();

    let nav_grid_map = Arc::new(Mutex::new(NavigationGridMap::new(local_maps, global_maps)));
    let nav_robot_path = Arc::new(Mutex::new(NavigationRobotPath::new(
        local_robot_path,
        global_robot_path,
    )));

    // Local map subscriber
    let dynamic_local_map = Arc::clone(&nav_grid_map);
    let _local_map_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap",
        1,
        move |msg: OccupancyGrid| {
            let map = dynamic_local_map.lock();
            let mut costmap = map.local_map().layer("costmap").unwrap().clone();
            update_grid_map_with_ros_navigation_costmap(&mut costmap, msg);
            let _ = map.local_map().layer("costmap").insert(&mut costmap);
        },
    )
    .unwrap();

    // Local map updated subscriber
    let dynamic_local_map_updated = Arc::clone(&nav_grid_map);
    let _update_local_map_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap_updates",
        1,
        move |msg: OccupancyGridUpdate| {
            let map = dynamic_local_map_updated.lock();
            let mut costmap = map.local_map().layer("costmap").unwrap().clone();
            update_grid_map_with_ros_navigation_costmap_update(&mut costmap, msg);
            let _ = map.local_map().layer("costmap").insert(&mut costmap);
        },
    )
    .unwrap();

    // Global map subscriber
    let dynamic_global_map = Arc::clone(&nav_grid_map);
    let _global_map_sub = rosrust::subscribe(
        "/move_base/global_costmap/costmap",
        1,
        move |msg: OccupancyGrid| {
            let map = dynamic_global_map.lock();
            let mut costmap = map.global_map().layer("costmap").unwrap().clone();
            update_grid_map_with_ros_navigation_costmap(&mut costmap, msg);
            let _ = map.global_map().layer("costmap").insert(&mut costmap);
        },
    )
    .unwrap();

    // Global map updated subscriber
    let dynamic_global_map_updated = Arc::clone(&nav_grid_map);
    let _update_global_map_sub = rosrust::subscribe(
        "/move_base/global_costmap/costmap_updates",
        1,
        move |msg: OccupancyGridUpdate| {
            let map = dynamic_global_map_updated.lock();
            let mut costmap = map.global_map().layer("costmap").unwrap().clone();
            update_grid_map_with_ros_navigation_costmap_update(&mut costmap, msg);
        },
    )
    .unwrap();

    // Local path subscriber
    let dynamic_local_robot_path = Arc::clone(&nav_robot_path);
    let _local_robot_path_sub =
        rosrust::subscribe("/move_base/DWAPlannerROS/local_plan", 1, move |msg| {
            let mut path = dynamic_local_robot_path.lock();
            update_robot_path_with_ros_navigation_path(&mut path.local_path_mut(), msg);
        });

    // Global path subscriber
    let dynamic_global_robot_path = Arc::clone(&nav_robot_path);
    let _global_robot_path_sub =
        rosrust::subscribe("/move_base/DWAPlannerROS/global_plan", 1, move |msg| {
            let mut path = dynamic_global_robot_path.lock();
            update_robot_path_with_ros_navigation_path(&mut path.global_path_mut(), msg);
        });

    let bevy_grid_map = Arc::clone(&nav_grid_map);
    let bevy_robot_path = Arc::clone(&nav_robot_path);

    let arc_grid_map = ArcNavGridMap(bevy_grid_map);
    let arc_robot_path = ArcNavRobotPath(bevy_robot_path);

    App::new()
        .insert_resource(arc_grid_map)
        .insert_resource(arc_robot_path)
        .add_plugins(default_plugins)
        .add_plugin(EguiPlugin)
        .add_system(bevy_ui_system_for_navigation)
        .run();
}

#[cfg(not(feature = "ros"))]
fn main() {
    println!("ROS is not featured.");
}
