#[cfg(feature = "ros")]
fn main() {
    use bevy::prelude::*;
    use bevy_egui::EguiPlugin;
    use grid_map::{GridMap, Position};
    use grid_map_viewer::{
        bevy_ui_system_for_arc, update_grid_map_with_ros_navigation_costmap,
        update_grid_map_with_ros_navigation_costmap_update, ArcGridMap,
    };
    use parking_lot::Mutex;
    use rosrust_msg::{map_msgs::OccupancyGridUpdate, nav_msgs::OccupancyGrid};
    use std::sync::Arc;

    rosrust::init("name");

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

    let dynamic_grid_map = Arc::clone(&grid_map);
    let _grid_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap",
        1,
        move |msg: OccupancyGrid| {
            let mut updated_grid_map = dynamic_grid_map.lock();
            update_grid_map_with_ros_navigation_costmap(&mut updated_grid_map, msg);
        },
    )
    .unwrap();

    let dynamic_grid_map_updated = Arc::clone(&grid_map);
    let _grid_update_sub = rosrust::subscribe(
        "/move_base/local_costmap/costmap_updates",
        1,
        move |msg: OccupancyGridUpdate| {
            let mut updated_grid_map_updated = dynamic_grid_map_updated.lock();
            update_grid_map_with_ros_navigation_costmap_update(&mut updated_grid_map_updated, msg);
        },
    )
    .unwrap();

    let bevy_grid_map = Arc::clone(&grid_map);
    let arc_grid_map = ArcGridMap(bevy_grid_map);

    App::new()
        .insert_resource(arc_grid_map)
        .add_plugins(default_plugins)
        .add_plugin(EguiPlugin)
        .add_system(bevy_ui_system_for_arc)
        .run();
}

#[cfg(not(feature = "ros"))]
fn main() {
    println!("ROS is not featured.");
}
