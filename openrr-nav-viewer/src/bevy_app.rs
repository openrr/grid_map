use std::sync::Arc;

use bevy::prelude::*;
use bevy_egui::{
    egui::{self, plot::Plot, Color32},
    EguiContexts, EguiPlugin,
};
use grid_map::LayeredGridMap;
use openrr_nav::{NavigationRobotPath, Pose};
use parking_lot::Mutex;

use crate::*;

pub const PATH_DISTANCE_MAP_NAME: &str = "path";
pub const GOAL_DISTANCE_MAP_NAME: &str = "goal";
pub const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";

pub struct SharedStructure {
    pub layered_grid_map: LayeredGridMap<u8>,
    pub robot_path: NavigationRobotPath,
    pub robot_pose: Pose,
}

impl SharedStructure {
    pub fn new() -> Self {
        Self {
            layered_grid_map: Default::default(),
            robot_path: Default::default(),
            robot_pose: Default::default(),
        }
    }
}

#[derive(Resource)]
pub struct ResSharedStructure(pub Arc<Mutex<SharedStructure>>);

pub struct BevyAppNav {}

impl BevyAppNav {
    pub fn run(res: ResSharedStructure) {
        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "OpenRR Nav Viz".to_owned(),
                resolution: (1920., 1080.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });

        let map_type = MapType::default();

        App::new()
            .insert_resource(res)
            .insert_resource(map_type)
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .add_startup_system(setup_system)
            .add_system(update_system)
            .run();
    }
}

pub fn setup_system(mut contexts: EguiContexts) {
    contexts.ctx_mut().set_visuals(egui::Visuals {
        window_rounding: 0.0.into(),
        ..Default::default()
    });
}

pub fn update_system(
    mut contexts: EguiContexts,
    res: Res<ResSharedStructure>,
    mut map_type: ResMut<MapType>,
) {
    let ctx = contexts.ctx_mut();

    egui::SidePanel::left("left_side_panel")
        .default_width(200.)
        .show(ctx, |ui| {
            ui.radio_value(map_type.as_mut(), MapType::PathDistanceMap, "Path");
            ui.radio_value(map_type.as_mut(), MapType::GoalDistanceMap, "Goal");
            ui.radio_value(map_type.as_mut(), MapType::ObstacleDistanceMap, "Obstacle");
        });

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Map").data_aspect(1.).show(ui, |plot_ui| {
            // Plot map
            match map_type.as_ref() {
                MapType::PathDistanceMap => {
                    let map = res.0.lock().layered_grid_map.clone();

                    match map.layer(PATH_DISTANCE_MAP_NAME) {
                        Some(dist_map) => {
                            for p in parse_grid_map_to_polygon(dist_map) {
                                plot_ui.polygon(p);
                            }
                        }
                        None => { /* Nothing */ }
                    }
                }
                MapType::GoalDistanceMap => {
                    let map = res.0.lock().layered_grid_map.clone();

                    match map.layer(GOAL_DISTANCE_MAP_NAME) {
                        Some(dist_map) => {
                            for p in parse_grid_map_to_polygon(dist_map) {
                                plot_ui.polygon(p);
                            }
                        }
                        None => { /* Nothing */ }
                    }
                }
                MapType::ObstacleDistanceMap => {
                    let map = res.0.lock().layered_grid_map.clone();

                    match map.layer(OBSTACLE_DISTANCE_MAP_NAME) {
                        Some(dist_map) => {
                            for p in parse_grid_map_to_polygon(dist_map) {
                                plot_ui.polygon(p);
                            }
                        }
                        None => { /* Nothing */ }
                    }
                }
            }

            // Plot path
            let path = res.0.lock().robot_path.clone();
            plot_ui.line(parse_robot_path_to_line(path.global_path(), Color32::BLUE));
            plot_ui.line(parse_robot_path_to_line(path.local_path(), Color32::YELLOW));

            // Plot robot pose
            let pose = res.0.lock().robot_pose;
            plot_ui.points(parse_robot_pose_to_point(&pose, Color32::DARK_RED));
        });
    });
}
