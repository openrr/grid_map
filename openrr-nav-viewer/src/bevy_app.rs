use bevy::prelude::*;
use bevy_egui::{
    egui::{self, plot::Plot, Color32},
    EguiContexts, EguiPlugin,
};

use crate::*;

pub const PATH_DISTANCE_MAP_NAME: &str = "path";
pub const GOAL_DISTANCE_MAP_NAME: &str = "goal";
pub const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";

pub struct BevyAppNav {
    pub app: App,
}

impl BevyAppNav {
    pub fn new() -> Self {
        Self { app: App::new() }
    }

    pub fn setup(
        &mut self,
        res_layered_grid_map: ResLayeredGridMap,
        res_robot_path: ResNavRobotPath,
        res_robot_pose: ResPose,
    ) {
        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "OpenRR Nav Viz".to_owned(),
                resolution: (1920., 1080.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });

        let map_type = MapType::default();

        self.app
            .insert_resource(res_layered_grid_map)
            .insert_resource(res_robot_path)
            .insert_resource(res_robot_pose)
            .insert_resource(map_type)
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .add_startup_system(setup_system)
            .add_system(ui_system)
            .add_system(update_system);
    }

    pub fn run(&mut self) {
        self.app.run();
    }
}

pub fn setup_system(mut _contexts: EguiContexts) {}

pub fn update_system(
    mut contexts: EguiContexts,
    res_layered_grid_map: Res<ResLayeredGridMap>,
    res_robot_path: Res<ResNavRobotPath>,
    res_robot_pose: Res<ResPose>,
    map_type: Res<MapType>,
) {
    let ctx = contexts.ctx_mut();

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Map").data_aspect(1.).show(ui, |plot_ui| {
            // Plot map
            let map = res_layered_grid_map.0.lock();
            match map_type.as_ref() {
                MapType::PathDistanceMap => {
                    if let Some(dist_map) = map.layer(PATH_DISTANCE_MAP_NAME) {
                        for p in parse_grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
                MapType::GoalDistanceMap => {
                    if let Some(dist_map) = map.layer(GOAL_DISTANCE_MAP_NAME) {
                        for p in parse_grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
                MapType::ObstacleDistanceMap => {
                    if let Some(dist_map) = map.layer(OBSTACLE_DISTANCE_MAP_NAME) {
                        for p in parse_grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
            }

            // Plot path
            let path = res_robot_path.0.lock();
            plot_ui.line(parse_robot_path_to_line(
                path.global_path(),
                Color32::BLUE,
                10.,
            ));
            plot_ui.line(parse_robot_path_to_line(
                path.local_path(),
                Color32::RED,
                10.,
            ));
            for (_, p) in path.get_user_defined_path_as_iter() {
                plot_ui.line(parse_robot_path_to_line(p, Color32::LIGHT_YELLOW, 3.));
            }

            // Plot robot pose
            let pose = res_robot_pose.0.lock();
            plot_ui.points(parse_robot_pose_to_point(&pose, Color32::DARK_RED, 10.));
        });
    });
}

pub fn ui_system(mut contexts: EguiContexts, mut map_type: ResMut<MapType>) {
    let ctx = contexts.ctx_mut();

    egui::SidePanel::left("left_side_panel")
        .default_width(200.)
        .show(ctx, |ui| {
            ui.radio_value(map_type.as_mut(), MapType::PathDistanceMap, "Path");
            ui.radio_value(map_type.as_mut(), MapType::GoalDistanceMap, "Goal");
            ui.radio_value(map_type.as_mut(), MapType::ObstacleDistanceMap, "Obstacle");
        });
}
