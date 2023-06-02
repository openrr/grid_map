use bevy::prelude::*;
use bevy_egui::{
    egui::{self, plot::Plot, Color32},
    EguiContexts, EguiPlugin,
};
use grid_map::Position;

use crate::*;

pub const PATH_DISTANCE_MAP_NAME: &str = "path";
pub const GOAL_DISTANCE_MAP_NAME: &str = "goal";
pub const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";
pub const DEFAULT_PATH_DISTANCE_WEIGHT: f64 = 0.8;
pub const DEFAULT_GOAL_DISTANCE_WEIGHT: f64 = 0.9;
pub const DEFAULT_OBSTACLE_DISTANCE_WEIGHT: f64 = 0.3;

#[derive(Debug, Resource)]
pub struct UiCheckboxes {
    pub set_start: bool,
    pub set_goal: bool,
    pub restart: bool,
}

impl Default for UiCheckboxes {
    fn default() -> Self {
        Self {
            set_start: false,
            set_goal: false,
            restart: true,
        }
    }
}

#[derive(Default)]
pub struct BevyAppNav {
    app: App,
}

impl BevyAppNav {
    pub fn new() -> Self {
        Self { app: App::new() }
    }

    pub fn setup(&mut self, nav: NavigationViz) {
        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "OpenRR Nav Viz".to_owned(),
                resolution: (1920., 1080.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });

        let map_type = MapType::default();
        let ui_checkboxes = UiCheckboxes::default();

        self.app
            .insert_resource(nav)
            .insert_resource(map_type)
            .insert_resource(ui_checkboxes)
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .add_system(ui_system)
            .add_system(update_system);
    }

    pub fn run(&mut self) {
        self.app.run();
    }
}

fn update_system(
    mut contexts: EguiContexts,
    res_nav: Res<NavigationViz>,
    map_type: Res<MapType>,
    mut ui_checkboxes: ResMut<UiCheckboxes>,
) {
    let ctx = contexts.ctx_mut();

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Map").data_aspect(1.).show(ui, |plot_ui| {
            // Plot map
            let map = res_nav.layered_grid_map.lock();
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
            let path = res_nav.robot_path.lock();
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
            let pose = res_nav.robot_pose.lock();
            plot_ui.points(parse_robot_pose_to_point(&pose, Color32::DARK_RED, 10.));

            let pointer_coordinate = plot_ui.pointer_coordinate();

            if let Some(p) = pointer_coordinate {
                if ui_checkboxes.set_start
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && !ctx.is_pointer_over_area()
                {
                    ui_checkboxes.set_start = false;
                    let mut start_position = res_nav.start_position.lock();
                    *start_position = Position::new(p.x, p.y);
                }
            }

            if let Some(p) = pointer_coordinate {
                if ui_checkboxes.set_goal
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && !ctx.is_pointer_over_area()
                {
                    ui_checkboxes.set_goal = false;
                    let mut goal_position = res_nav.goal_position.lock();
                    *goal_position = Position::new(p.x, p.y);
                    let mut is_run = res_nav.is_run.lock();
                    *is_run = true;
                }
            }
        });
    });
}

fn ui_system(
    mut contexts: EguiContexts,
    res_nav: Res<NavigationViz>,
    mut map_type: ResMut<MapType>,
    mut ui_checkboxes: ResMut<UiCheckboxes>,
) {
    let ctx = contexts.ctx_mut();

    egui::SidePanel::left("left_side_panel")
        .default_width(200.)
        .show(ctx, |ui| {
            ui.radio_value(map_type.as_mut(), MapType::PathDistanceMap, "Path");
            ui.radio_value(map_type.as_mut(), MapType::GoalDistanceMap, "Goal");
            ui.radio_value(map_type.as_mut(), MapType::ObstacleDistanceMap, "Obstacle");

            ui.horizontal(|h_ui| {
                if h_ui.button("Set Start").clicked() {
                    ui_checkboxes.set_start = !ui_checkboxes.set_start;
                    ui_checkboxes.set_goal = false;
                }
                if h_ui.button("Set Goal").clicked() {
                    ui_checkboxes.set_goal = !ui_checkboxes.set_goal;
                    ui_checkboxes.set_start = false;
                }
            });
            ui.label(if ui_checkboxes.set_start {
                "Set start  "
            } else if ui_checkboxes.set_goal {
                "Set goal   "
            } else {
                "Choose mode"
            });

            let mut path_weight = res_nav
                .weights
                .lock()
                .get(PATH_DISTANCE_MAP_NAME)
                .unwrap()
                .to_owned() as f32;
            ui.horizontal(|h_ui| {
                h_ui.label("path weight");
                h_ui.add(egui::Slider::new(&mut path_weight, 0.0..=1.0));
            });
            let mut goal_weight = res_nav
                .weights
                .lock()
                .get(GOAL_DISTANCE_MAP_NAME)
                .unwrap()
                .to_owned() as f32;
            ui.horizontal(|h_ui| {
                h_ui.label("goal weight");
                h_ui.add(egui::Slider::new(&mut goal_weight, 0.0..=1.0));
            });
            let mut obstacle_weight = res_nav
                .weights
                .lock()
                .get(OBSTACLE_DISTANCE_MAP_NAME)
                .unwrap()
                .to_owned() as f32;
            ui.horizontal(|h_ui| {
                h_ui.label("obstacle weight");
                h_ui.add(egui::Slider::new(&mut obstacle_weight, 0.0..=1.0));
            });

            if ui.button("Reset weights").clicked() {
                path_weight = DEFAULT_PATH_DISTANCE_WEIGHT as f32;
                goal_weight = DEFAULT_GOAL_DISTANCE_WEIGHT as f32;
                obstacle_weight = DEFAULT_OBSTACLE_DISTANCE_WEIGHT as f32;
            }

            let mut weights = res_nav.weights.lock();
            weights.insert(PATH_DISTANCE_MAP_NAME.to_owned(), path_weight as f64);
            weights.insert(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_weight as f64);
            weights.insert(
                OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                obstacle_weight as f64,
            );
        });
}
