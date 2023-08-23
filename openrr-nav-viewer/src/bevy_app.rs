use bevy::prelude::*;
use bevy_egui::{
    egui::{
        self, emath,
        epaint::RectShape,
        plot::{Line, Plot, PlotPoints, Polygon},
        Color32, Pos2, Rounding, Sense, Stroke,
    },
    EguiContexts, EguiPlugin,
};
use nalgebra::Vector2;
use openrr_nav::Pose;

use crate::*;

pub const PATH_DISTANCE_MAP_NAME: &str = "path";
pub const GOAL_DISTANCE_MAP_NAME: &str = "goal";
pub const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";
pub const LOCAL_GOAL_DISTANCE_MAP_NAME: &str = "local_goal";
pub const ROTATION_COST_NAME: &str = "rotation";
pub const PATH_DIRECTION_COST_NAME: &str = "path_direction";
pub const GOAL_DIRECTION_COST_NAME: &str = "goal_direction";

pub const DEFAULT_PATH_DISTANCE_WEIGHT: f64 = 0.8;
pub const DEFAULT_GOAL_DISTANCE_WEIGHT: f64 = 0.1;
pub const DEFAULT_OBSTACLE_DISTANCE_WEIGHT: f64 = 0.3;
pub const DEFAULT_LOCAL_GOAL_DISTANCE_MAP_WEIHT: f64 = 0.8;
pub const DEFAULT_ROTATION_COST_WEIGHT: f64 = 0.1;
pub const DEFAULT_PATH_DIRECTION_COST_WEIGHT: f64 = 0.1;
pub const DEFAULT_GOAL_DIRECTION_COST_WEIGHT: f64 = 0.01;

#[derive(Debug, Resource)]
pub struct UiCheckboxes {
    pub set_start: bool,
    pub set_goal: bool,
    pub restart: bool,
    pub counter: usize,
    pub is_painting: bool,
}

impl Default for UiCheckboxes {
    fn default() -> Self {
        Self {
            set_start: false,
            set_goal: false,
            restart: true,
            counter: 0,
            is_painting: false,
        }
    }
}

#[derive(Debug, Default, Resource)]
pub struct DisplayedArrows(Option<[[f64; 2]; 2]>);

impl DisplayedArrows {
    pub fn set_start(&mut self, point: [f64; 2]) {
        match self.0 {
            Some(p) => {
                self.0 = Some([point, p[1]]);
            }
            None => {
                self.0 = Some([point, point]);
            }
        }
    }

    pub fn set_end(&mut self, point: [f64; 2]) {
        match self.0 {
            Some(p) => {
                self.0 = Some([p[0], point]);
            }
            None => {
                self.0 = Some([point, point]);
            }
        }
    }
}

#[derive(Resource, Default)]
struct PaintedGlobalPath {
    pub lines: Vec<Vec<Pos2>>,
}

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
        let displayed_arrows = DisplayedArrows::default();
        let painted_global_path = PaintedGlobalPath::default();

        self.app
            .insert_resource(nav)
            .insert_resource(map_type)
            .insert_resource(ui_checkboxes)
            .insert_resource(displayed_arrows)
            .insert_resource(painted_global_path)
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .add_system(ui_system)
            .add_system(update_system)
            .add_system(bottom_monitor_system)
            .add_system(painting_ui_system);
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
    mut displayed_arrows: ResMut<DisplayedArrows>,
) {
    let ctx = contexts.ctx_mut();

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Map").data_aspect(1.).show(ui, |plot_ui| {
            // Plot map
            let map = res_nav.layered_grid_map.lock();
            match map_type.as_ref() {
                MapType::PathDistanceMap => {
                    if let Some(dist_map) = map.layer(PATH_DISTANCE_MAP_NAME) {
                        for p in grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
                MapType::GoalDistanceMap => {
                    if let Some(dist_map) = map.layer(GOAL_DISTANCE_MAP_NAME) {
                        for p in grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
                MapType::ObstacleDistanceMap => {
                    if let Some(dist_map) = map.layer(OBSTACLE_DISTANCE_MAP_NAME) {
                        for p in grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
                MapType::LocalGoalDisranceMap => {
                    if let Some(dist_map) = map.layer(LOCAL_GOAL_DISTANCE_MAP_NAME) {
                        for p in grid_map_to_polygon(dist_map) {
                            plot_ui.polygon(p);
                        }
                    }
                }
            }

            // Plot path
            let path = res_nav.robot_path.lock();
            plot_ui.line(robot_path_to_line(path.global_path(), Color32::BLUE, 10.));
            plot_ui.line(robot_path_to_line(path.local_path(), Color32::RED, 10.));
            for (_, p) in path.get_user_defined_path_as_iter() {
                plot_ui.line(robot_path_to_line(p, Color32::LIGHT_YELLOW, 3.));
            }

            // Plot robot pose
            let pose = res_nav.robot_pose.lock();
            plot_ui.polygon(robot_pose_to_polygon(&pose, Color32::DARK_RED, 1.));

            let pointer_coordinate = plot_ui.pointer_coordinate();

            if let Some(p) = pointer_coordinate {
                if ui_checkboxes.set_start
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && ui_checkboxes.counter == 1
                {
                    ui_checkboxes.set_start = false;
                    let mut start_position = res_nav.start_position.lock();
                    let angle = (p.y - start_position.translation.y)
                        .atan2(p.x - start_position.translation.x);
                    *start_position = Pose::new(
                        Vector2::new(start_position.translation.x, start_position.translation.y),
                        angle,
                    );
                    println!("start: {:?}", start_position);
                    ui_checkboxes.counter = 0;
                    displayed_arrows.0 = None;
                }
                if ui_checkboxes.set_start
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && !ctx.is_pointer_over_area()
                    && ui_checkboxes.counter == 0
                {
                    let mut start_position = res_nav.start_position.lock();
                    *start_position = Pose::new(Vector2::new(p.x, p.y), 0.0);
                    ui_checkboxes.counter = 1;
                    displayed_arrows.set_start([p.x, p.y]);
                }
                if ui_checkboxes.set_start && ui_checkboxes.counter == 1 {
                    displayed_arrows.set_end([p.x, p.y]);
                }
            }

            if let Some(p) = pointer_coordinate {
                if ui_checkboxes.set_goal
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && ui_checkboxes.counter == 1
                {
                    ui_checkboxes.set_goal = false;
                    let mut goal_position = res_nav.goal_position.lock();
                    let angle = (p.y - goal_position.translation.y)
                        .atan2(p.x - goal_position.translation.x);
                    *goal_position = Pose::new(
                        Vector2::new(goal_position.translation.x, goal_position.translation.y),
                        angle,
                    );
                    println!("goal: {:?}", goal_position);
                    let mut is_run = res_nav.is_run.lock();
                    *is_run = true;
                    ui_checkboxes.counter = 0;
                    displayed_arrows.0 = None;
                }
                if ui_checkboxes.set_goal
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && !ctx.is_pointer_over_area()
                    && ui_checkboxes.counter == 0
                {
                    let mut goal_position = res_nav.goal_position.lock();
                    *goal_position = Pose::new(Vector2::new(p.x, p.y), 0.0);
                    ui_checkboxes.counter = 1;
                    displayed_arrows.set_start([p.x, p.y]);
                }
                if ui_checkboxes.set_goal && ui_checkboxes.counter == 1 {
                    displayed_arrows.set_end([p.x, p.y]);
                }
            }

            match &displayed_arrows.0 {
                Some(p) => {
                    plot_ui.line(Line::new(PlotPoints::new(vec![p[0], p[1]])));
                }
                None => {}
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
        .min_width(200.)
        .show(ctx, |ui| {
            ui.radio_value(map_type.as_mut(), MapType::PathDistanceMap, "Path");
            ui.radio_value(map_type.as_mut(), MapType::GoalDistanceMap, "Goal");
            ui.radio_value(map_type.as_mut(), MapType::ObstacleDistanceMap, "Obstacle");
            ui.radio_value(
                map_type.as_mut(),
                MapType::LocalGoalDisranceMap,
                "Local Goal",
            );
            ui.label("");
            ui.separator();
            ui.label("");

            ui.horizontal(|h_ui| {
                h_ui.columns(2, |c_ui| {
                    if c_ui[0]
                        .add_sized([Default::default(), 30.], egui::Button::new("Set Start"))
                        .clicked()
                    {
                        ui_checkboxes.set_start = !ui_checkboxes.set_start;
                        ui_checkboxes.set_goal = false;
                        ui_checkboxes.counter = 0;
                    }
                    if c_ui[1]
                        .add_sized([Default::default(), 30.], egui::Button::new("Set Goal"))
                        .clicked()
                    {
                        ui_checkboxes.set_goal = !ui_checkboxes.set_goal;
                        ui_checkboxes.set_start = false;
                        ui_checkboxes.counter = 0;
                    }
                });
            });
            ui.colored_label(
                egui::Color32::RED,
                if ui_checkboxes.set_start {
                    "Set start  "
                } else if ui_checkboxes.set_goal {
                    "Set goal   "
                } else {
                    "Choose mode"
                },
            );
            ui.label("");

            {
                let mut planner = res_nav.planner.lock();
                let weight = planner.map_name_weight_mut();
                let mut path_weight = weight.get(PATH_DISTANCE_MAP_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("path weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(&mut path_weight, 0.0..=1.0));
                });
                let mut goal_weight = weight.get(GOAL_DISTANCE_MAP_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("goal weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(&mut goal_weight, 0.0..=1.0));
                });
                let mut obstacle_weight =
                    weight.get(OBSTACLE_DISTANCE_MAP_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("obstacle weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(&mut obstacle_weight, 0.0..=1.0));
                });
                let mut local_goal_weight =
                    weight.get(LOCAL_GOAL_DISTANCE_MAP_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("local goal weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(&mut local_goal_weight, 0.0..=1.0));
                });
                let mut rotation_cost_weight =
                    weight.get(ROTATION_COST_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("rotation weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(&mut rotation_cost_weight, 0.0..=1.0));
                });
                let mut path_direction_cost_weight =
                    weight.get(PATH_DIRECTION_COST_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("path direction weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(
                        &mut path_direction_cost_weight,
                        0.0..=1.0,
                    ));
                });
                let mut goal_direction_cost_weight =
                    weight.get(GOAL_DIRECTION_COST_NAME).unwrap().to_owned() as f32;
                ui.horizontal(|h_ui| {
                    h_ui.add_sized([100.0, 30.0], egui::Label::new("goal direction weight"));
                    h_ui.spacing_mut().slider_width = 250.;
                    h_ui.add(egui::Slider::new(
                        &mut goal_direction_cost_weight,
                        0.0..=1.0,
                    ));
                });
                ui.label("");

                ui.horizontal(|h_ui| {
                    if h_ui
                        .add_sized([200., 30.], egui::Button::new("Reset weights"))
                        .clicked()
                    {
                        path_weight = DEFAULT_PATH_DISTANCE_WEIGHT as f32;
                        goal_weight = DEFAULT_GOAL_DISTANCE_WEIGHT as f32;
                        obstacle_weight = DEFAULT_OBSTACLE_DISTANCE_WEIGHT as f32;
                        local_goal_weight = DEFAULT_LOCAL_GOAL_DISTANCE_MAP_WEIHT as f32;
                        rotation_cost_weight = DEFAULT_ROTATION_COST_WEIGHT as f32;
                        path_direction_cost_weight = DEFAULT_PATH_DIRECTION_COST_WEIGHT as f32;
                        goal_direction_cost_weight = DEFAULT_GOAL_DIRECTION_COST_WEIGHT as f32;
                    }
                });

                ui.horizontal(|h_ui| {
                    if h_ui
                        .add_sized([200., 30.], egui::Button::new("Rerun"))
                        .clicked()
                    {
                        let mut is_run = res_nav.is_run.lock();
                        *is_run = true;
                    }
                });

                weight.insert(PATH_DISTANCE_MAP_NAME.to_owned(), path_weight as f64);
                weight.insert(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_weight as f64);
                weight.insert(
                    OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                    obstacle_weight as f64,
                );
                weight.insert(
                    LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                    local_goal_weight as f64,
                );
                weight.insert(ROTATION_COST_NAME.to_owned(), rotation_cost_weight as f64);
                weight.insert(
                    PATH_DIRECTION_COST_NAME.to_owned(),
                    path_direction_cost_weight as f64,
                );
                weight.insert(
                    GOAL_DIRECTION_COST_NAME.to_owned(),
                    goal_direction_cost_weight as f64,
                );
            }
            ui.label("");
            ui.separator();
            ui.label("");

            if ui
                .add_sized([200., 30.], egui::Button::new("Reload planner config"))
                .clicked()
            {
                let mut planner = res_nav.planner.lock();
                planner
                    .update_params_from_config(format!(
                        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
                        env!("CARGO_MANIFEST_DIR")
                    ))
                    .unwrap();
            }

            ui.label("");
            ui.separator();
            ui.label("");

            ui.checkbox(&mut ui_checkboxes.is_painting, "Painting Global Path");
        });
}

fn bottom_monitor_system(mut contexts: EguiContexts, res_nav: Res<NavigationViz>) {
    let ctx = contexts.ctx_mut();

    egui::TopBottomPanel::bottom("monitor")
        .default_height(150.)
        .show(ctx, |ui| {
            let angle_table = res_nav.angle_table.lock();

            ui.columns(angle_table.len(), |c_ui| {
                let mut circle_points = vec![];
                let num = 20;
                for i in 0..num {
                    let theta = i as f64 / num as f64 * 2. * std::f64::consts::PI;
                    circle_points.push([theta.cos(), theta.sin()]);
                }

                for (i, (name, angle)) in angle_table.iter().enumerate() {
                    c_ui[i].label(name);
                    Plot::new(format!("angle{}", i))
                        .data_aspect(1.)
                        .auto_bounds_x()
                        .auto_bounds_y()
                        .allow_drag(false)
                        .allow_scroll(false)
                        .allow_zoom(false)
                        .allow_boxed_zoom(false)
                        .allow_double_click_reset(false)
                        .show_axes([false, false])
                        .show_background(false)
                        .show(&mut c_ui[i], |plot_ui| {
                            plot_ui.polygon(
                                Polygon::new(PlotPoints::new(circle_points.clone()))
                                    .color(Color32::WHITE)
                                    .width(2.),
                            );
                            plot_ui.line(
                                Line::new(vec![[0., 0.], [angle.cos(), angle.sin()]])
                                    .color(Color32::RED)
                                    .width(3.),
                            );
                        });
                }
            });
        });
}

fn painting_ui_system(
    mut contexts: EguiContexts,
    mut ui_checkboxes: ResMut<UiCheckboxes>,
    mut painted_global_path: ResMut<PaintedGlobalPath>,
    res_nav: Res<NavigationViz>,
) {
    let ctx = contexts.ctx_mut();

    egui::Window::new("Paint Global Path")
        .open(&mut ui_checkboxes.is_painting)
        .default_size([600., 300.])
        .show(ctx, |paint_ui| {
            if paint_ui.button("Reset").clicked() {
                painted_global_path.lines.clear();
            }

            paint_ui.separator();

            let (mut response, painter) =
                paint_ui.allocate_painter(paint_ui.available_size_before_wrap(), Sense::drag());

            let to_screen = emath::RectTransform::from_to(
                bevy_egui::egui::Rect::from_min_size(
                    Pos2::ZERO,
                    response.rect.square_proportions(),
                ),
                response.rect,
            );
            let from_screen = to_screen.inverse();

            if painted_global_path.lines.is_empty() {
                painted_global_path.lines.push(vec![]);
            }

            let current_line = painted_global_path.lines.last_mut().unwrap();

            if let Some(pointer_pos) = response.interact_pointer_pos() {
                let canvas_pos = from_screen * pointer_pos;
                if current_line.last() != Some(&canvas_pos) {
                    current_line.push(canvas_pos);
                    response.mark_changed();
                }
            } else if !current_line.is_empty() {
                painted_global_path.lines.push(vec![]);
                response.mark_changed();
            }

            let map = { res_nav.empty_map.lock().copy_without_value() };
            let rects = convert_grid_map_obstacle_to_egui_mesh(&map);
            let grids = rects.iter().enumerate().map(|(idx, rect)| {
                egui::Shape::Rect(RectShape {
                    rect: bevy_egui::egui::Rect {
                        min: to_screen * rect.min,
                        max: to_screen * rect.max,
                    },
                    rounding: Rounding::none(),
                    fill: if idx == 0 {
                        Color32::WHITE
                    } else {
                        Color32::RED
                    },
                    stroke: Stroke {
                        width: 0.,
                        color: Color32::BLACK,
                    },
                })
            });
            painter.extend(grids);

            let shapes = painted_global_path
                .lines
                .iter()
                .filter(|line| line.len() >= 2)
                .map(|line| {
                    let points: Vec<Pos2> = line.iter().map(|p| to_screen * *p).collect();
                    egui::Shape::line(points, Stroke::new(2., Color32::BLUE))
                });

            painter.extend(shapes);

            response
        });
}
