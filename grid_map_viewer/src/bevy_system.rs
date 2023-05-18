use bevy::prelude::*;
use bevy_egui::EguiContext;
use egui::plot::Plot;
use grid_map::{GridMap, NavigationGridMap, NavigationRobotPath, RobotPath};
use parking_lot::Mutex;
use std::ops::RangeInclusive;
use std::sync::Arc;

use crate::{parse_grid_map_to_polygon, parse_robot_path_to_line};

#[derive(Resource)]
pub struct ArcGridMap(pub Arc<Mutex<GridMap<u8>>>);

#[derive(Resource)]
pub struct ArcNavGridMap(pub Arc<Mutex<NavigationGridMap<u8>>>);

#[derive(Resource)]
pub struct ArcRobotPath(pub Arc<Mutex<RobotPath>>);

#[derive(Resource)]
pub struct ArcNavRobotPath(pub Arc<Mutex<NavigationRobotPath>>);

pub fn bevy_ui_system_for_arc(mut contexts: ResMut<EguiContext>, grid_map: ResMut<ArcGridMap>) {
    let raw_input = egui::RawInput::default();
    let ctx = contexts.ctx_mut();
    ctx.begin_frame(raw_input);

    egui::SidePanel::new(egui::panel::Side::Left, "UI")
        .width_range(RangeInclusive::new(100., 200.))
        .show(ctx, |ui| {
            let _ = ui.button("Map");
        });

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Grid").data_aspect(1.).show(ui, |plot_ui| {
            let dynamic_grid_map = grid_map.0.lock();

            let poly = parse_grid_map_to_polygon(&dynamic_grid_map);
            for p in poly {
                plot_ui.polygon(p);
            }
        })
    });
}

pub fn bevy_ui_system_for_arc2(
    mut contexts: ResMut<EguiContext>,
    grid_map: ResMut<ArcGridMap>,
    robot_path: ResMut<ArcRobotPath>,
) {
    let raw_input = egui::RawInput::default();
    let ctx = contexts.ctx_mut();
    ctx.begin_frame(raw_input);

    egui::SidePanel::new(egui::panel::Side::Left, "UI")
        .width_range(RangeInclusive::new(100., 200.))
        .show(ctx, |ui| {
            let _ = ui.button("Map");
        });

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Grid").data_aspect(1.).show(ui, |plot_ui| {
            let dynamic_grid_map = grid_map.0.lock();
            let dynamic_robot_path = robot_path.0.lock();

            for p in parse_grid_map_to_polygon(&dynamic_grid_map) {
                plot_ui.polygon(p);
            }

            plot_ui.line(parse_robot_path_to_line(&dynamic_robot_path));
        })
    });
}

pub fn bevy_ui_system_for_navigation(
    mut contexts: ResMut<EguiContext>,
    nav_grid_map: ResMut<ArcNavGridMap>,
    nav_robot_path: ResMut<ArcNavRobotPath>,
) {
    let raw_input = egui::RawInput::default();
    let ctx = contexts.ctx_mut();
    ctx.begin_frame(raw_input);

    egui::SidePanel::new(egui::panel::Side::Left, "UI")
        .width_range(RangeInclusive::new(100., 200.))
        .show(ctx, |ui| {
            let _ = ui.button("Map");
        });

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Grid").data_aspect(1.).show(ui, |plot_ui| {
            let dynamic_nav_grid_map = nav_grid_map.0.lock();
            let dynamic_nav_robot_path = nav_robot_path.0.lock();

            for local_map in dynamic_nav_grid_map.local_map().layer_as_iter() {
                for p in parse_grid_map_to_polygon(&local_map.1) {
                    plot_ui.polygon(p);
                }
            }

            for global_map in dynamic_nav_grid_map.global_map().layer_as_iter() {
                for p in parse_grid_map_to_polygon(&global_map.1) {
                    plot_ui.polygon(p);
                }
            }

            plot_ui.line(parse_robot_path_to_line(
                dynamic_nav_robot_path.local_path(),
            ));
            plot_ui.line(parse_robot_path_to_line(
                dynamic_nav_robot_path.global_path(),
            ));
        })
    });
}
