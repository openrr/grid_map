use bevy::prelude::*;
use bevy_egui::EguiContext;
use egui::plot::Plot;
use grid_map::{GridMap, RobotPath};
use parking_lot::Mutex;
use std::ops::RangeInclusive;
use std::sync::Arc;

use crate::{parse_grid_map_to_polygon, parse_robot_path_to_line};

#[derive(Resource)]
pub struct ArcGridMap(pub Arc<Mutex<GridMap<u8>>>);

#[derive(Resource)]
pub struct ArcRobotPath(pub Arc<Mutex<RobotPath>>);

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

            let poly = parse_grid_map_to_polygon(&dynamic_grid_map);
            for p in poly {
                plot_ui.polygon(p);
            }

            plot_ui.line(parse_robot_path_to_line(&dynamic_robot_path));
        })
    });
}
