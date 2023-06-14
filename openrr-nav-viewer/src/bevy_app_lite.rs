use arci::{Localization, MoveBase};
use bevy::prelude::*;
use bevy_egui::{
    egui::{self, plot::Plot, Color32},
    EguiContexts, EguiPlugin,
};
use grid_map::Position;

use crate::*;

#[derive(Default)]
pub struct BevyAppLite {
    app: App,
}

impl BevyAppLite {
    pub fn new() -> Self {
        Self { app: App::new() }
    }

    pub fn setup<M: MoveBase + 'static, L: Localization + 'static>(
        &mut self,
        nav: NavigationVizLite<M, L>,
    ) {
        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "OpenRR Nav Viz Lite".to_owned(),
                resolution: (1920., 1080.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });

        let ui_checkboxes = UiCheckboxes::default();

        self.app
            .insert_resource(nav)
            .insert_resource(ui_checkboxes)
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .add_system(ui_system::<M, L>)
            .add_system(update_system::<M, L>);
    }

    pub fn run(&mut self) {
        self.app.run();
    }
}

fn update_system<M: MoveBase + 'static, L: Localization + 'static>(
    mut contexts: EguiContexts,
    res_nav: Res<NavigationVizLite<M, L>>,
    mut ui_checkboxes: ResMut<UiCheckboxes>,
) {
    let ctx = contexts.ctx_mut();

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("Map").data_aspect(1.).show(ui, |plot_ui| {
            // Plot map
            let map = res_nav.grid_map.lock();
            for p in grid_map_to_polygon(&map) {
                plot_ui.polygon(p);
            }

            // Plot robot pose
            let pose = {
                let localization = res_nav.localization.lock();
                localization.current_pose("").unwrap()
            };
            plot_ui.polygon(robot_pose_to_polygon(&pose, Color32::DARK_RED, 1.));

            let pointer_coordinate = plot_ui.pointer_coordinate();

            if let Some(p) = pointer_coordinate {
                if ui_checkboxes.set_goal
                    && ctx.input(|i| i.pointer.button_pressed(egui::PointerButton::Primary))
                    && !ctx.is_pointer_over_area()
                {
                    ui_checkboxes.set_goal = false;
                    let mut goal_position = res_nav.goal.lock();
                    *goal_position = Position::new(p.x, p.y);
                    let mut is_run = res_nav.is_run.lock();
                    *is_run = true;
                }
            }
        });
    });
}

fn ui_system<M: MoveBase + 'static, L: Localization + 'static>(
    mut contexts: EguiContexts,
    res_nav: Res<NavigationVizLite<M, L>>,
    mut ui_checkboxes: ResMut<UiCheckboxes>,
) {
    let ctx = contexts.ctx_mut();

    egui::SidePanel::left("left_side_panel")
        .default_width(200.)
        .show(ctx, |ui| {
            ui.horizontal(|h_ui| {
                if h_ui.button("Set Goal").clicked() {
                    ui_checkboxes.set_goal = !ui_checkboxes.set_goal;
                }
            });
            ui.label(if ui_checkboxes.set_goal {
                "Set goal   "
            } else {
                "Choose mode"
            });

            if ui.button("Run").clicked() {
                let mut is_run = res_nav.is_run.lock();
                *is_run = true;
            }
        });
}
