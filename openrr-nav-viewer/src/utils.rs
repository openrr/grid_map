use bevy_egui::egui::{Pos2, Rect, Vec2};
use grid_map::{Grid, GridMap, Position};

pub fn convert_grid_map_obstacle_to_egui_mesh(grid_map: &GridMap<u8>) -> Vec<Rect> {
    let mut rects = vec![];

    let min_point = grid_map.min_point();
    let max_point = grid_map.max_point();

    let area = Rect {
        min: position_to_egui_pos2(min_point),
        max: position_to_egui_pos2(max_point),
    };
    rects.push(area);

    let resolution = grid_map.resolution();
    let width = grid_map.width();
    let height = grid_map.height();

    for w in 0..width {
        for h in 0..height {
            if let Some(c) = grid_map.cell(&Grid { x: w, y: h }) {
                if let grid_map::Cell::Obstacle = c {
                    let pos2 = Pos2 {
                        x: (min_point.x + w as f64 * resolution) as f32,
                        y: (min_point.y + h as f64 * resolution) as f32,
                    };
                    rects.push(Rect {
                        min: pos2,
                        max: pos2
                            + Vec2 {
                                x: resolution as f32,
                                y: resolution as f32,
                            },
                    })
                }
            }
        }
    }

    rects
}

pub fn position_to_egui_pos2(position: &Position) -> Pos2 {
    Pos2 {
        x: position.x as f32,
        y: position.y as f32,
    }
}
