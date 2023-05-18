use egui::plot::{Line, PlotPoints, Polygon};
use grid_map::{Cell, GridMap, RobotPath};

pub fn parse_grid_map_to_polygon(grid_map: &GridMap<u8>) -> Vec<Polygon> {
    let min_point = grid_map.min_point();
    let resolution = grid_map.resolution();
    let width = grid_map.width();
    let cells = grid_map.cells();

    let mut polygons = Vec::<Polygon>::new();

    for (i, cell_i) in cells.iter().enumerate() {
        let order = [0, 1, 3, 2];
        let plot_points: PlotPoints = order
            .iter()
            .map(|j| {
                let x = min_point.x + (i % width + j % 2) as f64 * resolution;
                let y = min_point.y + (i / width + j / 2) as f64 * resolution;
                [x, y]
            })
            .collect();
        let polygon = match cell_i {
            Cell::Unknown => Polygon::new(plot_points)
                .color(egui::Color32::from_gray(120))
                .fill_alpha(1.0),
            Cell::Value(v) => Polygon::new(plot_points)
                .color(egui::Color32::from_rgb(*v * 2, 120, 120))
                .fill_alpha(1.0),
            _ => Polygon::new(plot_points)
                .color(egui::Color32::from_gray(0))
                .fill_alpha(1.0),
        };
        polygons.push(polygon);
    }

    polygons
}

pub fn parse_robot_path_to_line(robot_path: &RobotPath) -> Line {
    let plot_points: PlotPoints = robot_path
        .get_vec()
        .iter()
        .map(|&p| [p.translation.x, p.translation.y])
        .collect();
    Line::new(plot_points).width(10.)
}
