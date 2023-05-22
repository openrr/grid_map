/// Utility for debug
pub fn show_ascii_map(map: &grid_map::GridMap<u8>, scale: f32) {
    use grid_map::Cell;
    for i in 0..map.height() {
        for j in 0..map.width() {
            let letter = match map.cells()[i * map.width() + j] {
                Cell::Value(v) => {
                    let v = (v as f32 * scale) as u8;
                    if v <= 9 {
                        format!("{v}")
                    } else {
                        "9".to_owned()
                    }
                }
                Cell::Obstacle => "x".to_owned(),
                Cell::Uninitialized => "u".to_owned(),
                Cell::Unknown => "?".to_owned(),
            };
            print!("{letter:}");
        }
        println!();
    }
}
