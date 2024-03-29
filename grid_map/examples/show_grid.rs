fn main() {
    use grid_map::*;
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-1.05, -1.05), Position::new(3.05, 1.05), 0.1);
    for i in 0..10 {
        map.set_value(&map.to_grid(0.5 + 0.2 * i as f64, -0.5).unwrap(), i)
            .unwrap();
        for j in 0..10 {
            map.set_value(
                &map.to_grid(0.1 * i as f64, -0.2 + 0.1 * j as f64).unwrap(),
                j,
            )
            .unwrap();
        }
    }
    for i in 0..map.height() {
        for j in 0..map.width() {
            match map.cells()[i * map.width() + j].value() {
                Some(v) => print!("{v}"),
                None => print!("-"),
            }
        }
        println!();
    }
}
