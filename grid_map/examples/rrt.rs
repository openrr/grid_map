use rand::distributions::{Distribution, Uniform};

fn main() {
    use grid_map::*;
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-1.05, -1.05), Position::new(3.05, 1.05), 0.1);
    for i in 0..20 {
        map.set_value(&map.to_grid(0.2 + 0.1 * i as f64, -0.5).unwrap(), 1)
            .unwrap();
        for j in 0..10 {
            map.set_value(
                &map.to_grid(0.1 * i as f64, -0.2 + 0.1 * j as f64).unwrap(),
                1,
            )
            .unwrap();
        }
    }
    let x_range = Uniform::new(map.min_point().x, map.max_point().x);
    let y_range = Uniform::new(map.min_point().y, map.max_point().y);
    let result = rrt::dual_rrt_connect(
        &[0.5, -0.8],
        &[2.5, 0.5],
        |p: &[f64]| map.value(&map.to_grid(p[0], p[1]).unwrap()).is_none(),
        || {
            let mut rng = rand::thread_rng();
            vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
        },
        0.05,
        1000,
    )
    .unwrap();

    for p in result {
        map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
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
