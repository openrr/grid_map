use rand::distributions::{Distribution, Uniform};
use rrt;

fn main() {
    use grid_map::*;
    let mut map = grid_map::GridMap::<u8>::new(
        Position::new(-1.05, -1.05),
        Position::new(3.05, 1.05),
        0.1,
    );
    for i in 0..20 {
        map.set_value(&Position::new(0.2 + 0.1 * i as f32, -0.5), 1)
            .unwrap();
        for j in 0..10 {
            map.set_value(&Position::new(0.1 * i as f32, -0.2 + 0.1 * j as f32), 1)
                .unwrap();
        }
    }
    let x_range = Uniform::new(map.min_point().x, map.max_point().x);
    let y_range = Uniform::new(map.min_point().y, map.max_point().y);
    let result = rrt::dual_rrt_connect(
        &[0.5, -0.8],
        &[2.5, 0.5],
        |p: &[f32]| map.value(&Position::new(p[0], p[1])).is_none(),
        || {
            let mut rng = rand::thread_rng();
            vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
        },
        0.05,
        1000,
    )
    .unwrap();

    for p in result {
        map.set_value(&Position::new(p[0], p[1]), 0).unwrap();
    }
    for i in 0..map.height() {
        for j in 0..map.width() {
            match map.cells()[i * map.width() + j].value() {
                Some(v) => print!("{v}"),
                None => print!("-"),
            }
        }
        println!("");
    }
}
