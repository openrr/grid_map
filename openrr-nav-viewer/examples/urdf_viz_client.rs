use arci_urdf_viz::UrdfVizWebClient;
use grid_map::*;
use nalgebra::Vector2;
use openrr_nav::*;
use parking_lot::Mutex;
use std::sync::Arc;

fn new_sample_map() -> GridMap<u8> {
    let mut map = grid_map::GridMap::<u8>::new(
        Position::new(-1.05, -1.05),
        Position::new(10.05, 10.05),
        0.1,
    );

    let height = map.height();
    let width = map.width();
    for h in 0..height {
        map.set_obstacle(&Grid { x: 0, y: h });
        map.set_obstacle(&Grid { x: width, y: h });
    }
    for w in 0..width {
        map.set_obstacle(&Grid { x: w, y: 0 });
        map.set_obstacle(&Grid { x: w, y: height });
    }

    map
}

fn main() {
    let client = UrdfVizWebClient::default();
    client.run_send_velocity_thread();

    let planner = DwaPlanner::new_from_config(format!(
        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();

    let mut local_plan_executor = LocalPlanExecutor::new(
        Arc::new(Mutex::new(client.clone())),
        Arc::new(Mutex::new(client)),
        "".to_owned(),
        planner,
        0.1,
    );

    let mut map = new_sample_map();

    let start = [0.0, 0.0, 0.0];
    let goal = [9.0, 8.0, 1.0];

    let mut global_plan = GlobalPlan::new(map.clone(), start, goal);
    let result = global_plan.global_plan();

    let mut cost_maps = CostMaps::new(&map, &result, &start, &goal);
    let mut angle_table = AngleTable::new(start[2], goal[2]);

    for p in result.iter() {
        map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
    }

    local_plan_executor.set_cost_maps(cost_maps.layered_grid_map());
    local_plan_executor.set_angle_table(angle_table.angle_table());

    let mut current_pose;
    let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

    const STEP: usize = 1500;
    for i in 0..STEP {
        current_pose = local_plan_executor.current_pose().unwrap();

        cost_maps.update(
            &None,
            &result,
            &[current_pose.translation.x, current_pose.translation.y],
            &[],
        );

        angle_table.update(Some(current_pose), &result);

        local_plan_executor.set_cost_maps(cost_maps.layered_grid_map());
        local_plan_executor.set_angle_table(angle_table.angle_table());

        local_plan_executor.exec_once().unwrap();

        println!(
            "[ {:4} / {} ] X: {:.3}, Y: {:.3}, THETA: {:.3}",
            i + 1,
            STEP,
            current_pose.translation.x,
            current_pose.translation.y,
            current_pose.rotation.angle()
        );
        std::thread::sleep(std::time::Duration::from_millis(5));

        const GOAL_THRESHOLD_DISTANCE: f64 = 0.1;
        const GOAL_THRESHOLD_ANGLE_DIFFERENCE: f64 = 0.4;
        if (goal_pose.translation.vector - current_pose.translation.vector).norm()
            < GOAL_THRESHOLD_DISTANCE
            && (goal_pose.rotation.angle() - current_pose.rotation.angle()).abs()
                < GOAL_THRESHOLD_ANGLE_DIFFERENCE
        {
            println!("GOAL! count = {i}");
            break;
        }
    }
}
