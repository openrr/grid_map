mod bevy_app;
mod converter;
mod map_type;
mod nav_viz;

pub use bevy_app::*;
pub use converter::*;
pub use map_type::*;
pub use nav_viz::*;

pub mod pb {
    tonic::include_proto!("openrr_nav_viewer");
}

use std::collections::HashMap;

#[tonic::async_trait]
impl pb::api_server::Api for NavigationViz {
    async fn get_start_position(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::Position>, tonic::Status> {
        let start = self.start_position.lock();
        Ok(tonic::Response::new((*start).into()))
    }
    async fn get_goal_position(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::Position>, tonic::Status> {
        let goal = self.goal_position.lock();
        Ok(tonic::Response::new((*goal).into()))
    }
    async fn get_planner(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::DwaPlanner>, tonic::Status> {
        let planner = self.planner.lock().clone();
        Ok(tonic::Response::new(pb::DwaPlanner {
            limits: Some(planner.limits().clone().into()),
            map_name_weight: Some(pb::WeightsResponse {
                weights: planner.map_name_weight().clone(),
            }),
            controller_dt: planner.controller_dt(),
            simulation_duration: planner.simulation_duration(),
            num_vel_sample: planner.num_vel_sample(),
        }))
    }
    async fn get_layered_grid_map(
        &self,
        request: tonic::Request<pb::GetLayeredGridMapRequest>,
    ) -> Result<tonic::Response<pb::LayeredGridMap>, tonic::Status> {
        let names = request.into_inner().names;
        let mut maps = HashMap::with_capacity(names.len());
        let layered_grid_map = self.layered_grid_map.lock();
        for name in names {
            match layered_grid_map.layer(&name) {
                Some(map) => {
                    maps.insert(name, map.into());
                }
                None => {
                    return Err(tonic::Status::invalid_argument(format!(
                        "grid map '{name}' not found"
                    )));
                }
            }
        }
        Ok(tonic::Response::new(pb::LayeredGridMap { maps }))
    }
    async fn get_is_run(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::IsRun>, tonic::Status> {
        Ok(tonic::Response::new(pb::IsRun {
            is_run: *self.is_run.lock(),
        }))
    }
    async fn set_global_path(
        &self,
        request: tonic::Request<pb::RobotPath>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        self.robot_path
            .lock()
            .set_global_path(request.into_inner().into());
        Ok(tonic::Response::new(()))
    }
    async fn set_local_path_and_candidates(
        &self,
        request: tonic::Request<pb::PathAndCandidates>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let pb::PathAndCandidates { path, candidates } = request.into_inner();
        let mut robot_path = self.robot_path.lock();
        robot_path.set_local_path(path.unwrap().into());
        for (i, candidate) in candidates.into_iter().enumerate() {
            robot_path.add_user_defined_path(
                &format!("candidate_{i}"),
                openrr_nav::RobotPath(candidate.path.into_iter().map(Into::into).collect()),
            );
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_layered_grid_map(
        &self,
        request: tonic::Request<pb::SetLayeredGridMapRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let pb::SetLayeredGridMapRequest { maps } = request.into_inner();
        let mut layered_grid_map = self.layered_grid_map.lock();
        for named_map in maps {
            layered_grid_map.add_layer(named_map.name, named_map.map.unwrap().into());
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_current_pose(
        &self,
        request: tonic::Request<pb::Isometry2>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        *self.robot_pose.lock() = request.into_inner().into();
        Ok(tonic::Response::new(()))
    }
    async fn set_is_run(
        &self,
        request: tonic::Request<pb::IsRun>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        *self.is_run.lock() = request.into_inner().is_run;
        Ok(tonic::Response::new(()))
    }
}

impl From<openrr_nav::RobotPath> for pb::RobotPath {
    fn from(val: openrr_nav::RobotPath) -> Self {
        Self {
            path: val.0.into_iter().map(Into::into).collect(),
        }
    }
}
impl From<pb::RobotPath> for openrr_nav::RobotPath {
    fn from(val: pb::RobotPath) -> Self {
        Self(val.path.into_iter().map(Into::into).collect())
    }
}

impl From<grid_map::Position> for pb::Position {
    fn from(val: grid_map::Position) -> Self {
        Self { x: val.x, y: val.y }
    }
}
impl From<pb::Position> for grid_map::Position {
    fn from(val: pb::Position) -> Self {
        Self { x: val.x, y: val.y }
    }
}

impl From<pb::Velocity> for openrr_nav::Velocity {
    fn from(value: pb::Velocity) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}
impl From<openrr_nav::Velocity> for pb::Velocity {
    fn from(value: openrr_nav::Velocity) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}

impl From<pb::Acceleration> for openrr_nav::Acceleration {
    fn from(value: pb::Acceleration) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}
impl From<openrr_nav::Acceleration> for pb::Acceleration {
    fn from(value: openrr_nav::Acceleration) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}

impl From<pb::Limits> for openrr_nav::Limits {
    fn from(value: pb::Limits) -> Self {
        Self {
            max_velocity: value.max_velocity.unwrap().into(),
            max_accel: value.max_accel.unwrap().into(),
            min_velocity: value.min_velocity.unwrap().into(),
            min_accel: value.min_accel.unwrap().into(),
        }
    }
}
impl From<openrr_nav::Limits> for pb::Limits {
    fn from(value: openrr_nav::Limits) -> Self {
        Self {
            max_velocity: Some(value.max_velocity.into()),
            max_accel: Some(value.max_accel.into()),
            min_velocity: Some(value.min_velocity.into()),
            min_accel: Some(value.min_accel.into()),
        }
    }
}

impl From<openrr_nav::Plan> for pb::Plan {
    fn from(val: openrr_nav::Plan) -> Self {
        Self {
            velocity: Some(pb::Velocity {
                x: val.velocity.x,
                theta: val.velocity.theta,
            }),
            cost: val.cost,
            path: val.path.into_iter().map(Into::into).collect(),
        }
    }
}
impl From<pb::Plan> for openrr_nav::Plan {
    fn from(val: pb::Plan) -> Self {
        let velocity = val.velocity.unwrap();
        Self {
            velocity: openrr_nav::Velocity {
                x: velocity.x,
                theta: velocity.theta,
            },
            cost: val.cost,
            path: val.path.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<&grid_map::GridMap<u8>> for pb::GridMap {
    fn from(val: &grid_map::GridMap<u8>) -> Self {
        Self {
            grid_converter: Some(pb::GridPositionConverter {
                resolution: val.resolution(),
                min_point: Some((*val.min_point()).into()),
                max_point: Some((*val.max_point()).into()),
                size: Some(pb::Size {
                    width: val.width() as _,
                    height: val.height() as _,
                }),
            }),
            cells: val.cells().iter().map(|c| (*c).into()).collect(),
        }
    }
}
impl From<pb::GridMap> for grid_map::GridMap<u8> {
    fn from(val: pb::GridMap) -> Self {
        let grid_converter = val.grid_converter.unwrap();
        let mut map = grid_map::GridMap::new(
            grid_converter.min_point.unwrap().into(),
            grid_converter.max_point.unwrap().into(),
            grid_converter.resolution,
        );
        assert_eq!(val.cells.len(), map.len());
        for (from, to) in val.cells.into_iter().zip(map.cells_mut()) {
            *to = from.into();
        }
        map
    }
}

impl From<grid_map::Cell<u8>> for pb::Cell {
    fn from(val: grid_map::Cell<u8>) -> Self {
        let kind = match val {
            grid_map::Cell::Uninitialized => pb::cell::Kind::Uninitialized(()),
            grid_map::Cell::Unknown => pb::cell::Kind::Unknown(()),
            grid_map::Cell::Obstacle => pb::cell::Kind::Obstacle(()),
            grid_map::Cell::Value(b) => pb::cell::Kind::Value(b as _),
        };
        Self { kind: Some(kind) }
    }
}
impl From<pb::Cell> for grid_map::Cell<u8> {
    fn from(val: pb::Cell) -> Self {
        let val = val.kind.unwrap();
        match val {
            pb::cell::Kind::Uninitialized(()) => Self::Uninitialized,
            pb::cell::Kind::Unknown(()) => Self::Unknown,
            pb::cell::Kind::Obstacle(()) => Self::Obstacle,
            pb::cell::Kind::Value(v) => Self::Value(v as u8),
        }
    }
}

impl From<nalgebra::Isometry2<f64>> for pb::Isometry2 {
    fn from(val: nalgebra::Isometry2<f64>) -> Self {
        Self {
            rotation: Some(pb::UnitComplex {
                re: val.rotation.re,
                im: val.rotation.im,
            }),
            translation: Some(pb::Translation2 {
                x: val.translation.x,
                y: val.translation.y,
            }),
        }
    }
}
impl From<pb::Isometry2> for nalgebra::Isometry2<f64> {
    fn from(val: pb::Isometry2) -> Self {
        let translation = val.translation.unwrap();
        let rotation = val.rotation.unwrap();
        Self::from_parts(
            nalgebra::Translation2::new(translation.x, translation.y),
            nalgebra::UnitComplex::from_complex(nalgebra::Complex {
                re: rotation.re,
                im: rotation.im,
            }),
        )
    }
}
