use bevy::prelude::*;
use grid_map::*;
use openrr_nav::*;
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Resource)]
pub struct ResLayeredGridMap(pub Arc<Mutex<LayeredGridMap<u8>>>);

#[derive(Resource)]
pub struct ResNavRobotPath(pub Arc<Mutex<NavigationRobotPath>>);

#[derive(Resource)]
pub struct ResPose(pub Arc<Mutex<Pose>>);

#[derive(Resource)]
pub struct ResPosition(pub Arc<Mutex<Position>>);

#[derive(Resource)]
pub struct ResVecPosition(pub Arc<Mutex<Vec<Position>>>);

#[derive(Resource)]
pub struct ResBool(pub Arc<Mutex<bool>>);
