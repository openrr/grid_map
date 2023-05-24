use bevy::prelude::*;
use grid_map::LayeredGridMap;
use openrr_nav::*;
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Resource)]
pub struct ResLayeredGridMap(pub Arc<Mutex<LayeredGridMap<u8>>>);

#[derive(Resource)]
pub struct ResNavRobotPath(pub Arc<Mutex<NavigationRobotPath>>);

#[derive(Resource)]
pub struct ResPose(pub Arc<Mutex<Pose>>);
