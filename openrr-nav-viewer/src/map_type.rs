use bevy::prelude::*;

#[derive(Debug, Clone, Default, Resource, PartialEq, PartialOrd)]
pub enum MapType {
    #[default]
    PathDistanceMap,
    GoalDistanceMap,
    ObstacleDistanceMap,
}
