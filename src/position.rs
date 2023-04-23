/// Real position for the map
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
}

impl Position {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

impl PartialOrd for Position {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        let comp_x = self.x.partial_cmp(&other.x)?;
        let comp_y = self.y.partial_cmp(&other.y)?;
        if comp_x == comp_y {
            Some(comp_x)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod pos_tests {
    use super::*;

    #[test]
    fn pos1() {
        let result = Position::new(-1.0, 1.0) < Position::new(0.0, 2.0);
        assert_eq!(result, true);
        let result = Position::new(-1.0, 3.0) < Position::new(0.0, 2.0);
        assert_eq!(result, false);
        let result = Position::new(-1.0, 3.0) < Position::new(-2.0, 2.0);
        assert_eq!(result, false);
        let result = Position::new(-1.0, 3.0) > Position::new(-2.0, 2.0);
        assert_eq!(result, true);
    }
}
