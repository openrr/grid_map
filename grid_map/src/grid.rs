/// Grid coordinates for the map
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Grid {
    pub x: usize,
    pub y: usize,
}

impl Grid {
    pub fn new(x: usize, y: usize) -> Self {
        Self { x, y }
    }
    /// Returns neighbor grid Up/Down/Left/Right
    pub fn neighbors4(&self) -> Vec<Self> {
        let mut neighbors = vec![Self::new(self.x, self.y + 1), Self::new(self.x + 1, self.y)];
        if self.x != 0 {
            neighbors.push(Self::new(self.x - 1, self.y));
        }
        if self.y != 0 {
            neighbors.push(Self::new(self.x, self.y - 1));
        }
        neighbors
    }
}

impl PartialOrd for Grid {
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
mod grid_tests {
    use super::*;

    #[test]
    fn pos1() {
        let result = Grid::new(1, 1) < Grid::new(2, 3);
        assert_eq!(result, true);
        let result = Grid::new(1, 3) < Grid::new(2, 2);
        assert_eq!(result, false);
        let result = Grid::new(1, 3) < Grid::new(0, 2);
        assert_eq!(result, false);
        let result = Grid::new(2, 3) > Grid::new(1, 2);
        assert_eq!(result, true);
    }
}
