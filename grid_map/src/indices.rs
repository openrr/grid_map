/// Real Indices for the map
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Indices {
    pub x: usize,
    pub y: usize,
}

impl Indices {
    pub fn new(x: usize, y: usize) -> Self {
        Self { x, y }
    }
    /// Returns neighbor indices Up/Down/Left/Right
    pub fn neighbors4(&self) -> Vec<Self> {
        let mut neighbors = vec![
            Self::new(self.x, self.y + 1),
            Self::new(self.x + 1, self.y)];
        if self.x != 0 {
            neighbors.push(Self::new(self.x - 1, self.y));
        }
        if self.y != 0 {
            neighbors.push(Self::new(self.x, self.y - 1));
        }
        neighbors
    }
}

impl PartialOrd for Indices {
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
mod indices_tests {
    use super::*;

    #[test]
    fn pos1() {
        let result = Indices::new(1, 1) < Indices::new(2, 3);
        assert_eq!(result, true);
        let result = Indices::new(1, 3) < Indices::new(2, 2);
        assert_eq!(result, false);
        let result = Indices::new(1, 3) < Indices::new(0, 2);
        assert_eq!(result, false);
        let result = Indices::new(2, 3) > Indices::new(1, 2);
        assert_eq!(result, true);
    }
}
