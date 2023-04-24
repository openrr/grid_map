use crate::cell::Cell;
use crate::position::Position;

/// Size of the map
#[derive(Copy, Clone, Debug, Default)]
pub struct Size {
    pub width: usize,
    pub height: usize,
}

impl Size {
    pub fn new(width: usize, height: usize) -> Self {
        Size { width, height }
    }
    pub fn len(&self) -> usize {
        self.width * self.height
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[derive(Clone, Debug)]
pub struct GridMap<T>
where
    T: Clone,
{
    resolution: f32,
    min_point: Position,
    max_point: Position,
    cells: Vec<Cell<T>>,
    size: Size,
}

impl<T> GridMap<T>
where
    T: Clone,
{
    pub fn new(min_point: Position, max_point: Position, resolution: f32) -> Self {
        assert!(max_point > min_point);
        let width = ((max_point.x - min_point.x) / resolution) as usize;
        let height = ((max_point.y - min_point.y) / resolution) as usize;
        let size = Size::new(width, height);
        let cells = vec![Cell::Unknown; size.len()];
        GridMap {
            resolution,
            min_point,
            max_point,
            cells,
            size,
        }
    }

    fn to_index(&self, position: &Position) -> Option<usize> {
        if position.x < self.min_point.x || position.y < self.min_point.y {
            return None;
        }
        let index = self.size.width * ((position.y - self.min_point.y) / self.resolution) as usize
            + ((position.x - self.min_point.x) / self.resolution) as usize;
        if self.cells.len() <= index {
            None
        } else {
            Some(index)
        }
    }

    pub fn cell(&self, position: &Position) -> Option<Cell<T>> {
        self.to_index(position)
            .map(|index| self.cells[index].clone())
    }

    pub fn cells(&self) -> &Vec<Cell<T>> {
        &self.cells
    }

    pub fn width(&self) -> usize {
        self.size.width
    }

    pub fn height(&self) -> usize {
        self.size.height
    }

    pub fn min_point(&self) -> Position {
        self.min_point
    }

    pub fn max_point(&self) -> Position {
        self.max_point
    }

    pub fn cell_mut(&mut self, position: &Position) -> Option<&mut Cell<T>> {
        match self.to_index(position) {
            Some(index) => Some(&mut self.cells[index]),
            None => None,
        }
    }

    pub fn set_value(&mut self, position: &Position, value: T) -> Option<()> {
        *self.cell_mut(position)? = Cell::Value(value);
        Some(())
    }

    pub fn value(&mut self, position: &Position) -> Option<T> {
        if let Cell::Value(value) = self.cell(position)? {
            Some(value)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_index() {
        let l = GridMap::<u8>::new(
            Position::new(0.1, 0.2),
            Position::new(0.5, 0.8),
            0.1,
        );
        assert_eq!(l.to_index(&Position::new(0.3, 0.4)).unwrap(), 10);
        assert_eq!(l.to_index(&Position::new(0.35, 0.4)).unwrap(), 10);
        assert_eq!(l.to_index(&Position::new(0.4, 0.4)).unwrap(), 11);
        assert!(l.to_index(&Position::new(0.0, 0.4)).is_none());
    }

    #[test]
    fn test_value() {
        let mut l = GridMap::new(
            Position::new(0.1, 0.2),
            Position::new(0.5, 0.8),
            0.1,
        );
        assert_eq!(l.cell(&Position::new(0.3, 0.4)).unwrap(), Cell::Unknown);
        l.set_value(&Position::new(0.3, 0.4), 1.0).unwrap();
        assert_eq!(l.cell(&Position::new(0.3, 0.4)).unwrap(), Cell::Value(1.0));
    }
}
