use crate::cell::Cell;
use crate::indices::Indices;
use crate::position::Position;

use std::fmt::Debug;

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
    resolution: f64,
    min_point: Position,
    max_point: Position,
    cells: Vec<Cell<T>>,
    size: Size,
}

impl<T> GridMap<T>
where
    T: Clone,
{
    pub fn new(min_point: Position, max_point: Position, resolution: f64) -> Self {
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

    pub fn to_index_by_position(&self, position: &Position) -> Option<usize> {
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

    pub fn to_index_by_indices(&self, indices: &Indices) -> Option<usize> {
        if indices.x >= self.width() || indices.y >= self.height() {
            return None;
        }
        Some(self.size.width * indices.y + indices.x)
    }

    // Get cell by raw position
    pub fn cell_by_position(&self, position: &Position) -> Option<Cell<T>> {
        self.to_index_by_position(position)
            .map(|index| self.cells[index].clone())
    }

    // Get cell by indices
    pub fn cell_by_indices(&self, indices: &Indices) -> Option<Cell<T>> {
        self.to_index_by_indices(indices)
            .map(|index| self.cells[index].clone())
    }

    pub fn cells(&self) -> &Vec<Cell<T>> {
        &self.cells
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn cells_mut(&mut self) -> &mut Vec<Cell<T>> {
        &mut self.cells
    }

    pub fn width(&self) -> usize {
        self.size.width
    }

    pub fn height(&self) -> usize {
        self.size.height
    }

    /// Return the minimum point in raw Position value
    pub fn min_point(&self) -> Position {
        self.min_point
    }

    /// Return the maximum point in raw Position value
    pub fn max_point(&self) -> Position {
        self.max_point
    }

    pub fn cell_by_position_mut(&mut self, position: &Position) -> Option<&mut Cell<T>> {
        match self.to_index_by_position(position) {
            Some(index) => Some(&mut self.cells[index]),
            None => None,
        }
    }

    pub fn cell_by_indices_mut(&mut self, indices: &Indices) -> Option<&mut Cell<T>> {
        match self.to_index_by_indices(indices) {
            Some(index) => Some(&mut self.cells[index]),
            None => None,
        }
    }

    pub fn set_value_by_position(&mut self, position: &Position, value: T) -> Option<()> {
        *self.cell_by_position_mut(position)? = Cell::Value(value);
        Some(())
    }

    pub fn set_value_by_indices(&mut self, indices: &Indices, value: T) -> Option<()> {
        *self.cell_by_indices_mut(indices)? = Cell::Value(value);
        Some(())
    }

    pub fn set_cell_by_position(&mut self, position: &Position, cell: Cell<T>) -> Option<()> {
        *self.cell_by_position_mut(position)? = cell;
        Some(())
    }

    pub fn set_cell_by_indices(&mut self, indices: &Indices, cell: Cell<T>) -> Option<()> {
        *self.cell_by_indices_mut(indices)? = cell;
        Some(())
    }

    pub fn value_by_position(&mut self, position: &Position) -> Option<T> {
        if let Cell::Value(value) = self.cell_by_position(position)? {
            Some(value)
        } else {
            None
        }
    }

    pub fn copy_without_value(&self) -> Self {
        let cells: Vec<_> = self
            .cells
            .iter()
            .map(|c| {
                if matches!(c, Cell::Value(_)) {
                    Cell::Uninitialized
                } else {
                    c.to_owned()
                }
            })
            .collect();

        Self {
            resolution: self.resolution,
            min_point: self.min_point,
            max_point: self.max_point,
            cells,
            size: self.size,
        }
    }

    pub fn resolution(&self) -> f64 {
        self.resolution
    }
}

impl<T> std::fmt::Display for GridMap<T>
where
    T: Clone + Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut displayed = String::new();
        for i in 0..self.height() {
            for j in 0..self.width() {
                match self.cells()[i * self.width() + j].value() {
                    Some(v) => displayed = format!("{}{:?}", displayed, v),
                    None => displayed = format!("{}-", displayed),
                }
            }
            displayed = format!("{}\n", displayed);
        }
        write!(f, "{}", displayed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_index() {
        let l = GridMap::<u8>::new(Position::new(0.1, 0.2), Position::new(0.5, 0.8), 0.1);
        assert_eq!(l.to_index_by_position(&Position::new(0.3, 0.4)).unwrap(), 9);
        assert_eq!(
            l.to_index_by_position(&Position::new(0.35, 0.4)).unwrap(),
            10
        );
        assert_eq!(
            l.to_index_by_position(&Position::new(0.4, 0.4)).unwrap(),
            11
        );
        assert!(l.to_index_by_position(&Position::new(0.0, 0.4)).is_none());
    }

    #[test]
    fn test_value() {
        let mut l = GridMap::new(Position::new(0.1, 0.2), Position::new(0.5, 0.8), 0.1);
        assert_eq!(
            l.cell_by_position(&Position::new(0.3, 0.4)).unwrap(),
            Cell::Unknown
        );
        l.set_value_by_position(&Position::new(0.3, 0.4), 1.0)
            .unwrap();
        assert_eq!(
            l.cell_by_position(&Position::new(0.3, 0.4)).unwrap(),
            Cell::Value(1.0)
        );
    }
}
