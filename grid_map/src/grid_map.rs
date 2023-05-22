use crate::cell::Cell;
use crate::grid::Grid;
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
pub struct GridPositionConverter {
    resolution: f64,
    min_point: Position,
    max_point: Position,
    size: Size,
}

impl GridPositionConverter {
    /// Create grid position converter
    pub fn new(min_point: Position, max_point: Position, resolution: f64) -> Self {
        let width = ((max_point.x - min_point.x) / resolution) as usize;
        let height = ((max_point.y - min_point.y) / resolution) as usize;
        let size = Size::new(width, height);
        Self {
            resolution,
            min_point,
            max_point,
            size,
        }
    }
    pub fn resolution(&self) -> f64 {
        self.resolution
    }
    pub fn min_point(&self) -> &Position {
        &self.min_point
    }
    pub fn max_point(&self) -> &Position {
        &self.max_point
    }
    pub fn size(&self) -> &Size {
        &self.size
    }
    pub fn to_grid(&self, position: &Position) -> Option<Grid> {
        if position.x < self.min_point.x || position.y < self.min_point.y {
            return None;
        }
        let x = ((position.x - self.min_point.x) / self.resolution) as usize;
        let y = ((position.y - self.min_point.y) / self.resolution) as usize;
        if x >= self.size.width || y >= self.size.height {
            return None;
        }
        Some(Grid { x, y })
    }
    pub fn to_index(&self, grid: &Grid) -> Option<usize> {
        if grid.x >= self.size.width || grid.y >= self.size.height {
            return None;
        }
        let index = self.size.width * grid.y + grid.x;
        if self.size.len() <= index {
            None
        } else {
            Some(index)
        }
    }

    pub fn to_grid_from_index(&self, index: usize) -> Option<Grid> {
        if index >= self.size.len() {
            return None;
        }
        let rows = index / self.size.width;
        let cols = index - rows * self.size.width;
        Some(Grid { x: cols, y: rows })
    }
}

#[derive(Clone, Debug)]
pub struct GridMap<T>
where
    T: Clone,
{
    grid_converter: GridPositionConverter,
    cells: Vec<Cell<T>>,
}

impl<T> GridMap<T>
where
    T: Clone,
{
    pub fn new(min_point: Position, max_point: Position, resolution: f64) -> Self {
        assert!(max_point > min_point);
        let grid_converter = GridPositionConverter::new(min_point, max_point, resolution);
        let cells = vec![Cell::Uninitialized; grid_converter.size().len()];
        GridMap {
            grid_converter,
            cells,
        }
    }

    fn to_index(&self, grid: &Grid) -> Option<usize> {
        self.grid_converter.to_index(grid)
    }

    /// Convert position into grid
    pub fn to_grid(&self, position: &Position) -> Option<Grid> {
        //let index = self.to_index_by_position(position)?;
        //self.to_grid_from_index(index)
        self.grid_converter.to_grid(position)
    }

    // Get cell by grid
    pub fn cell(&self, grid: &Grid) -> Option<&Cell<T>> {
        self.to_index(grid).map(|index| &self.cells[index])
    }

    pub fn cells(&self) -> &Vec<Cell<T>> {
        &self.cells
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn cells_mut(&mut self) -> &mut Vec<Cell<T>> {
        &mut self.cells
    }

    pub fn width(&self) -> usize {
        self.grid_converter.size().width
    }

    pub fn height(&self) -> usize {
        self.grid_converter.size().height
    }

    /// Return the minimum point in raw Position value
    pub fn min_point(&self) -> &Position {
        self.grid_converter.min_point()
    }

    /// Return the maximum point in raw Position value
    pub fn max_point(&self) -> &Position {
        self.grid_converter.max_point()
    }

    pub fn cell_mut(&mut self, grid: &Grid) -> Option<&mut Cell<T>> {
        match self.to_index(grid) {
            Some(index) => Some(&mut self.cells[index]),
            None => None,
        }
    }

    pub fn set_value(&mut self, grid: &Grid, value: T) -> Option<()> {
        *self.cell_mut(grid)? = Cell::Value(value);
        Some(())
    }

    pub fn value(&self, grid: &Grid) -> Option<T> {
        match self.cell(grid) {
            Some(Cell::Value(v)) => Some(v.to_owned()),
            _ => None,
        }
    }

    pub fn set_obstacle(&mut self, grid: &Grid) -> Option<()> {
        *self.cell_mut(grid)? = Cell::Obstacle;
        Some(())
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
            grid_converter: self.grid_converter.clone(),
            cells,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_index() {
        let map = GridMap::<u8>::new(Position::new(0.1, 0.2), Position::new(0.5, 0.8), 0.1);
        assert_eq!(
            map.to_index(&map.to_grid(&Position::new(0.3, 0.4)).unwrap())
                .unwrap(),
            9
        );
        assert_eq!(
            map.to_index(&map.to_grid(&Position::new(0.35, 0.4)).unwrap())
                .unwrap(),
            10
        );
        assert_eq!(
            map.to_index(&map.to_grid(&Position::new(0.4, 0.4)).unwrap())
                .unwrap(),
            11
        );
        assert!(&map.to_grid(&Position::new(0.0, 0.4)).is_none());
    }

    #[test]
    fn test_value() {
        let mut map = GridMap::new(Position::new(0.1, 0.2), Position::new(0.5, 0.8), 0.1);
        assert_eq!(
            *map.cell(&map.to_grid(&Position::new(0.3, 0.4)).unwrap())
                .unwrap(),
            Cell::Uninitialized
        );
        map.set_value(&map.to_grid(&Position::new(0.3, 0.4)).unwrap(), 1.0)
            .unwrap();
        assert_eq!(
            *map.cell(&map.to_grid(&Position::new(0.3, 0.4)).unwrap())
                .unwrap(),
            Cell::Value(1.0)
        );
    }
}
