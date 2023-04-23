#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Cell<T>
where
    T: Clone,
{
    Unknown,
    Value(T),
}

impl<T> Default for Cell<T>
where
    T: Clone,
{
    fn default() -> Self {
        Cell::Unknown
    }
}

/// Real position for the map
#[derive(Copy, Clone, Debug, Default)]
pub struct Position {
    pub x: f32,
    pub y: f32,
}

impl Position {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

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
pub struct Layer<T>
where
    T: Clone,
{
    name: String,
    resolution: f32,
    origin: Position,
    cells: Vec<Cell<T>>,
    size: Size,
}

impl<T> Layer<T>
where
    T: Clone,
{
    pub fn new(name: String, resolution: f32, min_point: Position, max_point: Position) -> Self {
        assert!(max_point.x > min_point.x);
        assert!(max_point.y > min_point.y);
        let width = ((max_point.x - min_point.x) / resolution) as usize;
        let height = ((max_point.y - min_point.y) / resolution) as usize;
        let size = Size::new(width, height);
        let cells = vec![Cell::Unknown; size.len()];
        Layer {
            name,
            resolution,
            origin: min_point,
            cells,
            size,
        }
    }
    pub fn name(&self) -> &str {
        &self.name
    }

    fn to_index(&self, position: &Position) -> Option<usize> {
        if position.x < self.origin.x || position.y < self.origin.y {
            return None;
        }
        let index = self.size.width * ((position.y - self.origin.y) / self.resolution) as usize
            + ((position.x - self.origin.x) / self.resolution) as usize;
        let len = self.cells.len();
        assert!(len > index, "len = {len}, ind = {index}");
        Some(index)
    }

    pub fn cell(&self, position: &Position) -> Option<Cell<T>> {
        self.to_index(position)
            .map(|index| self.cells[index].clone())
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name() {
        let l = Layer::new(
            "a".to_string(),
            0.1,
            Position::new(0.1, 0.2),
            Position::new(0.5, 0.8),
        );
        assert_eq!(l.name, "a");
    }
    #[test]
    fn test_to_index() {
        let l = Layer::new(
            "a".to_string(),
            0.1,
            Position::new(0.1, 0.2),
            Position::new(0.5, 0.8),
        );
        assert_eq!(l.to_index(&Position::new(0.3, 0.4)).unwrap(), 10);
        assert_eq!(l.to_index(&Position::new(0.35, 0.4)).unwrap(), 10);
        assert_eq!(l.to_index(&Position::new(0.4, 0.4)).unwrap(), 11);
        assert!(l.to_index(&Position::new(0.0, 0.4)).is_none());
    }

    #[test]
    fn test_Value() {
        let mut l = Layer::new(
            "a".to_string(),
            0.1,
            Position::new(0.1, 0.2),
            Position::new(0.5, 0.8),
        );
        assert_eq!(l.Cell(&Position::new(0.3, 0.4)).unwrap(), Cell::Unknown);
        l.set_Cell(&Position::new(0.3, 0.4), 1.0).unwrap();
        assert_eq!(l.Cell(&Position::new(0.3, 0.4)).unwrap(), Cell::Value(1.0));
    }
}
