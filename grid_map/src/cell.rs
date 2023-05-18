#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Cell<T>
where
    T: Clone,
{
    Uninitialized,
    Unknown,
    Obstacle,
    Value(T),
}

impl<T> Default for Cell<T>
where
    T: Clone,
{
    fn default() -> Self {
        Cell::Uninitialized
    }
}

impl<T> Cell<T>
where
    T: Clone,
{
    pub fn value(&self) -> Option<&T> {
        match self {
            Self::Value(v) => Some(v),
            _ => None,
        }
    }
    pub fn has_value(&self) -> bool {
        match self {
            Self::Value(_) => true,
            _ => false,
        }
    }
    pub fn is_uninitialized(&self) -> bool {
        match self {
            Self::Uninitialized => true,
            _ => false,
        }
    }
    pub fn is_obstacle(&self) -> bool {
        match self {
            Self::Obstacle => true,
            _ => false,
        }
    }

    pub fn from_value(value: T) -> Self {
        Self::Value(value)
    }
}
