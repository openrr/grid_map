#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub enum Cell<T>
where
    T: Clone,
{
    #[default]
    Uninitialized,
    Unknown,
    Obstacle,
    Value(T),
}
/*
impl<T> Default for Cell<T>
where
    T: Clone,
{
    fn default() -> Self {
        Cell::Uninitialized
    }
}
 */
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
        matches!(self, Self::Value(_))
    }
    pub fn is_uninitialized(&self) -> bool {
        matches!(self, Self::Uninitialized)
    }
    pub fn is_obstacle(&self) -> bool {
        matches!(self, Self::Obstacle)
    }

    pub fn from_value(value: T) -> Self {
        Self::Value(value)
    }
}
