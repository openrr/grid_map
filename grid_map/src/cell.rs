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
    pub fn from_value(value: T) -> Self {
        Self::Value(value)
    }
}
