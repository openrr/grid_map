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
