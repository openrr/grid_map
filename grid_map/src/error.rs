use crate::Grid;
use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("IO: {0}")]
    IoError(#[from] std::io::Error),
    #[error("image crate: {0}")]
    ImageError(#[from] image::ImageError),
    #[error("yaml parse error: {0}")]
    YamlParseError(#[from] serde_yaml::Error),
    #[error("out of range grid: {0:?}")]
    OutOfRangeGrid(Grid),
    #[error("out of range {0}, {1}")]
    OutOfRangePosition(f64, f64),
    #[error("{0}")]
    Other(String),
}

pub type Result<T> = std::result::Result<T, Error>;
