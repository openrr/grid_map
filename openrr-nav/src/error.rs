use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("IO: {0}")]
    IoError(#[from] std::io::Error),
    #[error("grid_map: {0:?}")]
    GridError(#[from] grid_map::Error),
    #[error("{0}")]
    Other(String),
}

pub type Result<T> = std::result::Result<T, Error>;
