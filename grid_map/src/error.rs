use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("IO: {0}")]
    IoError(#[from] std::io::Error),
    #[error("image crate: {0}")]
    ImageError(#[from] image::ImageError),
    #[error("yaml scan error: {0}")]
    YamlScanError(#[from] yaml_rust::ScanError),
    #[error("{0}")]
    Other(String),
}
