/// Alias for [`std::result::Result`] that wraps our [Error] type.
pub type Result<T> = std::result::Result<T, Error>;

/// Error type for this crate.
#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error(transparent)]
    Config(#[from] figment::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Python(#[from] pyo3::PyErr),
}

/// Convert [Error] to [`figment::Error`].
impl From<Error> for figment::Error {
    fn from(e: Error) -> Self {
        match e {
            Error::Config(e) => e,
            _ => figment::Error::from(e.to_string()),
        }
    }
}

/// Convert [Error] to [`pyo3::PyErr`].
impl From<Error> for pyo3::PyErr {
    fn from(e: Error) -> Self {
        match e {
            Error::Python(e) => e,
            _ => pyo3::exceptions::PyException::new_err(e.to_string()),
        }
    }
}
