// src/error.rs
use embedded_hal_async::i2c::Error as I2CError;
/// Custom error enum for the BME690 Rust driver.
#[derive(Debug)]
pub enum Error<E> {
    /// Error related to I2C communication.
    I2C(E),

    /// Error indicating an invalid chip ID.
    InvalidChipId,

    /// Error indicating a reading timeout.
    ReadingTimeout,
}

impl<E> From<E> for Error<E>
where
    E: I2CError,
{
    fn from(error: E) -> Self {
        Error::I2C(error)
    }
}
