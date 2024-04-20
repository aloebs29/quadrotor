use embassy_nrf::twim;

// Basic intra-project error type
#[derive(Debug, defmt::Format)]
pub enum XError {
    I2c(twim::Error),
    Invalid,
    Timeout,
}

impl From<twim::Error> for XError {
    fn from(err: twim::Error) -> Self {
        XError::I2c(err)
    }
}

pub type XResult<T> = Result<T, XError>;
