use embassy_nrf::twim;

// Basic intra-project error type
#[derive(Debug, defmt::Format)]
pub enum XError {
    I2c(twim::Error),
    Flash(nrf_softdevice::FlashError),
    Invalid,
    Timeout,
    CrcMismatch,
    SerializationFailure,
}

impl From<twim::Error> for XError {
    fn from(err: twim::Error) -> Self {
        XError::I2c(err)
    }
}

impl From<nrf_softdevice::FlashError> for XError {
    fn from(err: nrf_softdevice::FlashError) -> Self { XError::Flash(err) }
}

pub type XResult<T> = Result<T, XError>;
