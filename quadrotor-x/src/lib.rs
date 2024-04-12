#![cfg_attr(not(feature = "std"), no_std)]

mod lib {
    #[cfg(feature = "defmt")]
    pub use defmt::panic;
    #[cfg(feature = "std")]
    pub use std::panic;
}

pub mod utils;

#[cfg(feature = "defmt")]
pub mod fmt;
