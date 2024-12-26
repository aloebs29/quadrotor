#![no_main]
#![no_std]

use {defmt_rtt as _, panic_probe as _};

pub mod battery;
pub mod ble_server;
pub mod controller;
pub mod datatypes;
pub mod dps310;
pub mod fxas21002;
pub mod fxos8700;
pub mod motor;
pub mod persistent_data;
pub mod status_led;
pub mod usb_serial;
pub mod xerror;
