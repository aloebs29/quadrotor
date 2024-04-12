#![no_main]
#![no_std]

use {defmt_rtt as _, panic_probe as _};

pub mod ble_server;
pub mod usb_serial;
