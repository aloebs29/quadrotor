#![no_main]
#![no_std]

use core::mem;

use embassy_executor::Spawner;
use embassy_time::Timer;

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;

use nrf_softdevice as _;

use defmt::info;

use r4_firmware as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("r4 quadcopter flight controller");

    let p = embassy_nrf::init(Default::default());
    let clock: pac::CLOCK = unsafe { mem::transmute(()) };

    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    let mut led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}
