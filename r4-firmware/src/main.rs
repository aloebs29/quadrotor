#![no_main]
#![no_std]

use core::mem;

use embassy_executor::Spawner;
use embassy_time::Timer;

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt;
use embassy_nrf::pac;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;

use nrf_softdevice::{SocEvent, Softdevice};

use defmt::{info, unwrap};
use static_cell::StaticCell;

use r4_firmware::ble_server;
use r4_firmware::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("R4 Quadcopter flight controller");

    // Interrupt priority levels 0, 1, and 4 are reserved for the SoftDevice. Make sure to not use them.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;

    let p = embassy_nrf::init(config);
    let clock: pac::CLOCK = unsafe { mem::transmute(()) };

    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    // NOTE: The softdevice uses the POWER_CLOCK interrupt internally, which would otherwise be used for a hardware
    //       VBUS detect in the USB driver. To work around this, a software VBUS detect is used based on SoC events
    //       triggered by the soft device (see `softdevice_task`).
    static VBUS_DETECT_CELL: StaticCell<SoftwareVbusDetect> = StaticCell::new();
    let vbus_detect = VBUS_DETECT_CELL.init(SoftwareVbusDetect::new(true, true));

    // Initialize soft device with settings from BLE module
    let sd = Softdevice::enable(&ble_server::get_softdevice_config());

    // Initialize USB
    let (usb_driver, cdc_class) = usb_serial::init(p.USBD, vbus_detect);
    // Initialize BLE peripheral server
    let server = unwrap!(ble_server::Server::new(sd));

    // Start tasks
    unwrap!(spawner.spawn(softdevice_task(sd, vbus_detect)));
    unwrap!(spawner.spawn(usb_serial::usb_task(usb_driver)));
    unwrap!(spawner.spawn(usb_serial::serial_task(cdc_class)));
    unwrap!(spawner.spawn(ble_server::ble_task(sd, server)));

    let mut led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice, vbus_detect: &'static SoftwareVbusDetect) -> ! {
    unsafe {
        nrf_softdevice::raw::sd_power_usbdetected_enable(1);
        nrf_softdevice::raw::sd_power_usbpwrrdy_enable(1);
        nrf_softdevice::raw::sd_power_usbremoved_enable(1);
        nrf_softdevice::raw::sd_clock_hfclk_request();
    };
    sd.run_with_callback(|event: SocEvent| {
        match event {
            SocEvent::PowerUsbRemoved => vbus_detect.detected(false),
            SocEvent::PowerUsbDetected => vbus_detect.detected(true),
            SocEvent::PowerUsbPowerReady => vbus_detect.ready(),
            _ => {}
        };
    })
    .await
}
