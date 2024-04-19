#![no_main]
#![no_std]

use core::mem;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::saadc;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::{bind_interrupts, interrupt};

use nrf_softdevice::{SocEvent, Softdevice};

use defmt::{info, unwrap};
use static_cell::StaticCell;

use quadrotor_firmware::ble_server;
use quadrotor_firmware::datatypes::{InputState, InputStateSignal};
use quadrotor_firmware::usb_serial;

const MAIN_LOOP_INTERVAL_MS: u64 = 10;
const VBAT_DIVIDER: f32 = 568.75;

bind_interrupts!(struct SaadcIrqs {
    SAADC => saadc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Quadcopter flight controller");

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

    // Setup shared data between tasks
    static INPUT_STATE_SIGNAL: StaticCell<InputStateSignal> = StaticCell::new();
    let input_state_signal = INPUT_STATE_SIGNAL.init(InputStateSignal::new());

    // Initialize USB
    let (usb_driver, cdc_class) = usb_serial::init(p.USBD, vbus_detect);
    // Initialize BLE peripheral server
    let server = unwrap!(ble_server::Server::new(sd, input_state_signal));

    // Initialize ADC
    let adc_config = saadc::Config::default();
    let adc_channel_config = saadc::ChannelConfig::single_ended(p.P0_29);
    let mut adc = saadc::Saadc::new(p.SAADC, SaadcIrqs, adc_config, [adc_channel_config]);

    // Start tasks
    unwrap!(spawner.spawn(softdevice_task(sd, vbus_detect)));
    unwrap!(spawner.spawn(usb_serial::usb_task(usb_driver)));
    unwrap!(spawner.spawn(usb_serial::serial_task(cdc_class)));
    unwrap!(spawner.spawn(ble_server::ble_task(sd, server)));

    // Set up main control loop
    let mut led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    let mut input_state = InputState::default();
    let mut next_iter_start = Instant::now();
    let mut adc_msmt_buf = [0i16; 1];

    loop {
        next_iter_start += Duration::from_millis(MAIN_LOOP_INTERVAL_MS);
        Timer::at(next_iter_start).await;

        // ==============
        // Perform measurements
        led.set_high();

        adc.sample(&mut adc_msmt_buf).await;
        input_state.battery_voltage = adc_msmt_buf[0] as f32 / VBAT_DIVIDER;

        input_state.timestamp = embassy_time::Instant::now().as_ticks();

        input_state_signal.signal(input_state.clone());
        led.set_low();
        // ==============
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
