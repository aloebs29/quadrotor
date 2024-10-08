#![no_main]
#![no_std]

use core::mem;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Instant, Timer};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::saadc;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::{bind_interrupts, interrupt, peripherals};

use nrf_softdevice::{SocEvent, Softdevice};

use defmt::{info, unwrap};
use static_cell::StaticCell;
use ufmt::uwrite;

use quadrotor_firmware::ble_server;
use quadrotor_firmware::datatypes::{
    BleCommandChannel, BleCommandSender, TelemetrySignal, UsbCommand,
};
use quadrotor_firmware::dps310;
use quadrotor_firmware::fxas21002;
use quadrotor_firmware::fxos8700;
use quadrotor_firmware::usb_serial;

use quadrotor_x::accel::{AccelOffsetState, AccelOffsetsBuilder};
use quadrotor_x::datatypes::{BleCommand, Quatf, Telemetry, Vec3f};
use quadrotor_x::sensor_fusion;

const INITIAL_PRESSURE_TIMEOUT_MS: u64 = 2000;
const MAIN_LOOP_INTERVAL_MS: u64 = 10;
const VBAT_DIVIDER: f32 = 568.75;

const MS_TO_SEC: f32 = 0.001;

bind_interrupts!(struct I2cIrqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

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
    static TELEMETRY_SIGNAL: StaticCell<TelemetrySignal> = StaticCell::new();
    let telemetry_signal = TELEMETRY_SIGNAL.init(TelemetrySignal::new());
    static BLE_COMMAND_CHANNEL: StaticCell<BleCommandChannel> = StaticCell::new();
    let ble_command_channel = BLE_COMMAND_CHANNEL.init(BleCommandChannel::new());
    static BLE_COMMAND_SENDER: StaticCell<BleCommandSender> = StaticCell::new();
    let ble_command_sender = BLE_COMMAND_SENDER.init(ble_command_channel.sender());
    let ble_command_receiver = ble_command_channel.receiver();

    // Initialize USB
    let (usb_driver, serial_context, usb_cli) = usb_serial::init(p.USBD, vbus_detect);

    // Initialize BLE peripheral server
    let server = unwrap!(ble_server::Server::new(
        sd,
        telemetry_signal,
        ble_command_sender
    ));

    // Initialize ADC
    let adc_config = saadc::Config::default();
    let adc_channel_config = saadc::ChannelConfig::single_ended(p.P0_29);
    let mut adc = saadc::Saadc::new(p.SAADC, SaadcIrqs, adc_config, [adc_channel_config]);

    // Initialize I2C
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K400;
    let mut twim = Twim::new(p.TWISPI0, I2cIrqs, p.P0_12, p.P0_11, i2c_config);

    // Initialize sensors
    let mut accel_and_mag_sensor = unsafe { fxos8700::FXOS8700_HANDLE.take() };
    unwrap!(accel_and_mag_sensor.configure(&mut twim).await);
    let mut gyro_sensor = unsafe { fxas21002::FXAS21002_HANDLE.take() };
    unwrap!(gyro_sensor.configure(&mut twim).await);
    let mut pressure_sensor = unsafe { dps310::DPS310_HANDLE.take() };
    unwrap!(pressure_sensor.configure(&mut twim).await);

    // Start tasks
    unwrap!(spawner.spawn(softdevice_task(sd, vbus_detect)));
    unwrap!(spawner.spawn(usb_serial::usb_task(usb_driver)));
    unwrap!(spawner.spawn(usb_serial::serial_task(serial_context)));
    unwrap!(spawner.spawn(ble_server::ble_task(sd, server)));

    // Set up main control loop
    let mut led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    let mut next_iter_start = Instant::now();
    let mut timestamp = next_iter_start.as_millis() as f32 * MS_TO_SEC;
    let mut error_count = 0u32;

    let mut accel_msmt = Vec3f::zeroed();
    let mut mag_msmt = Vec3f::zeroed();
    let mut gyro_msmt = Vec3f::zeroed();
    let mut adc_msmt_buf = [0i16; 1];

    let mut orientation = Quatf::default();

    // NOTE: We don't want to start running the control loop with some default pressure measurement, because then our
    // initial altitude will be way off. Wait until the pressure sensor returns a valid reading.
    let initial_pressure_timeout =
        Instant::now() + Duration::from_millis(INITIAL_PRESSURE_TIMEOUT_MS);
    let mut pressure_msmt = loop {
        if let Ok(Some(p)) = pressure_sensor.read(&mut twim).await {
            break p;
        }
        if Instant::now() > initial_pressure_timeout {
            panic!("Timed out while waiting for initial pressure sensor reading.");
        }
    };

    let mut accel_offset_state: Option<AccelOffsetState> = None;

    loop {
        next_iter_start += Duration::from_millis(MAIN_LOOP_INTERVAL_MS);
        Timer::at(next_iter_start).await;

        // ==============
        // Handle commands
        while let Ok(command) = ble_command_receiver.try_receive() {
            match command {
                BleCommand::CalibrateAccel(duration_sec) => {
                    let builder = AccelOffsetsBuilder::new(
                        (duration_sec * (1000f32 / MAIN_LOOP_INTERVAL_MS as f32)) as usize,
                    );
                    accel_offset_state = Some(AccelOffsetState::InProgress(builder));
                }
            }
        }
        let _ = usb_cli.process_pending_commands(|cli_handle, command| match command {
            UsbCommand::BleAddress => {
                let address = nrf_softdevice::ble::get_address(sd).bytes();
                let _ = uwrite!(
                    cli_handle.writer(),
                    "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    address[5],
                    address[4],
                    address[3],
                    address[2],
                    address[1],
                    address[0]
                );
                Ok(())
            }
        });

        // ==============
        // Perform measurements
        led.set_high();

        let new_timestamp = embassy_time::Instant::now().as_millis() as f32 * MS_TO_SEC;
        let delta_t = new_timestamp - timestamp;
        timestamp = new_timestamp;

        // NOTE: I2C transactions need to happen in a serial fashion, since they all use the same I2C bus. However, the
        //       ADC read can happen in parallel with one of those I2C transactions.
        let adc_fut = adc.sample(&mut adc_msmt_buf);
        let accel_and_mag_fut =
            accel_and_mag_sensor.read(&mut twim, &mut accel_msmt, &mut mag_msmt);
        let (_, accel_and_mag_result) = join(adc_fut, accel_and_mag_fut).await;
        if let Err(_) = accel_and_mag_result {
            error_count += 1;
        }
        let battery_voltage = adc_msmt_buf[0] as f32 / VBAT_DIVIDER;

        if let Err(_) = gyro_sensor.read(&mut twim, &mut gyro_msmt).await {
            error_count += 1;
        };

        match pressure_sensor.read(&mut twim).await {
            Ok(Some(p)) => pressure_msmt = p,
            Ok(None) => (), // new measurement wasn't ready
            Err(_) => error_count += 1,
        }

        led.set_low();
        // ==============

        // Update and/or apply corrections
        if let Some(ref inner_offset_state) = accel_offset_state {
            match inner_offset_state {
                AccelOffsetState::InProgress(builder) => {
                    let new_inner_offset_state = builder.update(accel_msmt);
                    accel_offset_state = Some(new_inner_offset_state);
                }
                AccelOffsetState::Ready(offsets) => {
                    accel_msmt = offsets.apply(accel_msmt);
                }
            }
        }

        // Compute orientation
        orientation =
            sensor_fusion::madgwick_fusion_9(orientation, accel_msmt, gyro_msmt, mag_msmt, delta_t);
        telemetry_signal.signal(Telemetry {
            timestamp,
            error_count,
            battery_voltage,
            accel: accel_msmt.into(),
            gyro: gyro_msmt.into(),
            mag: mag_msmt.into(),
            pressure: pressure_msmt,
            orientation: orientation.into(),
        });
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
