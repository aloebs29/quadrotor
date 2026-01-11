#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Instant, Timer};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::saadc;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::{bind_interrupts, interrupt, peripherals};
use embassy_nrf::interrupt::InterruptExt;

use nrf_softdevice::{raw, RawError, SocEvent, Softdevice};

use defmt::{info, unwrap};
use static_cell::StaticCell;
use ufmt::uwrite;

use quadrotor_firmware::battery;
use quadrotor_firmware::ble_server;
use quadrotor_firmware::controller;
use quadrotor_firmware::datatypes::{BleCommandChannel, BleCommandSender, ControllerState, ControllerStateSignal, TelemetrySignal, UsbCommand};
use quadrotor_firmware::dps310;
use quadrotor_firmware::fxas21002;
use quadrotor_firmware::fxos8700;
use quadrotor_firmware::motor;
use quadrotor_firmware::persistent_data;
use quadrotor_firmware::status_led;
use quadrotor_firmware::usb_serial;
use quadrotor_x::accel::{AccelOffsetState, AccelOffsetsBuilder};
use quadrotor_x::datatypes::{BleCommand, ControllerSetpoints, Telemetry, Vec3f};

const INITIAL_PRESSURE_TIMEOUT_MS: u64 = 2000;
const MAIN_LOOP_INTERVAL_MS: u64 = 10;
const MS_TO_SEC: f32 = 0.001;

bind_interrupts!(struct I2cIrqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

bind_interrupts!(struct SaadcIrqs {
    SAADC => saadc::InterruptHandler;
});

bind_interrupts!(struct UsbdIrqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Quadcopter flight controller");

    // Interrupt priority levels 0, 1, and 4 are reserved for the SoftDevice. Make sure to not use them.
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;

    let p = embassy_nrf::init(config);

    // NOTE: The softdevice uses the POWER_CLOCK interrupt internally, which would otherwise be used for a hardware
    //       VBUS detect in the USB driver. To work around this, a software VBUS detect is used based on SoC events
    //       triggered by the soft device (see `softdevice_task`).
    static VBUS_DETECT_CELL: StaticCell<SoftwareVbusDetect> = StaticCell::new();
    let vbus_detect = VBUS_DETECT_CELL.init(SoftwareVbusDetect::new(true, true));

    // Initialize soft device with settings from BLE module
    let sd = Softdevice::enable(&ble_server::get_softdevice_config());
    unwrap!(RawError::convert(unsafe { raw::sd_clock_hfclk_request() }));

    // Setup shared data between tasks
    static TELEMETRY_SIGNAL: StaticCell<TelemetrySignal> = StaticCell::new();
    let telemetry_signal = TELEMETRY_SIGNAL.init(TelemetrySignal::new());
    static BLE_COMMAND_CHANNEL: StaticCell<BleCommandChannel> = StaticCell::new();
    let ble_command_channel = BLE_COMMAND_CHANNEL.init(BleCommandChannel::new());
    static BLE_COMMAND_SENDER: StaticCell<BleCommandSender> = StaticCell::new();
    let ble_command_sender = BLE_COMMAND_SENDER.init(ble_command_channel.sender());
    let ble_command_receiver = ble_command_channel.receiver();
    let mut controller_state = ControllerState::Inactive;
    static CONTROLLER_STATE_SIGNAL: StaticCell<ControllerStateSignal> = StaticCell::new();
    let controller_state_signal = CONTROLLER_STATE_SIGNAL.init(ControllerStateSignal::new());
    controller_state_signal.signal(controller_state);

    // Initialize LED
    let led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    // Initialize USB
    interrupt::USBD.set_priority(interrupt::Priority::P3);
    let usb_driver = usb::Driver::new(p.USBD, UsbdIrqs, &*vbus_detect);
    let (usb_device, serial_context, usb_cli) = usb_serial::init(usb_driver);

    // Initialize BLE peripheral server
    let server = unwrap!(ble_server::Server::new(
        sd,
        telemetry_signal,
        ble_command_sender
    ));

    // Initialize ADC
    let battery_adc_config = saadc::Config::default();
    let battery_adc_channel_config = saadc::ChannelConfig::single_ended(p.P0_29);
    interrupt::SAADC.set_priority(interrupt::Priority::P2);
    let battery_adc = saadc::Saadc::new(p.SAADC, SaadcIrqs, battery_adc_config, [battery_adc_channel_config]);

    // Initialize I2C
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K400;
    interrupt::TWISPI0.set_priority(interrupt::Priority::P2);
    static RAM_BUFFER: StaticCell<[u8; 2]> = StaticCell::new();
    let mut twim = Twim::new(p.TWISPI0, I2cIrqs, p.P0_12, p.P0_11, i2c_config, RAM_BUFFER.init([0; 2]));

    // Initialize sensors
    let mut accel_and_mag_sensor = unsafe { fxos8700::FXOS8700_HANDLE.take() };
    unwrap!(accel_and_mag_sensor.configure(&mut twim).await);
    let mut gyro_sensor = unsafe { fxas21002::FXAS21002_HANDLE.take() };
    unwrap!(gyro_sensor.configure(&mut twim).await);
    let mut pressure_sensor = unsafe { dps310::DPS310_HANDLE.take() };
    unwrap!(pressure_sensor.configure(&mut twim).await);
    let mut battery_reader = battery::BatteryReader::new(battery_adc);

    // Initialize motor outputs
    let mut motor_outputs = motor::MotorOutputs::new(
        p.PWM0,
        p.P0_07.into(),     // NRF52840 feather D6
        p.P0_26.into(),     // NRF52840 feather D9
        p.P0_27.into(),     // NRF52840 feather D10
        p.P0_06.into());    // NRF52840 feather D11

    // Initialize persistent data service
    let mut persistent_data_service = persistent_data::PersistentDataService::new(&sd).await;

    // Start tasks
    unwrap!(spawner.spawn(softdevice_task(sd, vbus_detect)));
    unwrap!(spawner.spawn(status_led::status_led_task(led, controller_state_signal)));
    unwrap!(spawner.spawn(usb_serial::usb_task(usb_device)));
    unwrap!(spawner.spawn(usb_serial::serial_task(serial_context)));
    unwrap!(spawner.spawn(ble_server::ble_task(sd, server)));

    // NOTE: We don't want to start running the control loop with some default pressure measurement,
    // because then our initial altitude will be way off. Wait until the pressure sensor returns a
    // valid reading.
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

    // Set up main control loop
    let mut next_iter_start = Instant::now();
    let mut timestamp = next_iter_start.as_millis() as f32 * MS_TO_SEC;
    let mut error_count = 0u32;

    let mut accel_msmt = Vec3f::zeroed();
    let mut mag_msmt = Vec3f::zeroed();
    let mut gyro_msmt = Vec3f::zeroed();

    let mut accel_offset_state = AccelOffsetState::from(persistent_data_service.get_contents().accel_offsets);
    let mut controller = controller::Controller::new(
        persistent_data_service.get_contents().controller_params,
        ControllerSetpoints::default(),
        timestamp,
    );

    loop {
        next_iter_start += Duration::from_millis(MAIN_LOOP_INTERVAL_MS);
        Timer::at(next_iter_start).await;

        // ==============
        // Handle BLE commands
        while let Ok(command) = ble_command_receiver.try_receive() {
            match command {
                BleCommand::CalibrateAccel(duration_sec) => {
                    let builder = AccelOffsetsBuilder::new(
                        (duration_sec * (1000f32 / MAIN_LOOP_INTERVAL_MS as f32)) as usize,
                    );
                    accel_offset_state = AccelOffsetState::InProgress(builder);
                },
                BleCommand::ActivateController(activate) => {
                    controller_state = if activate {
                        ControllerState::Active
                    } else {
                        ControllerState::Inactive
                    };
                    controller_state_signal.signal(controller_state);
                },
                BleCommand::UpdateControllerParams(new_controller_params) => {
                    persistent_data_service.update_contents(
                        |persistent_data| persistent_data.controller_params = new_controller_params
                    ).await;
                    controller.params = new_controller_params;
                },
                BleCommand::UpdateControllerSetpoints(new_controller_setpoints) => {
                    controller.setpoints = new_controller_setpoints;
                },
            }
        }

        // ==============
        // Handle USB CLI commands
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
                )?;
                Ok(())
            },
            UsbCommand::PwmSet { id, duty } => {
                if let ControllerState::Active = controller_state {
                    let _ = uwrite!(cli_handle.writer(), "Controller is activated; manual PWM entry disallowed.");
                } else if id > 3 {
                    let _ = uwrite!(cli_handle.writer(), "Invalid PWM ID {}.", id)?;
                } else if duty < 0.0 || duty > 1.0 {
                    let _ = uwrite!(cli_handle.writer(), "Invalid duty cycle.")?;
                } else {
                    let _ = uwrite!(cli_handle.writer(), "Setting output to {}%.", (duty * 100.0) as u32)?;
                    motor_outputs.set_single(id.into(), duty);
                }
                Ok(())
            },
        });

        // ==============
        // Perform measurements
        timestamp = Instant::now().as_millis() as f32 * MS_TO_SEC;

        // NOTE: I2C transactions need to happen in a serial fashion, since they all use the same I2C bus. However, the
        //       ADC read can happen in parallel with one of those I2C transactions.
        let battery_fut = battery_reader.read();
        let accel_and_mag_fut =
            accel_and_mag_sensor.read(&mut twim, &mut accel_msmt, &mut mag_msmt);
        let (battery_voltage, accel_and_mag_result) = join(battery_fut, accel_and_mag_fut).await;
        if let Err(_) = accel_and_mag_result {
            error_count += 1;
        }

        if let Err(_) = gyro_sensor.read(&mut twim, &mut gyro_msmt).await {
            error_count += 1;
        };

        match pressure_sensor.read(&mut twim).await {
            Ok(Some(p)) => pressure_msmt = p,
            Ok(None) => (), // new measurement wasn't ready
            Err(_) => error_count += 1,
        }

        // ==============
        // Update and/or apply corrections
        if let AccelOffsetState::InProgress(builder) = accel_offset_state {
            accel_offset_state = builder.update(accel_msmt);
            // If the builder has finished, update our persistent data
            if let AccelOffsetState::Ready(offsets) = accel_offset_state {
                persistent_data_service.update_contents(
                    |persistent_data| persistent_data.accel_offsets = offsets
                ).await;
            }
        }
        if let AccelOffsetState::Ready(offsets) = accel_offset_state {
            accel_msmt = accel_msmt - offsets;
        }

        // ==============
        // Update controls
        let motor_setpoints = controller.update(timestamp, &accel_msmt, &gyro_msmt, &mag_msmt).await;
        if let ControllerState::Active = controller_state {
            motor_outputs.set_all(motor_setpoints);
        }

        telemetry_signal.signal(Telemetry {
            timestamp,
            error_count,
            controller_params: controller.params,
            battery_voltage,
            accel: accel_msmt.into(),
            gyro: gyro_msmt.into(),
            mag: mag_msmt.into(),
            pressure: pressure_msmt,
            orientation: controller.get_orientation(),
            motor_setpoints,
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
