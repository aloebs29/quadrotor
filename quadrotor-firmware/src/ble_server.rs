use core::mem;

use embassy_futures::select::{select, Either};

use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::raw::{self, ble_gap_conn_params_t};
use nrf_softdevice::{ble, Softdevice};

use defmt::*;
use postcard::experimental::max_size::MaxSize;

use quadrotor_x::datatypes::{BleCommand, Telemetry};
use quadrotor_x::utils::lipo_1s_charge_percent_from_voltage;

use crate::datatypes::{BleCommandSender, TelemetrySignal};

const BT_DEVICE_NAME: &str = "Quadcopter";

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_service(uuid = "6dff68a3-f84a-4f54-a244-cc0b528425ea")]
struct TelemetryService {
    #[characteristic(uuid = "6dff68a3-f84a-4f54-a244-cc0b528425ea", read, notify)]
    telemetry: [u8; Telemetry::POSTCARD_MAX_SIZE],
}

#[nrf_softdevice::gatt_service(uuid = "51e426ca-502f-405b-89dc-1b299df7cf32")]
struct CommandService {
    #[characteristic(uuid = "51e426ca-502f-405b-89dc-1b299df7cf32", write)]
    command: [u8; BleCommand::POSTCARD_MAX_SIZE],
}

#[nrf_softdevice::gatt_server]
struct ServerInternal {
    battery_service: BatteryService,
    telemetry_service: TelemetryService,
    command_service: CommandService,
}

pub struct Server<'a> {
    internal: ServerInternal,
    telemetry_signal: &'a TelemetrySignal,
    command_sender: &'a BleCommandSender<'a>,
}

impl Server<'_> {
    pub fn new<'a>(
        sd: &mut Softdevice,
        telemetry_signal: &'static TelemetrySignal,
        command_sender: &'static BleCommandSender<'static>,
    ) -> Result<Self, ble::gatt_server::RegisterError> {
        let internal = ServerInternal::new(sd)?;

        Ok(Self {
            internal,
            telemetry_signal,
            command_sender,
        })
    }

    async fn update_input_values(&self, connection: &ble::Connection) {
        // NOTE: Notifying the BLE client on every telemetry update (currently 100 Hz) seems to cause occasional latency
        //       spikes.
        const UPDATES_PER_NOTIFY: u32 = 5;
        let mut iter_count = 0u32;
        loop {
            let telemetry = self.telemetry_signal.wait().await;

            if iter_count % UPDATES_PER_NOTIFY == 0 {
                // Update battery service
                let battery_percent =
                    lipo_1s_charge_percent_from_voltage(telemetry.battery_voltage) as u8;
                let _ = self
                    .internal
                    .battery_service
                    .battery_level_set(&battery_percent);

                // Notify telemetry service
                // NOTE: Unwrap here as any runtime error just represents a programming error.
                let mut telemetry_byte_array = [0u8; Telemetry::POSTCARD_MAX_SIZE];
                unwrap!(postcard::to_slice(&telemetry, &mut telemetry_byte_array));
                match self
                    .internal
                    .telemetry_service
                    .telemetry_notify(connection, &telemetry_byte_array)
                {
                    Ok(_) => (),
                    Err(_) => {
                        let _ = self
                            .internal
                            .telemetry_service
                            .telemetry_set(&telemetry_byte_array);
                    }
                };
            }
            iter_count += 1;
        }
    }
}

pub fn get_softdevice_config() -> nrf_softdevice::Config {
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: raw::BLE_GAP_CONN_COUNT_DEFAULT as u8,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: raw::BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT as u8,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: BT_DEVICE_NAME.as_bytes().as_ptr() as *const u8 as _, // NOTE: We're disabling writes to this, so its OK that it is const.
            current_len: BT_DEVICE_NAME.bytes().count() as u16,
            max_len: BT_DEVICE_NAME.bytes().count() as u16,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    return config;
}

#[embassy_executor::task]
pub async fn ble_task(sd: &'static Softdevice, server: Server<'static>) -> ! {
    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
        .full_name(BT_DEVICE_NAME)
        .build();
    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new().build();

    loop {
        let config = ble::peripheral::Config::default();
        let adv = ble::peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };
        let conn = match ble::peripheral::advertise_connectable(sd, adv, &config).await {
            Ok(c) => c,
            Err(err) => {
                error!("Advertisement error: {:?}.", err);
                continue;
            }
        };

        let _ = conn.set_conn_params(ble_gap_conn_params_t {
            min_conn_interval: 8,
            max_conn_interval: 8,
            slave_latency: 1,
            conn_sup_timeout: 0xFFFF,
        });

        info!("Connected to BLE client");

        let update_fut = server.update_input_values(&conn);
        let gatt_fut = ble::gatt_server::run(&conn, &server.internal, |e| match e {
            ServerInternalEvent::BatteryService(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    debug!("battery notifications: {}", notifications)
                }
            },
            ServerInternalEvent::TelemetryService(e) => match e {
                TelemetryServiceEvent::TelemetryCccdWrite { notifications } => {
                    debug!("telemetry notifications: {}", notifications)
                }
            },
            ServerInternalEvent::CommandService(e) => match e {
                CommandServiceEvent::CommandWrite(command_bytes) => {
                    if let Ok(command) = postcard::from_bytes::<BleCommand>(&command_bytes) {
                        if let Err(_) = server.command_sender.try_send(command) {
                            error!("Failed to push command onto queue.");
                        }
                    } else {
                        error!("Failed to parse command from bytes.");
                    }
                }
            },
        });

        match select(update_fut, gatt_fut).await {
            Either::First(_) => (),
            Either::Second(e) => {
                info!("Disconnected from BLE connection (with {:?})", e)
            }
        };
    }
}
