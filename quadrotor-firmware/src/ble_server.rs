use core::mem;

use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::{ble, raw, Softdevice};

use defmt::*;

const BT_DEVICE_NAME: &str = "R4 Quadcopter";
static mut BT_ADDRESS: Option<ble::Address> = None;

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_server]
struct ServerInternal {
    battery_service: BatteryService,
}

// NOTE: This is basically a structure to avoid mismatched privacy rules that would result from making the derived gatt
//       server public.
pub struct Server {
    internal: ServerInternal,
}

impl Server {
    pub fn new(
        sd: &mut Softdevice,
    ) -> Result<Self, nrf_softdevice::ble::gatt_server::RegisterError> {
        let internal = ServerInternal::new(sd)?;
        let _ = internal.battery_service.battery_level_set(&77u8);

        Ok(Self { internal })
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
pub async fn ble_task(sd: &'static Softdevice, server: Server) -> ! {
    unsafe {
        BT_ADDRESS = Some(ble::get_address(sd));
    }

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

        info!("Connected to BLE client");

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        let e = ble::gatt_server::run(&conn, &server.internal, |e| match e {
            ServerInternalEvent::BatteryService(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    debug!("battery notifications: {}", notifications)
                }
            },
        })
        .await;

        info!("Disconnected from BLE connection (with {:?})", e);
    }
}

pub fn get_address() -> Option<ble::Address> {
    // NOTE: The nRF BLE API's get_address command requires a reference to the soft device. This module basically just
    // reads the address from a context where there is a reference to the soft device (i.e. at the start of the server
    // task) and stores it for later.
    unsafe { BT_ADDRESS.clone() }
}
