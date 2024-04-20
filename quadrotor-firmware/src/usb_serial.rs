use core::fmt::Write as _;

use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, interrupt, peripherals, usb};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};

use defmt::{error, info};
use heapless::{String, Vec};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::ble_server;

const USB_MAX_PACKET_SIZE: u8 = 8;
const MAX_COMMAND_LEN: usize = 128;
const MAX_RESPONSE_LEN: usize = 128;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
});

pub type UsbDriver = Driver<'static, peripherals::USBD, &'static SoftwareVbusDetect>;

type UsbResult<T> = Result<T, EndpointError>;

struct SerialContext {
    class: CdcAcmClass<'static, UsbDriver>,
    read_buffer: Vec<u8, MAX_COMMAND_LEN>,
    write_buffer: String<MAX_RESPONSE_LEN>,
}

async fn write_line_to_usb(
    class: &mut CdcAcmClass<'static, UsbDriver>,
    data: &[u8],
) -> UsbResult<()> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
        class.write_packet(chunk).await?;
    }
    class.write_packet(&[b'\n']).await?;
    Ok(())
}

async fn command_handler(context: &mut SerialContext) -> UsbResult<()> {
    if context.read_buffer.as_slice() == b"mac_address" {
        if let Some(address) = ble_server::get_address() {
            let address = address.bytes();
            write!(
                context.write_buffer,
                "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                address[5], address[4], address[3], address[2], address[1], address[0]
            )
            .unwrap();
            let result =
                write_line_to_usb(&mut context.class, context.write_buffer.as_bytes()).await;
            context.write_buffer.clear();
            if result.is_err() {
                return result;
            }
        } else {
            write_line_to_usb(&mut context.class, b"Unknown").await?;
        }
    }
    Ok(())
}

async fn serial_parser(context: &mut SerialContext) -> UsbResult<()> {
    let mut read_chunk = [0; USB_MAX_PACKET_SIZE as usize];
    loop {
        let n = context.class.read_packet(&mut read_chunk).await?;
        for &byte in &read_chunk[..n] {
            // If newline was received, process the command
            if byte == b'\r' || byte == b'\n' {
                command_handler(context).await?;
                context.read_buffer.clear();
            } else {
                let _ = context.read_buffer.push(byte);
            }
        }
    }
}

pub fn init(
    usbd: embassy_nrf::peripherals::USBD,
    vbus_detect: &'static SoftwareVbusDetect,
) -> (
    UsbDevice<'static, UsbDriver>,
    CdcAcmClass<'static, UsbDriver>,
) {
    let driver = Driver::new(usbd, Irqs, vbus_detect);

    // Create embassy-usb Config
    let mut config = Config::new(0xfeed, 0xface);
    config.manufacturer = Some("your");
    config.product = Some("mother");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = USB_MAX_PACKET_SIZE;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    static STATE: StaticCell<State> = StaticCell::new();
    let state = STATE.init(State::new());

    // Create embassy-usb DeviceBuilder using the driver and config.
    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut DEVICE_DESC.init_with(|| [0; 256])[..],
        &mut CONFIG_DESC.init_with(|| [0; 256])[..],
        &mut BOS_DESC.init_with(|| [0; 256])[..],
        &mut MSOS_DESC.init_with(|| [0; 128])[..],
        &mut CONTROL_BUF.init_with(|| [0; 128])[..],
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, state, USB_MAX_PACKET_SIZE as u16);

    // Build the builder.
    let usb = builder.build();
    interrupt::USBD.set_priority(interrupt::Priority::P3);

    return (usb, class);
}

#[embassy_executor::task]
pub async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
pub async fn serial_task(class: CdcAcmClass<'static, UsbDriver>) -> ! {
    static CONTEXT_CELL: StaticCell<SerialContext> = StaticCell::new();
    let context = CONTEXT_CELL.init_with(|| SerialContext {
        class,
        write_buffer: String::new(),
        read_buffer: Vec::new(),
    });

    loop {
        context.class.wait_connection().await;
        info!("Connected to USB host");
        match serial_parser(context).await {
            Ok(_) | Err(EndpointError::Disabled) => info!("Disconnected from USB host"),
            Err(err) => error!("Unexpected USB error {:?}", err),
        };
    }
}
