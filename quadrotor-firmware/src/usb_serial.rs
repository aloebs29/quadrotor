use embassy_futures::select::{select, Either};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pipe;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};

use defmt::{error, info};
use embedded_cli::cli::{Cli, CliBuilder, CliHandle};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::datatypes::UsbCommand;

const USB_MAX_PACKET_SIZE: u8 = 8;
const MAX_COMMAND_LEN: usize = 128;
const MAX_RESPONSE_LEN: usize = 1024; // help text may be long
const HISTORY_LEN: usize = 1024;

pub type UsbDriver = Driver<'static, &'static SoftwareVbusDetect>;

type UsbResult<T> = Result<T, EndpointError>;

#[derive(Debug)]
pub struct SerialPipeWriteError(pipe::TryWriteError);

impl embedded_io::Error for SerialPipeWriteError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::OutOfMemory
    }
}

impl From<pipe::TryWriteError> for SerialPipeWriteError {
    fn from(value: pipe::TryWriteError) -> Self {
        Self(value)
    }
}

pub struct SerialPipeWriter {
    writer: pipe::Writer<'static, NoopRawMutex, MAX_RESPONSE_LEN>,
}

impl embedded_io::ErrorType for SerialPipeWriter {
    type Error = SerialPipeWriteError;
}

impl embedded_io::Write for SerialPipeWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(self.writer.try_write(buf)?)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // TODO: How to implement this without .await?
        Ok(())
    }
}

pub struct UsbCli {
    reader: pipe::Reader<'static, NoopRawMutex, MAX_COMMAND_LEN>,
    inner_cli: Cli<
        &'static mut SerialPipeWriter,
        SerialPipeWriteError,
        &'static mut [u8],
        &'static mut [u8],
    >,
}

impl UsbCli {
    pub fn process_pending_commands(
        &mut self,
        mut handler: impl FnMut(
            &mut CliHandle<&mut SerialPipeWriter, SerialPipeWriteError>,
            UsbCommand,
        ) -> Result<(), SerialPipeWriteError>,
    ) -> Result<(), pipe::TryReadError> {
        // Read until the pipe is empty, calling handler on each command received.
        loop {
            let mut received_byte = [0u8; 1];
            let _ = self.reader.try_read(received_byte.as_mut_slice())?;

            let _ = self.inner_cli.process_byte::<UsbCommand, _>(
                received_byte[0],
                &mut UsbCommand::processor(|cli, command| handler(cli, command)),
            );
        }
    }
}

pub struct SerialContext {
    class: CdcAcmClass<'static, UsbDriver>,
    writer_to_cli: pipe::Writer<'static, NoopRawMutex, MAX_COMMAND_LEN>,
    reader_from_cli: pipe::Reader<'static, NoopRawMutex, MAX_RESPONSE_LEN>,
}

async fn serial_session_handler(context: &mut SerialContext) -> UsbResult<()> {
    let mut read_chunk = [0; USB_MAX_PACKET_SIZE as usize];
    let mut write_chunk = [0; USB_MAX_PACKET_SIZE as usize];
    loop {
        // Service ingress/egress simultaneously
        let read_from_usb_future = context.class.read_packet(&mut read_chunk);
        let read_from_cli_future = context.reader_from_cli.read(&mut write_chunk);

        match select(read_from_usb_future, read_from_cli_future).await {
            Either::First(read_from_usb_result) => {
                let bytes_read_from_usb = read_from_usb_result?;
                // NOTE: Cannot await here. Waiting here would pend until the CLI services more
                //       bytes from the USB -> CLI pipe, however, the thread handling CLI commands
                //       may be awaiting a write to the CLI -> USB pipe (which is the other future
                //       we're selecting against here, and is blocked until we're done).
                // TODO: Separate tasks for USB write/read?
                if let Err(_) = context
                    .writer_to_cli
                    .try_write(&read_chunk[..bytes_read_from_usb])
                {
                    error!("Failed to write bytes from USB read buffer to CLI ingress pipe.");
                }
            }
            Either::Second(bytes_read_from_cli) => {
                context
                    .class
                    .write_packet(&write_chunk[..bytes_read_from_cli])
                    .await?;
            }
        }
    }
}

pub fn init(
    driver: UsbDriver,
) -> (
    UsbDevice<'static, UsbDriver>,
    &'static mut SerialContext,
    &'static mut UsbCli,
) {
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
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut CONFIG_DESC.init_with(|| [0; 256])[..],
        &mut BOS_DESC.init_with(|| [0; 256])[..],
        &mut MSOS_DESC.init_with(|| [0; 256])[..],
        &mut CONTROL_BUF.init_with(|| [0; 64])[..],
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, state, USB_MAX_PACKET_SIZE as u16);

    // Build the builder.
    let usb = builder.build();

    // Create pipes for communicating from CLI to USB serial thread
    static USB_TO_CLI_PIPE: StaticCell<pipe::Pipe<NoopRawMutex, MAX_COMMAND_LEN>> =
        StaticCell::new();
    let usb_to_cli_pipe = USB_TO_CLI_PIPE.init_with(|| pipe::Pipe::new());
    let (usb_to_cli_reader, usb_to_cli_writer) = usb_to_cli_pipe.split();

    static CLI_TO_USB_PIPE: StaticCell<pipe::Pipe<NoopRawMutex, MAX_RESPONSE_LEN>> =
        StaticCell::new();
    let cli_to_usb_pipe = CLI_TO_USB_PIPE.init_with(|| pipe::Pipe::new());
    let (cli_to_usb_reader, cli_to_usb_writer) = cli_to_usb_pipe.split();

    // Create the serial context
    static SERIAL_CONTEXT: StaticCell<SerialContext> = StaticCell::new();
    let serial_context = SERIAL_CONTEXT.init_with(|| SerialContext {
        class,
        writer_to_cli: usb_to_cli_writer,
        reader_from_cli: cli_to_usb_reader,
    });

    // Create the CLI
    static COMMAND_BUFFER: StaticCell<[u8; MAX_COMMAND_LEN]> = StaticCell::new();
    static HISTORY_BUFFER: StaticCell<[u8; HISTORY_LEN]> = StaticCell::new();

    static SERIAL_PIPE_WRITER: StaticCell<SerialPipeWriter> = StaticCell::new();
    static USB_CLI: StaticCell<UsbCli> = StaticCell::new();
    let usb_cli = USB_CLI.init_with(|| UsbCli {
        inner_cli: CliBuilder::default()
            .writer(SERIAL_PIPE_WRITER.init_with(|| SerialPipeWriter {
                writer: cli_to_usb_writer,
            }))
            .command_buffer(
                COMMAND_BUFFER
                    .init_with(|| [0; MAX_COMMAND_LEN])
                    .as_mut_slice(),
            )
            .history_buffer(HISTORY_BUFFER.init_with(|| [0; HISTORY_LEN]).as_mut_slice())
            .build()
            .unwrap(),
        reader: usb_to_cli_reader,
    });

    (usb, serial_context, usb_cli)
}

#[embassy_executor::task]
pub async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
pub async fn serial_task(context: &'static mut SerialContext) -> ! {
    loop {
        context.class.wait_connection().await;
        info!("Connected to USB host");
        match serial_session_handler(context).await {
            Ok(_) | Err(EndpointError::Disabled) => info!("Disconnected from USB host"),
            Err(err) => error!("Unexpected USB error {:?}", err),
        };
    }
}
