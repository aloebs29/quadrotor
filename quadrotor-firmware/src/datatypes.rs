use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;

use quadrotor_x::datatypes::{BleCommand, Telemetry};

const BLE_COMMAND_CHANNEL_LEN: usize = 8;

#[derive(embedded_cli::Command)]
pub enum UsbCommand {
    /// Retrieve the device's BLE address
    BleAddress,
}

pub type TelemetrySignal = Signal<NoopRawMutex, Telemetry>;

pub type BleCommandChannel = Channel<NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;
pub type BleCommandSender<'a> = Sender<'a, NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;
pub type BleCommandReceiver<'a> = Receiver<'a, NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;
