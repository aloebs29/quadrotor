use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;

use quadrotor_x::datatypes::{BleCommand, Telemetry};

const BLE_COMMAND_CHANNEL_LEN: usize = 8;

#[derive(embedded_cli::Command)]
pub enum UsbCommand {
    /// Retrieve the device's BLE address
    BleAddress,
    /// Set a PWM output
    PwmSet {
        /// ID of the PWM output (0-3)
        id: u8,
        /// Normalized duty cycle (0.0 -> 1.0)
        duty: f32,
    },
}

pub type TelemetrySignal = Signal<NoopRawMutex, Telemetry>;

pub type BleCommandChannel = Channel<NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;
pub type BleCommandSender<'a> = Sender<'a, NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;
pub type BleCommandReceiver<'a> = Receiver<'a, NoopRawMutex, BleCommand, BLE_COMMAND_CHANNEL_LEN>;

#[derive(Copy, Clone)]
pub enum ControllerState {
    Inactive,
    Active,
}
pub type ControllerStateSignal = Signal<NoopRawMutex, ControllerState>;