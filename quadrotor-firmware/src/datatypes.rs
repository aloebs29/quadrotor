use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;

use quadrotor_x::datatypes::{Command, Telemetry};

const COMMAND_CHANNEL_LEN: usize = 8;

pub type TelemetrySignal = Signal<NoopRawMutex, Telemetry>;
pub type CommandChannel = Channel<NoopRawMutex, Command, COMMAND_CHANNEL_LEN>;
pub type CommandSender<'a> = Sender<'a, NoopRawMutex, Command, COMMAND_CHANNEL_LEN>;
pub type CommandReceiver<'a> = Receiver<'a, NoopRawMutex, Command, COMMAND_CHANNEL_LEN>;
