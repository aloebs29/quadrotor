use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;

use micromath::vector::F32x3;

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct InputState {
    pub battery_voltage: f32,
    pub accel: F32x3,
    pub gyro: F32x3,
    pub mag: F32x3,
    pub timestamp: u64,
}

pub type InputStateSignal = Signal<NoopRawMutex, InputState>;
