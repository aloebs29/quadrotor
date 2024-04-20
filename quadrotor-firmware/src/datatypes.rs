use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;

use bytemuck::{Pod, Zeroable};
use micromath::vector::F32x3;

// TODO: Is this needed? Need a packed version of micromath's F32x3 for serialization.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct FlatVec3F32 {
    x: f32,
    y: f32,
    z: f32,
}

impl From<F32x3> for FlatVec3F32 {
    fn from(value: F32x3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct Telemetry {
    pub battery_voltage: f32,
    pub accel: FlatVec3F32,
    pub gyro: FlatVec3F32,
    pub mag: FlatVec3F32,
    pub timestamp: u64,
}

pub type TelemetrySignal = Signal<NoopRawMutex, Telemetry>;
