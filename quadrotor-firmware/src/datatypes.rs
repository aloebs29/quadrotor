use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;

use bytemuck::{Pod, Zeroable};
use micromath::{vector::F32x3, Quaternion};

// TODO: Is this needed? Need a packed version of micromath's F32x3 for serialization.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct FlatVec3F32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
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

// TODO: Is this needed? Need a packed version of micromath's Quaternion for serialization.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct FlatQuaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<Quaternion> for FlatQuaternion {
    fn from(value: Quaternion) -> Self {
        Self {
            w: value.w(),
            x: value.x(),
            y: value.y(),
            z: value.z(),
        }
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct Telemetry {
    // Metadata
    pub timestamp: f32,
    pub error_count: u32,

    // Sensor readings
    pub battery_voltage: f32,
    pub accel: FlatVec3F32,
    pub gyro: FlatVec3F32,
    pub mag: FlatVec3F32,
    pub pressure: f32,

    pub orientation: FlatQuaternion,
    pub velocity: FlatVec3F32,
}

pub type TelemetrySignal = Signal<NoopRawMutex, Telemetry>;
