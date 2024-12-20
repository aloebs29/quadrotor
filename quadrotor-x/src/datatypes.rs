use core::ops;

use micromath::{vector::F32x3, Quaternion};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, MaxSize, Copy, Clone, Debug, PartialEq)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3f {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn zeroed() -> Self {
        Self::new(0., 0., 0.)
    }
}

impl ops::Add for Vec3f {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl ops::Sub for Vec3f {
    type Output = Self;

    fn sub(self, rhs: Vec3f) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl ops::Div<f32> for Vec3f {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl From<F32x3> for Vec3f {
    fn from(value: F32x3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Vec3f> for F32x3 {
    fn from(value: Vec3f) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[derive(Serialize, Deserialize, MaxSize, Copy, Clone, Debug, PartialEq)]
pub struct Quatf {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quatf {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }
}

impl Default for Quatf {
    fn default() -> Self {
        Self {
            w: 1.,
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}

impl From<Quaternion> for Quatf {
    fn from(value: Quaternion) -> Self {
        Self::new(value.w(), value.x(), value.y(), value.z())
    }
}

impl From<Quatf> for Quaternion {
    fn from(value: Quatf) -> Self {
        Self::new(value.w, value.x, value.y, value.z)
    }
}

#[derive(Serialize, Deserialize, MaxSize, Copy, Clone, Debug, PartialEq)]
pub struct Telemetry {
    // Metadata
    pub timestamp: f32,
    pub error_count: u32,

    // Sensor readings
    pub battery_voltage: f32,
    pub accel: Vec3f,
    pub gyro: Vec3f,
    pub mag: Vec3f,
    pub pressure: f32,

    pub orientation: Quatf,
}

#[derive(Serialize, Deserialize, MaxSize, Copy, Clone, Debug, PartialEq)]
pub enum BleCommand {
    CalibrateAccel(f32),
    ActivateController(bool),
}
