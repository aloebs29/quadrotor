use embassy_nrf::twim::{Instance, Twim};
use embassy_time::{Duration, Instant};

use defmt::*;
use embassy_futures::join::join;
use quadrotor_x::accel::{AccelOffsetState, AccelOffsetsBuilder};
use quadrotor_x::datatypes::{Quatf, Telemetry, Vec3f};
use quadrotor_x::sensor_fusion::madgwick_fusion_9;

use crate::battery::BatteryReader;
use crate::datatypes::TelemetrySignal;
use crate::dps310::Dps310;
use crate::fxas21002::Fxas21002;
use crate::fxos8700::Fxos8700;
use crate::xerror::{XError, XResult};

const INITIAL_PRESSURE_TIMEOUT_MS: u64 = 2000;

pub struct Controller {
    accel_and_mag_sensor: Fxos8700,
    gyro_sensor: Fxas21002,
    pressure_sensor: Dps310,
    battery_reader: BatteryReader,

    orientation: Quatf,
    pressure: f32,
    error_count: u32,
    timestamp: f32,

    accel_offset_state: Option<AccelOffsetState>,

    telemetry_signal: &'static TelemetrySignal,
}

impl Controller {
    pub async fn create_and_initialize<'a, T: Instance>(
        twim: &mut Twim<'a, T>,
        accel_and_mag_sensor: Fxos8700,
        gyro_sensor: Fxas21002,
        pressure_sensor: Dps310,
        battery_reader: BatteryReader,
        timestamp: f32,
        telemetry_signal: &'static TelemetrySignal,
    ) -> XResult<Self> {
        // NOTE: We don't want to start running the control loop with some default pressure
        // measurement, because then our initial altitude will be way off. Wait until the pressure
        // sensor returns a valid reading.
        let initial_pressure_timeout =
            Instant::now() + Duration::from_millis(INITIAL_PRESSURE_TIMEOUT_MS);
        let pressure = loop {
            if let Ok(Some(p)) = pressure_sensor.read(twim).await {
                break p;
            }
            if Instant::now() > initial_pressure_timeout {
                error!("Timed out while waiting for initial pressure sensor reading.");
                return Err(XError::Timeout);
            }
        };

        Ok(Self {
            accel_and_mag_sensor,
            gyro_sensor,
            pressure_sensor,
            battery_reader,

            orientation: Quatf::default(),
            pressure,
            error_count: 0,
            timestamp,

            accel_offset_state: None,

            telemetry_signal,
        })
    }

    pub async fn update<'a, T: Instance>(self: &mut Self, twim: &mut Twim<'a, T>, timestamp: f32) -> () {
        let mut accel_msmt = Vec3f::zeroed();
        let mut mag_msmt = Vec3f::zeroed();
        let mut gyro_msmt = Vec3f::zeroed();

        // ==============
        // Perform measurements
        let delta_t = timestamp - self.timestamp;
        self.timestamp = timestamp;

        // NOTE: I2C transactions need to happen in a serial fashion, since they all use the same
        // I2C bus. However, the ADC read can happen in parallel with one of those I2C transactions.
        let battery_fut = self.battery_reader.read();
        let accel_and_mag_fut =
            self.accel_and_mag_sensor.read(twim, &mut accel_msmt, &mut mag_msmt);
        let (battery_voltage, accel_and_mag_result) = join(battery_fut, accel_and_mag_fut).await;
        if let Err(_) = accel_and_mag_result {
            self.error_count += 1;
        }

        if let Err(_) = self.gyro_sensor.read(twim, &mut gyro_msmt).await {
            self.error_count += 1;
        };

        match self.pressure_sensor.read(twim).await {
            Ok(Some(p)) => self.pressure = p,
            Ok(None) => (), // new measurement wasn't ready
            Err(_) => self.error_count += 1,
        }
        // ==============

        // Update and/or apply corrections
        if let Some(ref inner_offset_state) = self.accel_offset_state {
            match inner_offset_state {
                AccelOffsetState::InProgress(builder) => {
                    let new_inner_offset_state = builder.update(accel_msmt);
                    self.accel_offset_state = Some(new_inner_offset_state);
                }
                AccelOffsetState::Ready(offsets) => {
                    accel_msmt = offsets.apply(accel_msmt);
                }
            }
        }

        // Compute orientation
        self.orientation = madgwick_fusion_9(self.orientation, accel_msmt, gyro_msmt, mag_msmt, delta_t);
        self.telemetry_signal.signal(Telemetry {
            timestamp,
            error_count: self.error_count,
            battery_voltage,
            accel: accel_msmt.into(),
            gyro: gyro_msmt.into(),
            mag: mag_msmt.into(),
            pressure: self.pressure,
            orientation: self.orientation,
        });

        // TODO: Calculate and return motor outputs
    }

    pub fn start_accel_calibration(self: &mut Self, target_sample_count: usize) {
        let builder = AccelOffsetsBuilder::new(target_sample_count);
        self.accel_offset_state = Some(AccelOffsetState::InProgress(builder));
    }
}
