use core::mem::replace;

use bitfield_struct::bitfield;
use bytemuck::{Pod, Zeroable};
use defmt::error;
use embassy_nrf::twim::{Instance, Twim};
use embassy_time::{Duration, Instant};

use crate::xerror::*;
use quadrotor_x::utils::twos_complement;

const DEVICE_ADDRESS: u8 = 0x77;

const REG_ADDRESS_PRS_B2: u8 = 0x00;
const REG_ADDRESS_PRS_CFG: u8 = 0x06;
const REG_ADDRESS_TMP_CFG: u8 = 0x07;
const REG_ADDRESS_MEAS_CFG: u8 = 0x08;
const REG_ADDRESS_CFG: u8 = 0x09;
const REG_ADDRESS_PRODUCT_ID: u8 = 0x0D;
const REG_ADDRESS_COEF: u8 = 0x10;

const PRODUCT_ID_EXPECTED: u8 = 0x10;

const COEFF_RDY_TIMEOUT_MS: u64 = 1000;
const KP_KT: f32 = 253952f32;

#[bitfield(u8)]
struct RegPrsCfg {
    #[bits(4)]
    pm_prc: u8,
    #[bits(3)]
    pm_rate: u8,
    __: bool,
}

#[bitfield(u8)]
struct RegTmpCfg {
    #[bits(4)]
    tmp_prc: u8,
    #[bits(3)]
    tmp_rate: u8,
    __: bool,
}

#[bitfield(u8)]
struct RegMeasCfg {
    #[bits(3)]
    meas_ctrl: u8,
    __: bool,
    prs_rdy: bool,
    tmp_rdy: bool,
    sensor_rdy: bool,
    coef_rdy: bool,
}

#[bitfield(u8)]
struct RegCfg {
    spi_mode: bool,
    fifo_en: bool,
    p_shift: bool,
    t_shift: bool,
    int_prs: bool,
    int_tmp: bool,
    int_fifo: bool,
    int_hl: bool,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct CoefficientReadBuffer {
    c0: u8,
    c0c1: u8,
    c1: u8,
    c00_1: u8,
    c00_2: u8,
    c00c10: u8,
    c10_1: u8,
    c10_2: u8,
    c01_1: u8,
    c01_2: u8,
    c11_1: u8,
    c11_2: u8,
    c20_1: u8,
    c20_2: u8,
    c21_1: u8,
    c21_2: u8,
    c30_1: u8,
    c30_2: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct DataReadBuffer {
    prs_b2: u8,
    prs_b1: u8,
    prs_b0: u8,
    tmp_b2: u8,
    tmp_b1: u8,
    tmp_b0: u8,
}

struct Coefficients {
    c00: f32,
    c10: f32,
    c20: f32,
    c30: f32,
    c01: f32,
    c11: f32,
    c21: f32,
}

pub struct Dps310 {
    coefficients: Option<Coefficients>,
}

impl Dps310 {
    pub async fn configure<'a, T: Instance>(&mut self, twim: &mut Twim<'a, T>) -> XResult<()> {
        // TODO: Allow configuration of sample rate and measurement range

        // Check device ID
        let mut product_id_val = [0u8];
        twim.write_read(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_PRODUCT_ID],
            &mut product_id_val,
        )
        .await?;
        if product_id_val[0] != PRODUCT_ID_EXPECTED {
            error!(
                "DPS310 product ID did not match expected (got {}).",
                product_id_val[0]
            );
            return Err(XError::Invalid);
        }

        // Configure pressure and temperature sensors
        let pressure_config = RegPrsCfg::new()
            .with_pm_rate(0b111) // 128 measurements/sec
            .with_pm_prc(0b0100); // 16x oversampling
        twim.write(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_PRS_CFG, pressure_config.into()],
        )
        .await?;
        let temp_config = RegTmpCfg::new()
            .with_tmp_rate(0b111) // 128 measurements/sec
            .with_tmp_prc(0b0100); // 16x oversampling
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_TMP_CFG, temp_config.into()])
            .await?;
        let config = RegCfg::new()
            .with_p_shift(true) // Enable pressure shift (since we're oversampling >16x)
            .with_t_shift(true); // Enable temperature shift (since we're oversampling >16x)
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CFG, config.into()])
            .await?;

        // Wait for coefficients to become available
        let timeout_val = Instant::now() + Duration::from_millis(COEFF_RDY_TIMEOUT_MS);
        loop {
            let measurement_config = self.read_meas_cfg(twim).await?;
            if measurement_config.sensor_rdy() {
                break;
            }

            if Instant::now() > timeout_val {
                error!("Timed out while waiting for DPS310 coefficients to become readable.");
                return Err(XError::Timeout);
            }
        }

        // Read coefficients
        let mut coefficients_raw = CoefficientReadBuffer::zeroed();
        twim.write_read(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_COEF],
            bytemuck::bytes_of_mut(&mut coefficients_raw),
        )
        .await?;

        let c00 = twos_complement(
            (coefficients_raw.c00_1 as u32) << 12
                | (coefficients_raw.c00_2 as u32) << 4
                | (coefficients_raw.c00c10 as u32) >> 4,
            20,
        ) as f32;
        let c10 = twos_complement(
            ((coefficients_raw.c00c10 as u32) & 0x0F) << 16
                | (coefficients_raw.c10_1 as u32) << 8
                | (coefficients_raw.c10_2 as u32),
            20,
        ) as f32;
        let c20 = twos_complement(
            (coefficients_raw.c20_1 as u32) << 8 | (coefficients_raw.c20_2 as u32),
            16,
        ) as f32;
        let c30 = twos_complement(
            (coefficients_raw.c30_1 as u32) << 8 | (coefficients_raw.c30_2 as u32),
            16,
        ) as f32;
        let c01 = twos_complement(
            (coefficients_raw.c01_1 as u32) << 8 | (coefficients_raw.c01_2 as u32),
            16,
        ) as f32;
        let c11 = twos_complement(
            (coefficients_raw.c11_1 as u32) << 8 | (coefficients_raw.c11_2 as u32),
            16,
        ) as f32;
        let c21 = twos_complement(
            (coefficients_raw.c21_1 as u32) << 8 | (coefficients_raw.c21_2 as u32),
            16,
        ) as f32;
        self.coefficients = Some(Coefficients {
            c00,
            c10,
            c20,
            c30,
            c01,
            c11,
            c21,
        });

        // Enable
        let measurement_config = RegMeasCfg::new().with_meas_ctrl(0b111); // Continuous pressure measurement
        twim.write(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_MEAS_CFG, measurement_config.into()],
        )
        .await?;

        Ok(())
    }

    pub async fn read<'a, T: Instance>(&self, twim: &mut Twim<'a, T>) -> XResult<Option<f32>> {
        // Panic if this is called without coefficients loaded (indicating init function was not called or did not succeed).
        let coefficients = self.coefficients.as_ref().unwrap();

        // Check if measurement is ready
        let measurement_config = self.read_meas_cfg(twim).await?;
        if !measurement_config.prs_rdy() || !measurement_config.tmp_rdy() {
            return Ok(None);
        }

        let mut read_buffer = DataReadBuffer::zeroed();
        twim.write_read(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_PRS_B2],
            bytemuck::bytes_of_mut(&mut read_buffer),
        )
        .await?;

        // Calculate compensated pressure (see DPS310 datasheet section 4.9.1)
        let praw = twos_complement(
            (read_buffer.prs_b2 as u32) << 16
                | (read_buffer.prs_b1 as u32) << 8
                | read_buffer.prs_b0 as u32,
            24,
        );
        let traw = twos_complement(
            (read_buffer.tmp_b2 as u32) << 16
                | (read_buffer.tmp_b1 as u32) << 8
                | read_buffer.tmp_b0 as u32,
            24,
        );

        let praw_sc = (praw as f32) / KP_KT;
        let traw_sc = (traw as f32) / KP_KT;

        let pcomp = coefficients.c00
            + praw_sc
                * (coefficients.c10 + praw_sc * (coefficients.c20 + praw_sc * coefficients.c30))
            + traw_sc * coefficients.c01
            + traw_sc * praw_sc * (coefficients.c11 + praw_sc * coefficients.c21);

        Ok(Some(pcomp))
    }

    async fn read_meas_cfg<'a, T: Instance>(&self, twim: &mut Twim<'a, T>) -> XResult<RegMeasCfg> {
        let mut measurement_config_val = [0u8; 1];
        twim.write_read(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_MEAS_CFG],
            &mut measurement_config_val,
        )
        .await?;
        Ok(RegMeasCfg::from(measurement_config_val[0]))
    }
}

pub struct Dps310Handle {
    device: Option<Dps310>,
}

impl Dps310Handle {
    pub fn take(&mut self) -> Dps310 {
        let f = replace(&mut self.device, None);
        f.unwrap()
    }
}

pub static mut DPS310_HANDLE: Dps310Handle = Dps310Handle {
    device: Some(Dps310 { coefficients: None }),
};
