use core::mem::replace;

use bitfield_struct::bitfield;
use bytemuck::{Pod, Zeroable};
use defmt::{debug, error};
use embassy_nrf::twim::{Instance, Twim};
use micromath::vector::F32x3;

use crate::xerror::*;

const DEVICE_ADDRESS: u8 = 0x1F;

const REG_ADDRESS_STATUS: u8 = 0x00;
const REG_ADDRESS_WHOAMI: u8 = 0x0D;
const REG_ADDRESS_XYZ_DATA_CFG: u8 = 0x0E;
const REG_ADDRESS_CTRL_REG1: u8 = 0x2A;
const REG_ADDRESS_M_CTRL_REG1: u8 = 0x5B;
const REG_ADDRESS_M_CTRL_REG2: u8 = 0x5C;

const WHOAMI_EXPECTED: u8 = 0xC7;
const ACCEL_SCALE: f32 = 0.488;
const MILLIG_TO_MPS2: f32 = 0.00980665;
const MAG_SCALE: f32 = 0.1;

#[bitfield(u8)]
struct RegStatus {
    xdr: bool,
    ydr: bool,
    zdr: bool,
    zyxdr: bool,
    xow: bool,
    yow: bool,
    zow: bool,
    zyxow: bool,
}

#[bitfield(u8)]
struct RegXyzDataCfg {
    #[bits(2)]
    fs: u8,
    #[bits(2)]
    __: u8,
    hpf_out: bool,
    #[bits(3)]
    __: u8,
}

#[bitfield(u8)]
struct RegCtrl1 {
    active: bool,
    f_read: bool,
    lnoise: bool,
    #[bits(3)]
    dr: u8,
    #[bits(2)]
    aslp_rate: u8,
}

#[bitfield(u8)]
struct RegMagCtrl1 {
    #[bits(2)]
    m_hms: u8,
    #[bits(3)]
    m_os: u8,
    m_ost: bool,
    m_rst: bool,
    m_acal: bool,
}

#[bitfield(u8)]
struct RegMagCtrl2 {
    #[bits(2)]
    m_rst_cnt: u8,
    m_maxmin_rst: bool,
    m_maxmin_dis_ths: bool,
    m_maxmin_dis: bool,
    hyb_autoinc_mode: bool,
    #[bits(2)]
    __: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct DataReadBuffer {
    status: u8,
    accel_x_msb: u8,
    accel_x_lsb: u8,
    accel_y_msb: u8,
    accel_y_lsb: u8,
    accel_z_msb: u8,
    accel_z_lsb: u8,
    mag_x_msb: u8,
    mag_x_lsb: u8,
    mag_y_msb: u8,
    mag_y_lsb: u8,
    mag_z_msb: u8,
    mag_z_lsb: u8,
}

pub struct Fxos8700 {
    _private: (),
}

impl Fxos8700 {
    pub async fn configure<'a, T: Instance>(&mut self, twim: &mut Twim<'a, T>) -> XResult<()> {
        // TODO: Allow configuration of sample rate and measurement range

        // Check device ID
        let mut whoami_val = [0u8];
        twim.write_read(DEVICE_ADDRESS, &[REG_ADDRESS_WHOAMI], &mut whoami_val)
            .await?;
        if whoami_val[0] != WHOAMI_EXPECTED {
            error!(
                "FXOS8700 device ID did not match expected (got {}).",
                whoami_val[0]
            );
            return Err(XError::Invalid);
        }

        // Place in standby
        let standby_val = RegCtrl1::new().with_active(false);
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CTRL_REG1, standby_val.into()])
            .await?;

        // Configure
        let mag_ctrl1 = RegMagCtrl1::new()
            .with_m_acal(true) // Enable auto-calibration
            .with_m_os(0b111) // 8x oversampling (for 200Hz) to reduce noise
            .with_m_hms(0b11); // hybrid mode with accel/mag active
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_M_CTRL_REG1, mag_ctrl1.into()])
            .await?;

        let mag_ctrl2 = RegMagCtrl2::new().with_hyb_autoinc_mode(true); // map magnetometer read regs after accel read regs
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_M_CTRL_REG2, mag_ctrl2.into()])
            .await?;

        let xyz_data_cfg = RegXyzDataCfg::new().with_fs(0b01); // accel full scale range +/-4g
        twim.write(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_XYZ_DATA_CFG, xyz_data_cfg.into()],
        )
        .await?;

        // Final configuration and take device out of standby
        let ctrl1 = RegCtrl1::new()
            .with_dr(0b001) // 200 Hz data rate
            .with_lnoise(true) // low noise mode
            .with_active(true); // enable
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CTRL_REG1, ctrl1.into()])
            .await?;

        Ok(())
    }

    pub async fn read<'a, T: Instance>(
        &mut self,
        twim: &mut Twim<'a, T>,
        accel_out: &mut F32x3,
        mag_out: &mut F32x3,
    ) -> XResult<()> {
        let mut read_buffer = DataReadBuffer::zeroed();
        twim.write_read(
            DEVICE_ADDRESS,
            &[REG_ADDRESS_STATUS],
            bytemuck::bytes_of_mut(&mut read_buffer),
        )
        .await?;

        let status = RegStatus::from(read_buffer.status);
        if !status.xdr() || !status.ydr() || !status.zdr() {
            debug!("Data not ready when FXOS8700 read was performed!");
        }

        accel_out.x = Self::scale_accel(
            (((read_buffer.accel_x_msb as u16) << 8 | read_buffer.accel_x_lsb as u16) as i16) >> 2,
        );
        accel_out.y = Self::scale_accel(
            (((read_buffer.accel_y_msb as u16) << 8 | read_buffer.accel_y_lsb as u16) as i16) >> 2,
        );
        accel_out.z = Self::scale_accel(
            (((read_buffer.accel_z_msb as u16) << 8 | read_buffer.accel_z_lsb as u16) as i16) >> 2,
        );

        mag_out.x = Self::scale_mag(
            ((read_buffer.mag_x_msb as u16) << 8 | read_buffer.mag_x_lsb as u16) as i16,
        );
        mag_out.y = Self::scale_mag(
            ((read_buffer.mag_y_msb as u16) << 8 | read_buffer.mag_y_lsb as u16) as i16,
        );
        mag_out.z = Self::scale_mag(
            ((read_buffer.mag_z_msb as u16) << 8 | read_buffer.mag_z_lsb as u16) as i16,
        );

        Ok(())
    }

    fn scale_accel(raw: i16) -> f32 {
        f32::from(raw) * ACCEL_SCALE * MILLIG_TO_MPS2
    }

    fn scale_mag(raw: i16) -> f32 {
        f32::from(raw) * MAG_SCALE
    }
}

pub struct Fxos8700Handle {
    device: Option<Fxos8700>,
}

impl Fxos8700Handle {
    pub fn take(&mut self) -> Fxos8700 {
        let f = replace(&mut self.device, None);
        f.unwrap()
    }
}

pub static mut FXOS8700_HANDLE: Fxos8700Handle = Fxos8700Handle {
    device: Some(Fxos8700 { _private: () }),
};
