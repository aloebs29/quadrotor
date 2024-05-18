use core::mem::replace;

use bitfield_struct::bitfield;
use bytemuck::{Pod, Zeroable};
use defmt::{debug, error};
use embassy_nrf::twim::{Instance, Twim};

use quadrotor_x::datatypes::Vec3f;

use crate::xerror::*;

const DEVICE_ADDRESS: u8 = 0x21;

const REG_ADDRESS_STATUS: u8 = 0x00;
const REG_ADDRESS_WHOAMI: u8 = 0x0C;
const REG_ADDRESS_CTRL_REG0: u8 = 0x0D;
const REG_ADDRESS_CTRL_REG1: u8 = 0x13;

const WHOAMI_EXPECTED: u8 = 0xD7;
const SCALE: f32 = 0.03125;
const DEGREES_TO_RADIANS: f32 = 0.01745329;

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
struct RegCtrl0 {
    #[bits(2)]
    fs: u8,
    hpf_en: bool,
    #[bits(2)]
    sel: u8,
    spiw: bool,
    #[bits(2)]
    bw: u8,
}

#[bitfield(u8)]
struct RegCtrl1 {
    ready: bool,
    active: bool,
    #[bits(3)]
    dr: u8,
    st: bool,
    rst: bool,
    __: bool,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct DataReadBuffer {
    status: u8,
    x_msb: u8,
    x_lsb: u8,
    y_msb: u8,
    y_lsb: u8,
    z_msb: u8,
    z_lsb: u8,
}

pub struct Fxas21002 {
    _private: (),
}

impl Fxas21002 {
    pub async fn configure<'a, T: Instance>(&mut self, twim: &mut Twim<'a, T>) -> XResult<()> {
        // TODO: Allow configuration of sample rate and measurement range

        // Check device ID
        let mut whoami_val = [0u8];
        twim.write_read(DEVICE_ADDRESS, &[REG_ADDRESS_WHOAMI], &mut whoami_val)
            .await?;
        if whoami_val[0] != WHOAMI_EXPECTED {
            error!(
                "Fxas21002 device ID did not match expected (got {}).",
                whoami_val[0]
            );
            return Err(XError::Invalid);
        }

        // Place in standby
        let standby_val = RegCtrl1::new().with_active(false);
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CTRL_REG1, standby_val.into()])
            .await?;

        // Configure
        let ctrl0 = RegCtrl0::new().with_fs(0b01); // full scale range +/-1000dps
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CTRL_REG0, ctrl0.into()])
            .await?;

        // Final configuration and take device out of standby
        let ctrl1 = RegCtrl1::new()
            .with_dr(0b010) // 200 Hz data rate
            .with_active(true); // enable
        twim.write(DEVICE_ADDRESS, &[REG_ADDRESS_CTRL_REG1, ctrl1.into()])
            .await?;

        Ok(())
    }

    pub async fn read<'a, T: Instance>(
        &mut self,
        twim: &mut Twim<'a, T>,
        gyro_out: &mut Vec3f,
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
            debug!("Data not ready when Fxas21002 read was performed!");
        }

        gyro_out.x =
            Self::scale(((read_buffer.x_msb as u16) << 8 | read_buffer.x_lsb as u16) as i16);
        gyro_out.y =
            Self::scale(((read_buffer.y_msb as u16) << 8 | read_buffer.y_lsb as u16) as i16);
        gyro_out.z =
            Self::scale(((read_buffer.z_msb as u16) << 8 | read_buffer.z_lsb as u16) as i16);

        Ok(())
    }

    fn scale(raw: i16) -> f32 {
        f32::from(raw) * SCALE * DEGREES_TO_RADIANS
    }
}

pub struct Fxas21002Handle {
    device: Option<Fxas21002>,
}

impl Fxas21002Handle {
    pub fn take(&mut self) -> Fxas21002 {
        let f = replace(&mut self.device, None);
        f.unwrap()
    }
}

pub static mut FXAS21002_HANDLE: Fxas21002Handle = Fxas21002Handle {
    device: Some(Fxas21002 { _private: () }),
};
