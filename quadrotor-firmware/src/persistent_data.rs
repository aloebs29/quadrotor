use bytemuck::{Pod, Zeroable};
use core::mem::MaybeUninit;
use defmt::*;
use embedded_storage_async::nor_flash::*;
use nrf_softdevice::{Flash, Softdevice};
use postcard::experimental::max_size::MaxSize;

use quadrotor_x::datatypes::PersistentDataFileContents;

use crate::xerror::{XError, XResult};

const FLASH_PAGE_SIZE: usize = 4096;

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct PersistentDataFile {
    crc: u32,
    contents: [u8; PersistentDataFileContents::POSTCARD_MAX_SIZE],
}
// NOTE: The bytemuck derive macros fail for a u8 array above a certain size. These traits are safe
// to implement since the PersistentDataFile uses a packed C repr composed of Pod/Zeroable types.
unsafe impl Zeroable for PersistentDataFile {}
unsafe impl Pod for PersistentDataFile {}

#[link_section = ".userdata.PAGE"]
static PERSISTENT_DATA_PAGE: MaybeUninit<[u8; FLASH_PAGE_SIZE]> = MaybeUninit::uninit();

pub struct PersistentDataService {
    flash: Flash,
    file_contents: PersistentDataFileContents,
}

fn persistent_data_file_addr() -> u32 {
    PERSISTENT_DATA_PAGE.as_ptr() as u32
}

impl PersistentDataService {
    pub async fn new(sd: &Softdevice) -> Self {
        let flash = Flash::take(sd);

        let mut instance = Self {
            flash,
            file_contents: PersistentDataFileContents::default(),
        };
        if let Err(_) = instance.read_file_from_flash().await {
            warn!("Persistent data file contents invalid; using defaults.");
        };
        instance
    }

    pub fn get_contents(self: &Self) -> PersistentDataFileContents {
        self.file_contents
    }

    pub async fn update_contents(
        self: &mut Self,
        mut modifier: impl FnMut(&mut PersistentDataFileContents),
    ) {
        modifier(&mut self.file_contents);
        if let Err(e) = self.write_file_to_flash().await {
            warn!("Failed to update persistent file contents! (err: {:?})", e);
        }
    }

    async fn read_file_from_flash(self: &mut Self) -> XResult<()> {
        let mut file = PersistentDataFile::zeroed();
        self.flash
            .read(
                persistent_data_file_addr(),
                bytemuck::bytes_of_mut(&mut file),
            )
            .await?;

        // NOTE: CRC is used here just as a "catch all" for version mismatches between the serialized
        // data, e.g. instead of deserializing a bunch of bogus floats if our format changes, catch
        // it and just use default data.
        let expected_crc =
            crc::Crc::<u32, crc::NoTable>::new(&crc::CRC_32_ISCSI).checksum(&file.contents);
        let actual_crc = file.crc; // needed since struct field may be misaligned
        if actual_crc != expected_crc {
            warn!(
                "Persistent data file CRC ({:04x}) did not match expected ({:04x})",
                actual_crc, expected_crc
            );
            return Err(XError::CrcMismatch);
        }

        if let Ok(file_contents) = postcard::from_bytes(&file.contents) {
            self.file_contents = file_contents;
        } else {
            warn!("Failed to deserialize persistent file contents.");
            return Err(XError::SerializationFailure);
        }

        Ok(())
    }

    async fn write_file_to_flash(self: &mut Self) -> XResult<()> {
        let mut file = PersistentDataFile::zeroed();

        // NOTE: Unwrap here as any runtime error just represents a programming error.
        unwrap!(postcard::to_slice(&self.file_contents, &mut file.contents));
        file.crc = crc::Crc::<u32, crc::NoTable>::new(&crc::CRC_32_ISCSI).checksum(&file.contents);

        // NOTE: Erase for every flash write. Wear-levelling could be used here, but the nRF52840's
        // internal flash has an endurance of 10k writes/erases, which is more than enough.
        self.flash
            .erase(
                persistent_data_file_addr(),
                persistent_data_file_addr() + (FLASH_PAGE_SIZE as u32),
            )
            .await?;
        self.flash
            .write(persistent_data_file_addr(), bytemuck::bytes_of(&file))
            .await?;

        Ok(())
    }
}
