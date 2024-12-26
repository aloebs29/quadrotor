MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* These values correspond to the NRF52840 without a Softdevice */
  /*
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
  */

  /* These values correspond to the NRF52840 with Softdevices S140 7.3.0 */
  FLASH : ORIGIN = 0x00027000, LENGTH = 864K
  USERDATA : ORIGIN = 0x000FF000, LENGTH = 4K
  RAM : ORIGIN = 0x20002ce0, LENGTH = 250656
}

SECTIONS
{
    .userdata (NOLOAD) : ALIGN(4)
    {
        *(.userdata .userdata.*);
        . = ALIGN(4);
    } > USERDATA
}