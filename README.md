# Quadrotor

WIP quadcopter flight controller.

## Hardware

* [nRF52840 feather](https://www.adafruit.com/product/4062)
* [FXOS8700 accelerometer and magnetometer and FXAS21002 gyroscope](https://www.adafruit.com/product/3463)
* [DPS310 pressure sensor](https://www.adafruit.com/product/4494)

## Software

The flight controller software is written in rust and targets the nRF52840 SoC. The top-level directory of the repo is
a cargo workspace with the following members:
* `quadrotor-x` Hardware-independent controls (and other utility) code. This is in a separate crate to allow easy testing
    on the host machine.
* `quadrotor-firmware` Platform-specific code that depends on `quadrotor-x` and runs on the target platform (nRF52840).

[Cargo make](https://github.com/sagiegurari/cargo-make) is used to build/test the code. Run
`cargo make --list-all-steps` to see a list of all tasks and their descriptions.

## Tools

The host tools require that Python 3.10 (or above) is installed on the host machine, and that dependencies are installed
with:

```
pip install -r tools/requirements.txt
```

### Hardware-in-the-loop testing

There are basic HIL tests found in the `tools/hil_test` directory. The tests require that the target device be plugged
into the host machine via USB, and that the host machine has a Bluetooth adapter. Currently, the tests do some basic
sanity checking on the USB serial and BLE interfaces themselves, as well as the telemetry service provided by the device
(via BLE). In order for these sanity checks on the telemetry data to pass, a battery must be plugged in, the device should
be at rest (accel ~1g and angular velocity ~0), and the magnetic field interference should be reasonably small (the
magnetometer is configured to auto-calibrate, but these aren't very effective until the device has been moved around in
a figure-8 pattern).

To run the tests, run:

```
pytest tools/hil_test/
```

The test setup will automatically build and flash the firmware onto the target device, then reset and run the target device.
This relies on a probe-rs compatible debugger being connected to the target device. To run without flashing/resetting
the target device, use:

```
pytest tools/hil_test/ --skip_flash
```

### Telemetry viewer

A basic live-plotter for telemetry data from the target device can be run with:

```
python -m tools.telemetry_viewer
```

The telemetry viewer requires that the device is connected via USB (to query the BLE address from the device) and is
connectable via BLE.

TODO: Allow optional arg to bypass the BLE address query over USB and let the user choose from scanned devices.
