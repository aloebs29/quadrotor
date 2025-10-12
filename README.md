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

[Cargo make](https://github.com/sagiegurari/cargo-make) is used to build/test the code. Run `cargo make --list-all-steps` to see a list of all tasks and their descriptions.

The firmware is written on top of the [Embassy](https://github.com/embassy-rs/embassy) framework which provides hardware abstraction and async primitives + constructions.

### Async tasks

![task relationships diagram](./assets/task_relationships.png)

#### Main task

The main task performs initialization of the device, spawns all other tasks, then executes the application's main loop. The main loop:
* processes commands received from BLE and/or the USB CLI
* collects new sensor inputs
* estimates the device's orientation based on those sensor inputs
* updates the controller based on the current inputs and orientation estimate
* updates motor outputs based on the controller output

#### Status LED task

Blinks the LED based on a blink pattern which corresponds with the current controller state (i.e. enabled/disabled).

#### BLE task

Runs in a loop which waits for a connection from a peer, services that connection with the peer until it is disconnected, then waits for the next connection (and so on). While a connection is active, the task will notify the peer of updates to the device's telemetry (e.g. battery level, sensor inputs, orientation estimate, etc.), as well as wait for commands sent from the peer (e.g. target orientation and thrust, as well as initiating discrete operations such as calculating the accelerometer offset).

#### Softdevice task

The softdevice task is used to service the Nordic Softdevice (which performs much of the underlying interaction with the radio peripheral). The softdevice also sends a VBUS detect signal to the USB task.

#### USB task

Services the underlying USB peripheral interaction.

#### Serial task

Runs in a loop which waits for a connection from a peer, services that connection with the peer until it is disconnected, then waits for the next connection (and so on). While a connection is active, the task will push bytes received by the peer into a pipe for consumption by the main task (which runs a CLI parser) and pop bytes off of a pipe produced by the main task (e.g. CLI responses) and write them to the peer.

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

### Control interface

A basic control interface which accepts gamepad inputs to control the quadrotor and contains a live-plotter for
telemetry data from the target device can be run with:

```
python -m tools.control_interface
```

The control interface requires that the device is connected via USB (to query the BLE address from the device) and is
connectable via BLE.

TODO: Allow optional arg to bypass the BLE address query over USB and let the user choose from scanned devices.
