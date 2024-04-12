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
