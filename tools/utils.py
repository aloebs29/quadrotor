from contextlib import (asynccontextmanager, contextmanager)
from dataclasses import dataclass
import math
import struct

import bleak
import numpy as np
import serial
from serial.tools import list_ports

TELEMETRY_CHARACTERISTIC = "6dff68a3-f84a-4f54-a244-cc0b528425ea"
COMMAND_CHARACTERISTIC = "51e426ca-502f-405b-89dc-1b299df7cf32"
_SERIAL_TIMEOUT = 2.0


@contextmanager
def get_usb_serial():
    dev = None
    for port in list_ports.comports():
        if port.vid == 0xFEED and port.pid == 0xFACE:
            dev = port.device
            break
    else:
        raise Exception("Did not find USB device.")

    ser = serial.Serial(dev, baudrate=115200, timeout=_SERIAL_TIMEOUT)
    yield ser
    ser.close()


@asynccontextmanager
async def get_ble_client(query_mac_over_usb=False):
    if query_mac_over_usb:
        # Get BLE address
        with get_usb_serial() as usb_serial:
            usb_serial.write(b"mac_address\n")

            response = usb_serial.readline()
            ble_address = bytes.decode(response).strip()

        ble_device = await bleak.BleakScanner.find_device_by_address(ble_address)
        if ble_device is None:
            raise ValueError(f"Could not find BLE device with address {ble_address}.")

    else:
        ble_device = await bleak.BleakScanner.find_device_by_name("Quadcopter")
        if ble_device is None:
            raise ValueError(f"Could not find BLE device with the expected name and characteristics.")

    async with bleak.BleakClient(ble_device) as client:
        yield client


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def scale(self, factor):
        return Vec3(self.x * factor, self.y * factor, self.z * factor)

    def as_array(self):
        return np.array([self.x, self.y, self.z])


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def normalize(self):
        a = self.as_array()
        return Quaternion(*(a / np.linalg.norm(a)))

    def as_array(self):
        return np.array([self.w, self.x, self.y, self.z])


class _PostcardDeserializer:
    # https://postcard.jamesmunns.com/wire-format.html

    def __init__(self, bytes_):
        self._bytes = bytes_

    def pop_f32(self):
        val = struct.unpack("<f", self._bytes[:4])[0]
        self._bytes = self._bytes[4:]
        return val

    def pop_varint(self):
        val = 0
        while True:
            current = self._bytes.pop(0)
            val = val << 7 | current & 0x7F
            if not (current & 0x80):
                break
        return val

    def pop_vec3(self):
        return Vec3(self.pop_f32(), self.pop_f32(), self.pop_f32())

    def pop_quat(self):
        return Quaternion(self.pop_f32(), self.pop_f32(), self.pop_f32(), self.pop_f32())


class _PostcardSerializer:
    # https://postcard.jamesmunns.com/wire-format.html

    def __init__(self):
        self._bytes = bytes()

    def push_f32(self, val):
        self._bytes += struct.pack("<f", val)

    def push_varint(self, val):
        while True:
            current = val & 0x7F
            val >>= 7
            if val:
                self._bytes += (current | 0x80).to_bytes()
            else:
                self._bytes += current.to_bytes()
                break

    def get_bytes(self):
        return self._bytes


class Telemetry:
    def __init__(self, packed_bytes):
        # TODO: Figure out a [concise] way to not duplicate these serialized structs in Rust/Python..
        #
        # Using a lib for common/serializable types that gets compiled for firmware as well as for Python bindings (PyO3)
        # seems like a good idea at first.. but PyO3 does not generate struct field accessors, so these types would end
        # up needing PyO3 methods for accessing each field (or a __getattr__ implementation that matches them all).

        deser = _PostcardDeserializer(packed_bytes)
        self.timestamp = deser.pop_f32()  # seconds
        self.error_count = deser.pop_varint()
        self.battery_voltage = deser.pop_f32()  # V
        self.accel = deser.pop_vec3()  # m/s^2
        self.gyro = deser.pop_vec3()  # rad/s
        self.mag = deser.pop_vec3()  # uT
        self.pressure = deser.pop_f32()  # Pa
        self.orientation = deser.pop_quat()


class Command:
    @staticmethod
    def calibrate_accel(seconds):
        ser = _PostcardSerializer()
        ser.push_varint(0)
        ser.push_f32(seconds)
        return ser.get_bytes()
