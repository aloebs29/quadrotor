from contextlib import (asynccontextmanager, contextmanager)
from dataclasses import dataclass
import math
import struct

import bleak
import serial
from serial.tools import list_ports

TELEMETRY_CHARACTERISTIC = "6dff68a3-f84a-4f54-a244-cc0b528425ea"
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
async def get_ble_client():
    # Get BLE address
    with get_usb_serial() as usb_serial:
        usb_serial.write(b"mac_address\n")

        response = usb_serial.readline()
        ble_address = bytes.decode(response).strip()

    async with bleak.BleakClient(ble_address) as client:
        yield client


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


class Telemetry:
    def __init__(self, packed_bytes):
        t = struct.unpack("<QIfffffffffff", packed_bytes)
        self.timestamp = t[0] # millis
        self.error_count = t[1]
        self.battery_voltage = t[2] # V
        self.accel = Vec3(t[3], t[4], t[5]) # mg
        self.gyro  = Vec3(t[6], t[7], t[8]) # dps
        self.mag   = Vec3(t[9], t[10], t[11]) # uT
        self.pressure = t[12] # Pa