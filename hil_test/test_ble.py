import asyncio
from dataclasses import dataclass
import math
import struct

import bleak
import pytest

TELEMETRY_CHARACTERISTIC = "6dff68a3-f84a-4f54-a244-cc0b528425ea"


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


class Telemetry:
    def __init__(self, packed_bytes):
        t = struct.unpack("<QIffffffffff", packed_bytes)
        self.timestamp = t[0] # millis
        self.error_count = t[1]
        self.battery_voltage = t[2] # V
        self.accel = Vec3(t[3], t[4], t[5]) # mg
        self.gyro  = Vec3(t[6], t[7], t[8]) # dps
        self.mag   = Vec3(t[9], t[10], t[11]) # uT


@pytest.mark.asyncio
async def test_ble_battery_service(ble_client):
    battery_level = await ble_client.read_gatt_char(bleak.uuids.normalize_uuid_str("2A19"))
    assert len(battery_level) == 1

    battery_level = battery_level[0]
    assert battery_level >= 0 and battery_level <= 100


@pytest.mark.asyncio
async def test_ble_telemetry_service(ble_client):
    telemetry_entries = []
    def telemetry_cb(sender, data):
        telemetry_entries.append(Telemetry(data))

    await ble_client.start_notify(TELEMETRY_CHARACTERISTIC, telemetry_cb)
    await asyncio.sleep(1)
    await ble_client.stop_notify(TELEMETRY_CHARACTERISTIC)

    # NOTE: The _nominal_ telemetry rate is 100/s
    assert len(telemetry_entries) > 80
    for i in range(len(telemetry_entries)):
        t = telemetry_entries[i]

        accel_magnitude = t.accel.magnitude()
        mag_magnitude = t.mag.magnitude()
        gyro_magnitude = t.gyro.magnitude()

        # Make sure values are in sensible range
        # NOTE: This assumes the device is at rest with a reasonably small amount of magnetic field interference. The
        #       magnetometer test may need to be commented out when the device has not had a chance to auto-calibrate
        #       (via being rotated in a figure-8 pattern).
        assert t.battery_voltage > 3.0 and t.battery_voltage < 4.5
        assert accel_magnitude > 800 and accel_magnitude < 1200
        assert mag_magnitude > 25 and mag_magnitude < 65
        assert gyro_magnitude > -10 and gyro_magnitude < 10

        # Make sure entries are sequential
        if i > 0:
            tlast = telemetry_entries[i - 1]
            assert t.timestamp > tlast.timestamp
