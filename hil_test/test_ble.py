import asyncio
from dataclasses import dataclass
import struct

import bleak
import pytest

TELEMETRY_CHARACTERISTIC = "6dff68a3-f84a-4f54-a244-cc0b528425ea"


@dataclass
class Vec3:
    x: float
    y: float
    z: float


class Telemetry:
    def __init__(self, packed_bytes):
        t = struct.unpack("<ffffffffffQ", packed_bytes)
        self.battery_voltage = t[0]
        self.accel = Vec3(t[1], t[2], t[3])
        self.gyro  = Vec3(t[4], t[5], t[6])
        self.mag   = Vec3(t[7], t[8], t[9])
        self.timestamp = t[10] / 1000 # millis -> seconds


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
        assert t.battery_voltage > 3.0 and t.battery_voltage < 4.5

        if i > 0:
            tlast = telemetry_entries[i - 1]
            assert t.timestamp > tlast.timestamp
