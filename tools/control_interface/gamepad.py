# Adapted from gist by user rdb: https://gist.github.com/rdb/8864666

import array
from dataclasses import dataclass
from fcntl import ioctl
import os
import struct
import time

from ..utils import ControllerSetpoints

JSIOCG_AXES = 0x80016a11
JSIOCG_GAXMAP = 0x80406a32

JS_EVENT_TYPE_AXIS = 0x02
JS_EVENT_FORMAT_STRING = "IhBB"

AXIS_MAX = 32767.0


@dataclass
class JsEvent:
    time: int
    value: int
    type: int
    number: int

    @staticmethod
    def from_event_buf(event_buf):
        return JsEvent(*struct.unpack(JS_EVENT_FORMAT_STRING, event_buf))

    @staticmethod
    def calcsize():
        return struct.calcsize(JS_EVENT_FORMAT_STRING)


class GamepadInputHandler:
    def __init__(self, dev_path="/dev/input/js0"):
        self.axis_values = [0] * 4

        self.device = None
        self.setter_map = {}

        # TODO: Add windows support
        if os.path.exists(dev_path):
            self.device = open(dev_path, "rb")
        else:
            # NOTE: We still want to support using the control interface as just a telemetry viewer and not an input
            print(f"Did not find gamepad to use for input; the device at {dev_path} does not exist.")
            return

        # Map the device's axis indexes to our `axis_values` indexes
        axis_ids = [
            0x01,  # left stick Y - thrust
            0x03,  # right stick X - roll
            0x04,  # right stick Y - pitch
            0x00,  # left stick X - yaw
        ]
        self.axis_index_map = {}

        buf = array.array("B", [0])
        ioctl(self.device, JSIOCG_AXES, buf)
        num_axes = buf[0]

        buf = array.array("B", [0] * num_axes)
        ioctl(self.device, JSIOCG_GAXMAP, buf)
        for device_axis_index, axis_id in enumerate(buf):
            try:
                axis_index = axis_ids.index(axis_id)
                self.axis_index_map[device_axis_index] = axis_index
            except ValueError:
                continue

        os.set_blocking(self.device.fileno(), False)

    def update(self):
        if self.device is None:
            return ControllerSetpoints(0, 0, 0, 0)

        # Read all pending events
        while True:
            event_buf = self.device.read(JsEvent.calcsize())
            if not event_buf:
                break
            event = JsEvent.from_event_buf(event_buf)
            if event.type & JS_EVENT_TYPE_AXIS and event.number in self.axis_index_map:
                axis_index = self.axis_index_map[event.number]
                fvalue = event.value / AXIS_MAX
                self.axis_values[axis_index] = fvalue

        setpoints = ControllerSetpoints(*self.axis_values)

        # Apply fixups
        setpoints.thrust = (-setpoints.thrust + 1.0) / 2.0
        return setpoints


if __name__ == "__main__":
    try:
        handler = GamepadInputHandler()
        while True:
            print(handler.update())
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
