import evdev

from ..utils import ControllerSetpoints, Command

AXIS_MAX = 32768

class GamepadInputHandler:
    def __init__(self):
        self.left_stick_x = 0
        self.left_stick_y = 0
        self.right_stick_x = 0
        self.right_stick_y = 0

        self.device = None

        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            # TODO: Figure out a better way to generically support gamepads (this just happens to work for my gamepad..)
            if "Gamepad" in device.name:
                self.device = device
                break
        else:
            # NOTE: We still want to support using the telemetry viewer as just a telemetry viewer and not an input
            print("Did not find gamepad to use for input.")

    def update(self, setpoints_queue):
        if self.device is None:
            return

        try:
            for event in self.device.read():
                print(f"{event.type} {event.code} {event.value}")
                if event.type == evdev.ecodes.EV_ABS:
                    if event.code == evdev.ecodes.ABS_X:
                        self.left_stick_x = event.value
                    elif event.code == evdev.ecodes.ABS_Y:
                        self.left_stick_y = event.value
                    elif event.code == evdev.ecodes.ABS_RX:
                        self.right_stick_x = event.value
                    elif event.code == evdev.ecodes.ABS_RY:
                        self.right_stick_y = event.value
        except BlockingIOError:
            pass

        setpoints = ControllerSetpoints(
            self.left_stick_y / AXIS_MAX,
            self.right_stick_x / AXIS_MAX,
            self.right_stick_y / AXIS_MAX,
            self.left_stick_x / AXIS_MAX,
            )
        setpoints_queue.put(Command.update_controller_setpoints(setpoints))