import argparse
import asyncio
import ctypes
import math
import pathlib
import platform
from queue import Queue, Empty
from threading import Event, Thread
import time

from ahrs.filters import Madgwick
import dearpygui.dearpygui as dpg
import dearpygui_grid as dpg_grid

from ..utils import Command, COMMAND_CHARACTERISTIC, get_ble_client, Telemetry, TELEMETRY_CHARACTERISTIC, Quaternion
from .plot import TimeSeriesPlot
from .rot_viz import RotationVisualizer

ASSETS_DIR = pathlib.Path(__file__).parents[2] / "assets"
DEGREES_TO_RADIANS = 0.01745329


class _BleThread(Thread):
    def __init__(self, stop_event, telemetry_queue, command_queue, *args, **kwargs):
        super(_BleThread, self).__init__(*args, **kwargs)
        self.stop_event = stop_event
        self.telemetry_queue = telemetry_queue
        self.command_queue = command_queue

    async def run_command(self):
        def telemetry_cb(sender, data):
            self.telemetry_queue.put(Telemetry(data))

        async with get_ble_client() as ble_client:
            await ble_client.start_notify(TELEMETRY_CHARACTERISTIC, telemetry_cb)

            while not self.stop_event.is_set():
                try:
                    command = self.command_queue.get_nowait()
                    await ble_client.write_gatt_char(COMMAND_CHARACTERISTIC, command, response=False)
                except Empty:
                    await asyncio.sleep(0.1)
            await ble_client.stop_notify(TELEMETRY_CHARACTERISTIC)

    def run(self):
        try:
            asyncio.run(self.run_command())
        except BaseException as e:
            self.stop_event.set()
            raise e


class Ui:
    def __init__(self, stop_event, telemetry_queue, command_queue, use_host_orientation):
        self.stop_event = stop_event
        self.telemetry_queue = telemetry_queue
        self.command_queue = command_queue
        self.use_host_orientation = use_host_orientation

        self.calibrate_accel_time = 5.0

        # Make fonts not blurry on Windows (TODO: Is this needed on mac/linux?)
        scale_factor = 1.0
        if platform.system() == "Windows":
            scale_factor = ctypes.windll.shcore.GetScaleFactorForDevice(0) / 100
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        elif platform.system() == "Linux":
            # TODO: How to query this?
            scale_factor = 2.0

        dpg.create_context()

        # Set default font
        with dpg.font_registry():
            default_font = dpg.add_font(ASSETS_DIR / "JetBrainsMono-Regular.ttf", 16 * scale_factor)
            dpg.bind_font(default_font)

        # Setup widgets
        with dpg.window(tag="primary") as window:
            # Status + controls window
            with dpg.window(label="Status & Control", no_collapse=True, no_move=True, no_scrollbar=True,
                            no_close=True) as status_window:
                with dpg.table(header_row=False):
                    dpg.add_table_column()
                    dpg.add_table_column()
                    dpg.add_table_column()

                    with dpg.table_row():
                        dpg.add_input_float(label="seconds", min_value=0.0, max_value=20.0,
                                            default_value=self.calibrate_accel_time,
                                            callback=self._update_calibrate_accel_time)
                        dpg.add_button(label="Calibrate Accel", callback=self._calibrate_accel_button_cb)
                    with dpg.table_row():
                        self.error_count_label = dpg.add_text("Error count: ")
                    with dpg.table_row():
                        self.battery_level_label = dpg.add_text("Battery level: ")

            # Rotation visualization window
            with dpg.window(label="Rotation Visualization", no_collapse=True, no_move=True, no_scrollbar=True,
                            no_close=True) as self.rot_window:
                self.rot_visualizer = RotationVisualizer()

            # Plots
            self.orientation_plot = TimeSeriesPlot("Orientation Estimate", "Vector Magnitude (norm)", (-1, 1))
            self.orientation_w_series = self.orientation_plot.add_series("W")
            self.orientation_x_series = self.orientation_plot.add_series("X")
            self.orientation_y_series = self.orientation_plot.add_series("Y")
            self.orientation_z_series = self.orientation_plot.add_series("Z")

            self.accel_plot = TimeSeriesPlot("Accelerometer Data", "Acceleration (m/s^2)", (-40, 40))
            self.accel_x_series = self.accel_plot.add_series("X")
            self.accel_y_series = self.accel_plot.add_series("Y")
            self.accel_z_series = self.accel_plot.add_series("Z")

            self.gyro_plot = TimeSeriesPlot("Gyroscope Data", "Angular velocity (rad/s)", (-20, 20))
            self.gyro_x_series = self.gyro_plot.add_series("X")
            self.gyro_y_series = self.gyro_plot.add_series("Y")
            self.gyro_z_series = self.gyro_plot.add_series("Z")

            self.mag_plot = TimeSeriesPlot("Magnetometer Data", "Magnetic field strength (uT)", (-100, 100))
            self.mag_x_series = self.mag_plot.add_series("X")
            self.mag_y_series = self.mag_plot.add_series("Y")
            self.mag_z_series = self.mag_plot.add_series("Z")

            self.pressure_plot = TimeSeriesPlot("Pressure Sensor Data", "Pressure (kPa)")
            self.pressure_series = self.pressure_plot.add_series(None)

            # Arrange items in a grid
            grid = dpg_grid.Grid(2, 4, window)

            grid.push(status_window, 0, 0)
            grid.push(self.rot_window, (0, 1), (0, 2))
            grid.push(self.orientation_plot.plot, 0, 3)

            grid.push(self.accel_plot.plot, 1, 0)
            grid.push(self.gyro_plot.plot, 1, 1)
            grid.push(self.mag_plot.plot, 1, 2)
            grid.push(self.pressure_plot.plot, 1, 3)

        # Finish setting up dear imgui
        dpg.create_viewport(title="Quadrotor Telemetry Viewer")
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.maximize_viewport()
        dpg.set_primary_window("primary", True)

        with dpg.item_handler_registry() as window_hr:
            dpg.add_item_visible_handler(callback=grid)
        dpg.bind_item_handler_registry(window, window_hr)

    def _update_calibrate_accel_time(self, sender, app_data):
        self.calibrate_accel_time = app_data

    def _calibrate_accel_button_cb(self, sender, app_data):
        self.command_queue.put(Command.calibrate_accel(self.calibrate_accel_time))

    def run(self):
        timestamp_offset = None
        q = Quaternion(1.0, 0.0, 0.0, 0.0)
        madgwick = Madgwick()
        last_timestamp = None

        while dpg.is_dearpygui_running() and not stop_event.is_set():
            # Read new queue items
            while not telemetry_queue.empty():
                t = telemetry_queue.get()

                # Establish time axis offset if not already established
                new_timestamp = t.timestamp
                if timestamp_offset is None:
                    timestamp_offset = new_timestamp - time.time()

                # Update status window
                dpg.set_value(self.error_count_label, f"Error count: {t.error_count}")
                dpg.set_value(self.battery_level_label, f"Battery level: {t.battery_voltage:.2f}V")

                # Update host-side orientation if thats what we're using
                if self.use_host_orientation:
                    if last_timestamp is not None:
                        try:
                            madgwick.Dt = new_timestamp - last_timestamp
                            q = Quaternion(
                                *madgwick.updateMARG(q.as_array(), t.gyro.scale(DEGREES_TO_RADIANS).as_array(),
                                                     t.accel.as_array(), t.mag.as_array()))
                        except ZeroDivisionError:
                            pass
                else:
                    q = t.orientation

                # Update orientation plot and rotation visualizer
                try:
                    q_axis_mag = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
                    theta = 2 * math.atan2(q_axis_mag, q.w)

                    self.rot_visualizer.transform([q.x / q_axis_mag, q.y / q_axis_mag, q.z / q_axis_mag], theta)
                except ZeroDivisionError:
                    pass

                self.orientation_w_series.append_data(new_timestamp, q.w)
                self.orientation_x_series.append_data(new_timestamp, q.x)
                self.orientation_y_series.append_data(new_timestamp, q.y)
                self.orientation_z_series.append_data(new_timestamp, q.z)

                # Update other plots
                self.accel_x_series.append_data(new_timestamp, t.accel.x)
                self.accel_y_series.append_data(new_timestamp, t.accel.y)
                self.accel_z_series.append_data(new_timestamp, t.accel.z)

                self.gyro_x_series.append_data(new_timestamp, t.gyro.x)
                self.gyro_y_series.append_data(new_timestamp, t.gyro.y)
                self.gyro_z_series.append_data(new_timestamp, t.gyro.z)

                self.mag_x_series.append_data(new_timestamp, t.mag.x)
                self.mag_y_series.append_data(new_timestamp, t.mag.y)
                self.mag_z_series.append_data(new_timestamp, t.mag.z)

                self.pressure_series.append_data(new_timestamp, t.pressure)

                # Update last timestamp (for host side orientation calcs)
                last_timestamp = new_timestamp

            # Shift timeseries plots X-values
            if timestamp_offset is not None:
                xaxis_end = time.time() + timestamp_offset

                self.orientation_plot.update(xaxis_end)

                self.accel_plot.update(xaxis_end)
                self.gyro_plot.update(xaxis_end)
                self.mag_plot.update(xaxis_end)
                self.pressure_plot.update(xaxis_end)

            # Update rotation visualizer clip space
            self.rot_visualizer.update(dpg.get_item_width(self.rot_window), dpg.get_item_height(self.rot_window))

            dpg.render_dearpygui_frame()

        dpg.destroy_context()


if __name__ == "__main__":
    # Parse CLI arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--use-host-orientation", action="store_true",
                        help="Calculate orientation on the host rather than using values calculated on target device.")
    args = parser.parse_args()

    # Setup second thread for receiving telemetry notifications over BLE.
    stop_event = Event()
    telemetry_queue = Queue()
    command_queue = Queue()
    ble_thread = _BleThread(stop_event, telemetry_queue, command_queue)
    ble_thread.start()

    try:
        ui = Ui(stop_event, telemetry_queue, command_queue, args.use_host_orientation)
        ui.run()
    finally:
        stop_event.set()
        ble_thread.join()
