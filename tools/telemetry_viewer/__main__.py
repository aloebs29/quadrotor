import argparse
import asyncio
import ctypes
import math
import pathlib
import platform
from queue import Queue
from threading import Event, Thread
import time

from ahrs.filters import Madgwick
import dearpygui.dearpygui as dpg
import dearpygui_grid as dpg_grid

from ..utils import get_ble_client, Telemetry, TELEMETRY_CHARACTERISTIC, Quaternion
from .plot import TimeSeriesPlot
from .rot_viz import RotationVisualizer

ASSETS_DIR = pathlib.Path(__file__).parents[2] / "assets"
DEGREES_TO_RADIANS = 0.01745329


class _TelemetryReadThread(Thread):
    def __init__(self, stop_event, queue, *args, **kwargs):
        super(_TelemetryReadThread, self).__init__(*args, **kwargs)
        self.stop_event = stop_event
        self.queue = queue

    async def run_command(self):
        def telemetry_cb(sender, data):
            self.queue.put(Telemetry(data))

        async with get_ble_client() as ble_client:
            await ble_client.start_notify(TELEMETRY_CHARACTERISTIC, telemetry_cb)
            
            while not self.stop_event.is_set():
                await asyncio.sleep(0.1)
            await ble_client.stop_notify(TELEMETRY_CHARACTERISTIC)
            
    def run(self):
        try:
            asyncio.run(self.run_command())
        except BaseException as e:
            self.stop_event.set()
            raise e


def run_ui(stop_event, telemetry_queue, use_host_orientation):
    # Make fonts not blurry on Windows (TODO: Is this needed on mac/linux?)
    scale_factor = 1.0
    if platform.system() == "Windows":
        scale_factor = ctypes.windll.shcore.GetScaleFactorForDevice(0) / 100
        ctypes.windll.shcore.SetProcessDpiAwareness(2)

    dpg.create_context()

    # Set default font
    with dpg.font_registry():
        default_font = dpg.add_font(ASSETS_DIR / "JetBrainsMono-Regular.ttf", 16 * scale_factor)
        dpg.bind_font(default_font)

    # Setup widgets
    with dpg.window(tag="primary") as window:
        # Status + controls window
        with dpg.window(label="Status", no_collapse=True, no_move=True, no_scrollbar=True, no_close=True) as status_window:
            error_count_label = dpg.add_text("Error count: ")
            battery_level_label = dpg.add_text("Battery level: ")

        # Rotation visualization window
        with dpg.window(label="Rotation Visualization", no_collapse=True, no_move=True, no_scrollbar=True, no_close=True) as rot_window:
            rot_visualizer = RotationVisualizer()

        # Plots
        orientation_plot = TimeSeriesPlot("Orientation", "Vector Magnitude (norm)", -1, 1)
        orientation_w_series = orientation_plot.add_series("W")
        orientation_x_series = orientation_plot.add_series("X")
        orientation_y_series = orientation_plot.add_series("Y")
        orientation_z_series = orientation_plot.add_series("Z")

        accel_plot = TimeSeriesPlot("Accelerometer Data", "Acceleration (g)", -4, 4)
        accel_x_series = accel_plot.add_series("X")
        accel_y_series = accel_plot.add_series("Y")
        accel_z_series = accel_plot.add_series("Z")

        gyro_plot = TimeSeriesPlot("Gyroscope Data", "Angular velocity (dps)", -1000, 1000)
        gyro_x_series = gyro_plot.add_series("X")
        gyro_y_series = gyro_plot.add_series("Y")
        gyro_z_series = gyro_plot.add_series("Z")

        mag_plot = TimeSeriesPlot("Magnetometer Data", "Magnetic field strength (uT)", -100, 100)
        mag_x_series = mag_plot.add_series("X")
        mag_y_series = mag_plot.add_series("Y")
        mag_z_series = mag_plot.add_series("Z")

        pressure_plot = TimeSeriesPlot("Pressure Sensor Data", "Pressure (kPa)", 100, 105)
        pressure_series = pressure_plot.add_series(None)

        # Arrange items in a grid
        grid = dpg_grid.Grid(2, 4, window)

        grid.push(status_window, 0, 0)
        grid.push(rot_window, (0, 1), (0, 2))
        grid.push(orientation_plot.plot, 0, 3)

        grid.push(accel_plot.plot, 1, 0)
        grid.push(gyro_plot.plot, 1, 1)
        grid.push(mag_plot.plot, 1, 2)
        grid.push(pressure_plot.plot, 1, 3)

    # Finish setting up dear imgui
    dpg.create_viewport(title="Quadrotor Telemetry Viewer")
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.maximize_viewport()
    dpg.set_primary_window("primary", True)

    with dpg.item_handler_registry() as window_hr:
        dpg.add_item_visible_handler(callback=grid)
    dpg.bind_item_handler_registry(window, window_hr)

    timestamp_offset = None
    q = Quaternion(1.0, 0.0, 0.0, 0.0)
    madgwick = Madgwick()
    last_timestamp = None

    while dpg.is_dearpygui_running() and not stop_event.is_set():
        # Read new queue items
        while not telemetry_queue.empty():
            t = telemetry_queue.get()

            # Establish time axis offset if not already established
            new_timestamp = t.timestamp / 1000 # millis -> seconds
            if timestamp_offset is None:
                timestamp_offset = new_timestamp - time.time()

            # Update status window
            dpg.set_value(error_count_label, f"Error count: {t.error_count}")
            dpg.set_value(battery_level_label, f"Battery level: {t.battery_voltage:.2f}V")

            # Update host-side orientation if thats what we're using
            if use_host_orientation:
                if last_timestamp is not None:
                    try:
                        madgwick.Dt = new_timestamp - last_timestamp
                        q = Quaternion(*madgwick.updateMARG(q.as_array(), t.gyro.scale(DEGREES_TO_RADIANS).as_array(),
                                                            t.accel.as_array(), t.mag.as_array()))
                    except ZeroDivisionError:
                        pass
            else:
                q = t.orientation

            # Update orientation plot and rotation visualizer
            try:
                q_axis_mag = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
                theta = 2 * math.atan2(q_axis_mag, q.w)

                rot_visualizer.transform([q.x / q_axis_mag, q.y / q_axis_mag, q.z / q_axis_mag], theta)
            except ZeroDivisionError:
                pass

            orientation_w_series.append_data(new_timestamp, q.w)
            orientation_x_series.append_data(new_timestamp, q.x)
            orientation_y_series.append_data(new_timestamp, q.y)
            orientation_z_series.append_data(new_timestamp, q.z)

            # Update other plots
            accel_x_series.append_data(new_timestamp, t.accel.x / 1000) # mg -> g
            accel_y_series.append_data(new_timestamp, t.accel.y / 1000) # mg -> g
            accel_z_series.append_data(new_timestamp, t.accel.z / 1000) # mg -> g

            gyro_x_series.append_data(new_timestamp, t.gyro.x)
            gyro_y_series.append_data(new_timestamp, t.gyro.y)
            gyro_z_series.append_data(new_timestamp, t.gyro.z)

            mag_x_series.append_data(new_timestamp, t.mag.x)
            mag_y_series.append_data(new_timestamp, t.mag.y)
            mag_z_series.append_data(new_timestamp, t.mag.z)

            pressure_series.append_data(new_timestamp, t.pressure / 1000) # Pa -> kPa

            # Update last timestamp (for host side orientation calcs)
            last_timestamp = new_timestamp

        # Shift timeseries plots X-values
        if timestamp_offset is not None:
            xaxis_end = time.time() + timestamp_offset

            orientation_plot.update(xaxis_end)

            accel_plot.update(xaxis_end)
            gyro_plot.update(xaxis_end)
            mag_plot.update(xaxis_end)
            pressure_plot.update(xaxis_end)

        # Update rotation visualizer clip space
        rot_visualizer.update(dpg.get_item_width(rot_window), dpg.get_item_height(rot_window))

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
    telemetry_read_thread = _TelemetryReadThread(stop_event, telemetry_queue)
    telemetry_read_thread.start()

    # NOTE: To profile, uncomment these lines.
    # import cProfile
    # cProfile.run("run_ui(stop_event, telemetry_queue)", "program.prof")
    try:
        run_ui(stop_event, telemetry_queue, args.use_host_orientation)
    finally:
        stop_event.set()
        telemetry_read_thread.join()
