import asyncio
from collections import deque
import ctypes
import pathlib
import platform
from queue import Queue
from threading import Event, Thread
import time

import dearpygui.dearpygui as dpg
import dearpygui_grid as dpg_grid

from .utils import get_ble_client, Telemetry, TELEMETRY_CHARACTERISTIC

ASSETS_DIR = pathlib.Path(__file__).parents[1] / "assets"


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


class TimeSeriesData:
    def __init__(self, parent, label, maxlen):
        self.timestamps = deque(maxlen=maxlen)
        self.data = deque(maxlen=maxlen)
        self.series = dpg.add_line_series(list(self.timestamps), list(self.data), parent=parent, label=label)

    def extend_data(self, new_timestamps, new_data):
        self.timestamps.extend(new_timestamps)
        self.data.extend(new_data)

    def update(self):
        dpg.configure_item(self.series, x=list(self.timestamps), y=list(self.data))


class TimeSeriesPlot:
    def __init__(self, label, ylabel, miny, maxy, time_axis_range=10):
        self.series = []
        self.time_axis_range = time_axis_range

        with dpg.plot(label=label) as plot:
            self.plot = plot
            dpg.add_plot_legend()
            self.xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="Time", time=True, no_tick_labels=True)
            self.yaxis = dpg.add_plot_axis(dpg.mvYAxis, label=ylabel)
            dpg.set_axis_limits(self.yaxis, miny, maxy)

    def add_series(self, label, maxlen=1000):
        series = TimeSeriesData(self.yaxis, label, maxlen)
        self.series.append(series)
        return series
    
    def update(self, time_axis_end):
        for series in self.series:
            series.update()
        dpg.set_axis_limits(self.xaxis, time_axis_end - self.time_axis_range, time_axis_end)


def run_ui(stop_event, telemetry_queue):
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

    # Setup plots
    with dpg.window(tag="primary") as window:
        grid = dpg_grid.Grid(2, 3, window)

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

        error_count_plot = TimeSeriesPlot("Error Count", "Count", 0, 100)
        error_count_series = error_count_plot.add_series(None)

        battery_plot = TimeSeriesPlot("Battery Level Data", "Battery level (V)", 0, 5)
        battery_series = battery_plot.add_series(None)

        pressure_plot = TimeSeriesPlot("Pressure Sensor Data", "Pressure (kPa)", 75, 105)
        pressure_series = pressure_plot.add_series(None)

        grid.push(accel_plot.plot, 0, 0)
        grid.push(gyro_plot.plot, 0, 1)
        grid.push(mag_plot.plot, 0, 2)
        
        grid.push(error_count_plot.plot, 1, 0)
        grid.push(battery_plot.plot, 1, 1)
        grid.push(pressure_plot.plot, 1, 2)

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
    while dpg.is_dearpygui_running() and not stop_event.is_set():
        # TODO: Is there a less verbose way of doing this? The series need to be separate lists for the plots..
        new_timestamps = []

        new_accel_x = []
        new_accel_y = []
        new_accel_z = []

        new_gyro_x = []
        new_gyro_y = []
        new_gyro_z = []

        new_mag_x = []
        new_mag_y = []
        new_mag_z = []

        new_error_counts = []
        new_battery_levels = []
        new_pressure = []

        while not telemetry_queue.empty():
            t = telemetry_queue.get()

            new_timestamps.append(t.timestamp / 1000) # millis -> seconds

            new_accel_x.append(t.accel.x / 1000) # mg -> g
            new_accel_y.append(t.accel.y / 1000) # mg -> g
            new_accel_z.append(t.accel.z / 1000) # mg -> g

            new_gyro_x.append(t.gyro.x)
            new_gyro_y.append(t.gyro.y)
            new_gyro_z.append(t.gyro.z)

            new_mag_x.append(t.mag.x)
            new_mag_y.append(t.mag.y)
            new_mag_z.append(t.mag.z)

            new_error_counts.append(t.error_count)
            new_battery_levels.append(t.battery_voltage)
            new_pressure.append(t.pressure / 1000) # Pa -> kPa

        if len(new_timestamps) != 0:
            # Establish time axis offset
            if timestamp_offset is None:
                timestamp_offset = new_timestamps[0] - time.time()

            accel_x_series.extend_data(new_timestamps, new_accel_x)
            accel_y_series.extend_data(new_timestamps, new_accel_y)
            accel_z_series.extend_data(new_timestamps, new_accel_z)

            gyro_x_series.extend_data(new_timestamps, new_gyro_x)
            gyro_y_series.extend_data(new_timestamps, new_gyro_y)
            gyro_z_series.extend_data(new_timestamps, new_gyro_z)

            mag_x_series.extend_data(new_timestamps, new_mag_x)
            mag_y_series.extend_data(new_timestamps, new_mag_y)
            mag_z_series.extend_data(new_timestamps, new_mag_z)

            error_count_series.extend_data(new_timestamps, new_error_counts)
            battery_series.extend_data(new_timestamps, new_battery_levels)
            pressure_series.extend_data(new_timestamps, new_pressure)

        if timestamp_offset is not None:
            xaxis_end = time.time() + timestamp_offset
            accel_plot.update(xaxis_end)
            gyro_plot.update(xaxis_end)
            mag_plot.update(xaxis_end)

            error_count_plot.update(xaxis_end)
            battery_plot.update(xaxis_end)
            pressure_plot.update(xaxis_end)

        dpg.render_dearpygui_frame()

    dpg.destroy_context()


if __name__ == "__main__":
    # Setup second thread for receiving telemetry notifications over BLE.
    stop_event = Event()
    telemetry_queue = Queue()
    telemetry_read_thread = _TelemetryReadThread(stop_event, telemetry_queue)
    telemetry_read_thread.start()

    # NOTE: To profile, uncomment these lines.
    # import cProfile
    # cProfile.run("run_ui(stop_event, telemetry_queue)", "program.prof")
    run_ui(stop_event, telemetry_queue)

    stop_event.set()
    telemetry_read_thread.join()
