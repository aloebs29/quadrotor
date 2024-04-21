import asyncio
from collections import deque
import ctypes
from multiprocessing import Event, Process, Queue
import pathlib
import platform

import dearpygui.dearpygui as dpg

from .utils import get_ble_client, Telemetry, TELEMETRY_CHARACTERISTIC

ASSETS_DIR = pathlib.Path(__file__).parents[1] / "assets"


class _TelemetryReadProcess(Process):
    def __init__(self, stop_event, queue, *args, **kwargs):
        super(_TelemetryReadProcess, self).__init__(*args, **kwargs)
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
        asyncio.run(self.run_command())


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
    def __init__(self, height, ylabel, miny, maxy, xrange=10):
        self.series = []
        self.xrange = xrange

        with dpg.plot(height=height, width=-1):
            dpg.add_plot_legend()
            self.xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="Time", time=True, no_tick_labels=True)
            self.yaxis = dpg.add_plot_axis(dpg.mvYAxis, label=ylabel)
            dpg.set_axis_limits(self.yaxis, miny, maxy)

    def add_series(self, label, maxlen=1000):
        series = TimeSeriesData(self.yaxis, label, maxlen)
        self.series.append(series)
        return series
    
    def update(self, timestamp):
        for series in self.series:
            series.update()
        dpg.set_axis_limits(self.xaxis, timestamp - self.xrange, timestamp)


def run():
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
    with dpg.window(tag="primary"):
        error_count_plot = TimeSeriesPlot(100 * scale_factor, "Error count", 0, 100)
        error_count_series = error_count_plot.add_series(None)

        battery_plot = TimeSeriesPlot(300 * scale_factor, "Battery level (V)", 0, 5)
        battery_series = battery_plot.add_series(None)

        accel_plot = TimeSeriesPlot(300 * scale_factor, "Acceleration (g)", -4, 4)
        accel_x_series = accel_plot.add_series("X")
        accel_y_series = accel_plot.add_series("Y")
        accel_z_series = accel_plot.add_series("Y")

        gyro_plot = TimeSeriesPlot(300 * scale_factor, "Angular velocity (dps)", -1000, 1000)
        gyro_x_series = gyro_plot.add_series("X")
        gyro_y_series = gyro_plot.add_series("X")
        gyro_z_series = gyro_plot.add_series("X")

        mag_plot = TimeSeriesPlot(300 * scale_factor, "Magnetic field strength (uT)", -100, 100)
        mag_x_series = mag_plot.add_series("X")
        mag_y_series = mag_plot.add_series("X")
        mag_z_series = mag_plot.add_series("X")

        pressure_plot = TimeSeriesPlot(300 * scale_factor, "Pressure (kPa)", 75, 105)
        pressure_series = pressure_plot.add_series(None)

    # Finish setting up dear imgui
    dpg.create_viewport(title='Updating plot data')
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.maximize_viewport()
    dpg.set_primary_window("primary", True)

    # Setup second process for receiving telemetry notifications over BLE. It may be overkill to use multiprocess here
    # on top of asyncio (which bleak requires), but oh well.
    stop_event = Event()
    telemetry_queue = Queue()
    telemetry_read_proc = _TelemetryReadProcess(stop_event, telemetry_queue)

    telemetry_read_proc.start()

    while dpg.is_dearpygui_running():
        # TODO: Is there a less verbose way of doing this? The series need to be separate lists for the plots..
        new_timestamps = []
        new_error_counts = []
        new_battery_levels = []

        new_accel_x = []
        new_accel_y = []
        new_accel_z = []

        new_gyro_x = []
        new_gyro_y = []
        new_gyro_z = []

        new_mag_x = []
        new_mag_y = []
        new_mag_z = []

        new_pressure = []

        while not telemetry_queue.empty():
            t = telemetry_queue.get()

            new_timestamps.append(t.timestamp / 1000) # millis -> seconds
            new_error_counts.append(t.error_count)
            new_battery_levels.append(t.battery_voltage)

            new_accel_x.append(t.accel.x / 1000) # mg -> g
            new_accel_y.append(t.accel.y / 1000) # mg -> g
            new_accel_z.append(t.accel.z / 1000) # mg -> g

            new_gyro_x.append(t.gyro.x)
            new_gyro_y.append(t.gyro.y)
            new_gyro_z.append(t.gyro.z)

            new_mag_x.append(t.mag.x)
            new_mag_y.append(t.mag.y)
            new_mag_z.append(t.mag.z)

            new_pressure.append(t.pressure / 1000) # Pa -> kPa

        if len(new_timestamps) != 0:
            error_count_series.extend_data(new_timestamps, new_error_counts)
            battery_series.extend_data(new_timestamps, new_battery_levels)

            accel_x_series.extend_data(new_timestamps, new_accel_x)
            accel_y_series.extend_data(new_timestamps, new_accel_y)
            accel_z_series.extend_data(new_timestamps, new_accel_z)

            gyro_x_series.extend_data(new_timestamps, new_gyro_x)
            gyro_y_series.extend_data(new_timestamps, new_gyro_y)
            gyro_z_series.extend_data(new_timestamps, new_gyro_z)

            mag_x_series.extend_data(new_timestamps, new_mag_x)
            mag_y_series.extend_data(new_timestamps, new_mag_y)
            mag_z_series.extend_data(new_timestamps, new_mag_z)

            pressure_series.extend_data(new_timestamps, new_pressure)

            last_timestamp = new_timestamps[-1]

            error_count_plot.update(last_timestamp)
            battery_plot.update(last_timestamp)
            accel_plot.update(last_timestamp)
            gyro_plot.update(last_timestamp)
            mag_plot.update(last_timestamp)
            pressure_plot.update(last_timestamp)

        dpg.render_dearpygui_frame()

    stop_event.set()
    telemetry_read_proc.join()
    dpg.destroy_context()


if __name__ == "__main__":
    run()