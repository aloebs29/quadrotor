from collections import deque

import dearpygui.dearpygui as dpg


class TimeSeriesData:
    def __init__(self, parent, label, maxlen):
        self.timestamps = deque(maxlen=maxlen)
        self.data = deque(maxlen=maxlen)
        self.series = dpg.add_line_series(list(self.timestamps), list(self.data), parent=parent, label=label)

    def append_data(self, new_timestamp, new_data):
        self.timestamps.append(new_timestamp)
        self.data.append(new_data)

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
