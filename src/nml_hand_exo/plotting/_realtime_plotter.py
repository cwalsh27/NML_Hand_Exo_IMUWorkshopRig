import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtGui import QDoubleValidator

from nml_hand_exo.processing import RealtimeEMGFilter


class StackedPlotter(QtWidgets.QMainWindow):
    def __init__(self, client, channels_to_plot=None, interval_ms=30, buffer_secs=2, downsample_factor=1,
                 enable_car=False, enable_bandpass=False, enable_notch=False):
        super().__init__()
        self.setWindowTitle("Real-time Stacked-View EMG Plotter")

        self.client = client
        self.sampling_rate = client.sampling_rate
        self.n_channels_total = client.n_channels
        self.interval_ms = interval_ms
        self.buffer_secs = buffer_secs
        self.downsample_factor = downsample_factor
        self.enable_car = enable_car
        self.enable_bandpass = enable_bandpass
        self.enable_notch = enable_notch
        self.channel_height = 150
        self.broken_ch_std_threshold = 300  # std

        # Channel setup
        if channels_to_plot == "all":
            self.channels_to_plot = list(range(self.n_channels_total))
        else:
            self.channels_to_plot = channels_to_plot or [0]

        self.n_channels = len(self.channels_to_plot)
        self.buffer_size = int(buffer_secs * self.sampling_rate // self.downsample_factor)
        self.buffers = [np.zeros(self.buffer_size, dtype=np.float32) for _ in range(self.n_channels)]
        self.x = np.linspace(-buffer_secs, 0, self.buffer_size)

        # Initialize filter
        self.emg_filter = RealtimeEMGFilter(
            sampling_rate=self.sampling_rate,
            n_channels=self.n_channels,
            enable_car=enable_car,
            enable_bandpass=enable_bandpass,
            enable_notch=enable_notch,
        )

        self.init_ui()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

    def init_ui(self):
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        # Scrollable plot layout
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        plot_container = QtWidgets.QWidget()
        self.plot_layout = QtWidgets.QVBoxLayout(plot_container)
        scroll_area.setWidget(plot_container)
        main_layout.addWidget(scroll_area, stretch=5)

        # Curves and plots
        self.curves = []
        self.plot_widgets = []
        for i in range(self.n_channels):
            pw = pg.PlotWidget()
            pw.setMaximumHeight(self.channel_height)
            pw.setYRange(-200, 200)
            pw.setMouseEnabled(x=False, y=False)
            pw.setMenuEnabled(False)
            pw.showGrid(x=True, y=True)
            if i < self.n_channels - 1:
                pw.hideAxis('bottom')
            pw.getPlotItem().getAxis('left').setWidth(50)  # Allocate label width
            pw.setLabel('left', f'Ch {self.channels_to_plot[i]} (μV)', **{'color': '#000', 'font-size': '9pt'})
            curve = pw.plot(pen=pg.mkPen(color=pg.intColor(i), width=1))
            self.curves.append(curve)
            self.plot_widgets.append(pw)
            self.plot_layout.addWidget(pw)

        # Control panel
        self.ctrl_panel = QtWidgets.QGroupBox("Controls")
        ctrl_layout = QtWidgets.QVBoxLayout()

        self.car_checkbox = QtWidgets.QCheckBox("CAR Filter")
        self.bandpass_checkbox = QtWidgets.QCheckBox("Bandpass Filter")
        self.notch_checkbox = QtWidgets.QCheckBox("Notch Filter")

        ctrl_layout.addWidget(self.car_checkbox)
        ctrl_layout.addWidget(self.bandpass_checkbox)
        ctrl_layout.addWidget(self.notch_checkbox)

        self.low_cut_input = QtWidgets.QLineEdit("10")
        self.high_cut_input = QtWidgets.QLineEdit("500")
        for box in [self.low_cut_input, self.high_cut_input]:
            box.setValidator(QDoubleValidator(0.0, self.sampling_rate / 2, 2))

        ctrl_layout.addWidget(QtWidgets.QLabel("Bandpass Low Cut (Hz):"))
        ctrl_layout.addWidget(self.low_cut_input)
        ctrl_layout.addWidget(QtWidgets.QLabel("Bandpass High Cut (Hz):"))
        ctrl_layout.addWidget(self.high_cut_input)

        # Connect filter toggles
        self.car_checkbox.stateChanged.connect(self.update_filter_settings)
        self.bandpass_checkbox.stateChanged.connect(self.update_filter_settings)
        self.notch_checkbox.stateChanged.connect(self.update_filter_settings)
        self.low_cut_input.textChanged.connect(self.update_filter_settings)
        self.high_cut_input.textChanged.connect(self.update_filter_settings)

        ctrl_layout.addStretch(1)
        self.ctrl_panel.setLayout(ctrl_layout)
        main_layout.addWidget(self.ctrl_panel, stretch=1)

        self.resize(900, 600)

    def update_filter_settings(self):
        try:
            low = float(self.low_cut_input.text())
            high = float(self.high_cut_input.text())
            self.emg_filter.set_bandpass(low, high)
        except ValueError:
            pass

        self.emg_filter.toggle_car(self.car_checkbox.isChecked())
        self.emg_filter.toggle_notch(self.notch_checkbox.isChecked())
        self.emg_filter.toggle_bandpass(self.bandpass_checkbox.isChecked())

    def start(self):
        self.client.start_streaming()
        self.timer.start(self.interval_ms)
        self.show()

    def update_plot(self):
        window = self.client.get_latest_window(window_ms=self.interval_ms)
        if window is None or window.shape[1] == 0:
            return

        # Extract selected channels and filter
        data = window[self.channels_to_plot]
        filtered = self.emg_filter.update(data)

        for i, ch in enumerate(self.channels_to_plot):
            new_data = filtered[i][::self.downsample_factor]
            if new_data.shape[0] > 0:
                self.buffers[i] = np.roll(self.buffers[i], -len(new_data))
                self.buffers[i][-len(new_data):] = new_data
                self.curves[i].setData(self.x, self.buffers[i])

    def closeEvent(self, event):
        self.client.stop_streaming()
        event.accept()
