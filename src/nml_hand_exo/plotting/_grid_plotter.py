import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QDoubleValidator
import pyqtgraph as pg
from pyqtgraph import ColorMap

from nml_hand_exo.processing import RealtimeEMGFilter, compute_rolling_rms


def reversed_colormap(colors):
    """Reverse a list of RGB tuples and corresponding positions."""
    positions = np.linspace(0, 1, len(colors))
    return ColorMap(pos=positions[::-1], color=colors[::-1])

COLORMAPS = {
    "viridis": reversed_colormap([
        (68, 1, 84), (59, 82, 139), (33, 145, 140),
        (94, 201, 98), (253, 231, 37), (255, 255, 255)
    ]),
    "plasma": reversed_colormap([
        (13, 8, 135), (126, 3, 168), (203, 71, 119),
        (248, 149, 64), (251, 246, 85), (255, 255, 255)
    ]),
    "hot": reversed_colormap([
        (0, 0, 0), (255, 0, 0), (255, 255, 0), (255, 255, 255)
    ]),
    "jet": reversed_colormap([
        (0, 0, 143), (0, 255, 255), (255, 255, 0), (255, 0, 0), (128, 0, 0)
    ])
}


class GridPlotter(QtWidgets.QMainWindow):
    def __init__(self, client, channels_to_plot=None, interval_ms=30, buffer_ms=200, downsample_factor=1,
                 enable_car=False, enable_bandpass=False, enable_notch=False, verbose=False):
        super().__init__()
        self.setWindowTitle("Real-time EMG Grid (8x8)")
        self.client = client
        self.sampling_rate = client.sampling_rate if hasattr(client, 'sampling_rate') else 1000
        self.downsample_factor = downsample_factor
        self.n_channels = client.n_channels
        self.interval_ms = interval_ms
        self.buffer_ms = buffer_ms
        self.enable_car = enable_car
        self.enable_bandpass = enable_bandpass
        self.enable_notch = enable_notch
        self.verbose = verbose

        if channels_to_plot == 'all':
            self.channels_to_plot = list(range(min(self.n_channels, 64)))
        else:
            if self.verbose:
                print(f"[GridPlotter] Channels to plot: {channels_to_plot}")
            self.channels_to_plot = channels_to_plot or [0, 1, 2, 3]

        self.rms_max = 200  # Default max RMS for heatmap
        self.n_rows, self.n_cols = 8, 8
        self.buffer_size = int((buffer_ms / 1000.0) * self.sampling_rate)
        self.realtime_filter = None
        self.buffers = [np.zeros(self.buffer_size, dtype=np.float32) for _ in range(self.n_channels)]
        self.x = np.linspace(-buffer_ms / 1000.0, 0, self.buffer_size)

        # Initialize filter
        self.emg_filter = RealtimeEMGFilter(
            sampling_rate=self.sampling_rate,
            n_channels=self.n_channels,
            enable_car=enable_car,
            enable_bandpass=enable_bandpass,
            enable_notch=enable_notch,
            verbose=self.verbose,
        )

        self.init_ui()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

    def init_ui(self):
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QtWidgets.QHBoxLayout(self.central_widget)

        # Grid layout for waveform plots
        self.grid = QtWidgets.QGridLayout()
        self.plots = []
        self.curves = []
        for i, idx in enumerate(self.channels_to_plot):
            row = 7 - (i // self.n_cols)
            col = i % self.n_cols
            plot = pg.PlotWidget()

            if row == 7:
                plot.getAxis('bottom').setLabel("Time (s)", units='s')
                plot.showAxis('bottom')
            else:
                plot.hideAxis('bottom')

            if col == 0:
                plot.getAxis('left').setLabel("uV", units='V')
                plot.showAxis('left')
            else:
                plot.hideAxis('left')

            plot.setBackground("w")
            plot.setYRange(-200, 200)
            plot.setMouseEnabled(x=False, y=False)
            curve = plot.plot(self.x, self.buffers[i], pen=pg.mkPen('k', width=1))
            plot.setTitle(f"{idx}", color='gray', size="8pt")
            self.grid.addWidget(plot, row, col)
            self.plots.append(plot)
            self.curves.append(curve)

        print(f"# plots created: {len(self.plots)}")

        self.main_layout.addLayout(self.grid)

        # RMS heatmap setup
        self.heatmap_widget = pg.GraphicsLayoutWidget()
        self.heatmap_plot = self.heatmap_widget.addPlot()
        self.heatmap_img = pg.ImageItem()
        self.heatmap_plot.addItem(self.heatmap_img)
        self.heatmap_plot.setTitle("RMS Heatmap")
        self.heatmap_plot.hideAxis('bottom')
        self.heatmap_plot.hideAxis('left')
        self.main_layout.addWidget(self.heatmap_widget)
        self.heatmap_widget.hide()

        # Colorbar for heatmap
        self.colorbar_plot = pg.PlotWidget()
        self.colorbar_plot.setFixedWidth(80)
        self.colorbar_plot.setMaximumWidth(80)
        self.colorbar_plot.setMouseEnabled(x=False, y=False)
        self.colorbar_plot.hideAxis('bottom')
        self.colorbar_plot.setLabel('left', 'RMS', units='uV')

        self.colorbar_img = pg.ImageItem(axisOrder='row-major')
        self.colorbar_plot.addItem(self.colorbar_img)
        self.main_layout.addWidget(self.colorbar_plot)

        self.colorbar_gradient = np.linspace(0, 1, 256).reshape(256, 1)

        # Control panel
        self.control_panel = QtWidgets.QVBoxLayout()

        self.car_checkbox = QtWidgets.QCheckBox("Enable CAR")
        self.bandpass_checkbox = QtWidgets.QCheckBox("Enable Bandpass Filter")
        self.notch_checkbox = QtWidgets.QCheckBox("Enable Notch")
        self.low_cut_input = QtWidgets.QLineEdit("10")
        self.high_cut_input = QtWidgets.QLineEdit("500")
        self.low_cut_input.setValidator(QDoubleValidator(0.0, self.sampling_rate / 2, 2))
        self.high_cut_input.setValidator(QDoubleValidator(0.0, self.sampling_rate / 2, 2))
        self.colormap_selector = QtWidgets.QComboBox()
        self.colormap_selector.addItems(list(COLORMAPS.keys()))
        self.rms_max_input = QtWidgets.QLineEdit("200")
        self.rms_max_input.editingFinished.connect(self.update_heatmap_levels)
        self.rms_max_input.returnPressed.connect(self.update_heatmap_levels)

        self.mode_selector = QtWidgets.QComboBox()
        self.mode_selector.addItems(["Waveform", "Heatmap"])
        self.control_panel.addWidget(QtWidgets.QLabel("Plot Mode:"))
        self.control_panel.addWidget(self.mode_selector)
        self.control_panel.addWidget(self.car_checkbox)
        self.control_panel.addWidget(self.bandpass_checkbox)
        self.control_panel.addWidget(self.notch_checkbox)
        self.control_panel.addWidget(QtWidgets.QLabel("Bandpass Range (Hz):"))
        self.control_panel.addWidget(QtWidgets.QLabel("Low Cut"))
        self.control_panel.addWidget(self.low_cut_input)
        self.control_panel.addWidget(QtWidgets.QLabel("High Cut"))
        self.control_panel.addWidget(self.high_cut_input)
        self.control_panel.addWidget(QtWidgets.QLabel("Colormap:"))
        self.control_panel.addWidget(self.colormap_selector)
        self.control_panel.addWidget(QtWidgets.QLabel("Heatmap Max RMS:"))
        self.control_panel.addWidget(self.rms_max_input)

        # Connect toggles
        self.car_checkbox.stateChanged.connect(self.update_filter_settings)
        self.bandpass_checkbox.stateChanged.connect(self.update_filter_settings)
        self.notch_checkbox.stateChanged.connect(self.update_filter_settings)
        self.low_cut_input.textChanged.connect(self.update_filter_settings)
        self.high_cut_input.textChanged.connect(self.update_filter_settings)

        self.control_panel.addStretch()
        self.main_layout.addLayout(self.control_panel)

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
        # TO-DO: allow for channel selection
        data = window[range(self.n_channels), :]
        data = self.emg_filter.update(data)

        for i, ch in enumerate(self.channels_to_plot):
            new_data = data[i][::self.downsample_factor]
            if new_data.shape[0] > 0:
                self.buffers[i] = np.roll(self.buffers[i], -len(new_data))
                self.buffers[i][-len(new_data):] = new_data

        mode = self.mode_selector.currentText()

        if mode == "Waveform":
            self.heatmap_widget.hide()
            for ch in range(len(self.channels_to_plot)):
                self.plots[ch].show()
                signal = self.buffers[ch]
                self.curves[ch].setData(self.x, signal)
        elif mode == "Heatmap":
            self.update_colormap()
            for ch in range(len(self.channels_to_plot)):
                self.plots[ch].hide()
            self.heatmap_widget.show()

            rms_values = [compute_rolling_rms(self.buffers[ch], window_size=self.buffer_size)[-1] for ch in range(len(self.channels_to_plot))]
            heatmap = np.reshape(rms_values, (8, 8))
            heatmap = np.flipud(heatmap)
            self.heatmap_img.setImage(heatmap.astype(np.float32), levels=(0, self.rms_max), autoLevels=False, autoDownsample=True)

    def closeEvent(self, event):
        self.timer.stop()
        self.client.stop_streaming()
        event.accept()

    def update_colormap(self):
        cmap = COLORMAPS[self.colormap_selector.currentText()]
        lut = cmap.getLookupTable(0.0, 1.0, 256)

        max_rms = float(self.rms_max_input.text())
        self.heatmap_img.setLookupTable(lut)
        self.heatmap_img.setLevels([0, max_rms])

        self.colorbar_img.setImage(self.colorbar_gradient)
        self.colorbar_img.setLookupTable(lut)
        self.colorbar_img.setLevels([0, 1])

    def update_heatmap_levels(self):
        try:
            new_limit = float(self.rms_max_input.text())
            self.rms_max = new_limit
            #print(f"[Heatmap] RMS upper limit set to {self.rms_max}")
        except ValueError:
            print("[Heatmap] Invalid RMS max value. Please enter a number.")




