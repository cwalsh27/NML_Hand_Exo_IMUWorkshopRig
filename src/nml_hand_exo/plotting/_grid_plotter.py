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

class GridPlotterOld(QtWidgets.QMainWindow):
    def __init__(self, client, fs=1000, buffer_ms=200, channels=64):
        super().__init__()
        self.setWindowTitle("Real-time EMG Grid (8x8)")

        self.client = client
        self.fs = fs
        self.n_channels = channels
        self.n_rows, self.n_cols = 8, 8
        self.buffer_size = int((buffer_ms / 1000.0) * fs)
        self.interval_ms = 30  # update every 30 ms
        self.realtime_filter = None

        self.buffers = [np.zeros(self.buffer_size, dtype=np.float32) for _ in range(self.n_channels)]
        self.x = np.linspace(-buffer_ms / 1000.0, 0, self.buffer_size)

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QtWidgets.QHBoxLayout(self.central_widget)

        self.grid = QtWidgets.QGridLayout()
        self.plots = []
        self.curves = []
        self.heatmap_images = []
        #pg.setConfigOptions(antialias=True)
        for idx in range(self.n_channels):
            row = 7 - (idx // self.n_cols)  # invert vertically
            col = idx % self.n_cols
            plot = pg.PlotWidget()

            # Set only bottom row plots to show time
            if row == 7:
                plot.getAxis('bottom').setLabel("Time (s)", units='s')
                plot.showAxis('bottom')
                #plot.getAxis('bottom').setTickSpacing(0.5, 2)
            else:
                plot.hideAxis('bottom')

            # Set only leftmost column plots to show voltage
            if col == 0:
                plot.getAxis('left').setLabel("uV", units='V')
                plot.showAxis('left')
                plot.getAxis('left').setTickSpacing(100, 2)
            else:
                plot.hideAxis('left')

            plot.setBackground("w")
            plot.setYRange(-200, 200)
            plot.setMouseEnabled(x=False, y=False)
            curve = plot.plot(self.x, self.buffers[idx], pen=pg.mkPen('k', width=1))

            # === NEW ====
            heatmap = pg.ImageItem()
            plot.addItem(heatmap)
            heatmap.hide()
            # ============

            plot.setTitle(f"{idx}", color='gray', size="8pt")
            self.grid.addWidget(plot, row, col)
            self.plots.append(plot)
            self.curves.append(curve)
            self.heatmap_images.append(heatmap)

        self.main_layout.addLayout(self.grid)

        # Control panel
        self.control_panel = QtWidgets.QVBoxLayout()
        self.car_checkbox = QtWidgets.QCheckBox("Enable CAR")
        self.filter_checkbox = QtWidgets.QCheckBox("Enable Bandpass Filter")
        self.notch_checkbox = QtWidgets.QCheckBox("Enable Notch")
        self.low_cut_input = QtWidgets.QLineEdit("10")
        self.high_cut_input = QtWidgets.QLineEdit("500")
        self.low_cut_input.setValidator(QDoubleValidator(0.0, fs / 2, 2))
        self.high_cut_input.setValidator(QDoubleValidator(0.0, fs / 2, 2))

        # Plot Mode dropdown
        self.color_map = ColorMap([0.0, 0.5, 1.0], [(0, 0, 0), (255, 0, 0), (255, 255, 0)])
        self.mode_selector = QtWidgets.QComboBox()
        self.mode_selector.addItems(["Waveform", "Heatmap"])

        self.control_panel.addWidget(QtWidgets.QLabel("Plot Mode:"))
        self.control_panel.addWidget(self.mode_selector)
        self.control_panel.addWidget(self.car_checkbox)
        self.control_panel.addWidget(self.filter_checkbox)
        self.control_panel.addWidget(self.notch_checkbox)
        self.control_panel.addWidget(QtWidgets.QLabel("Bandpass Range (Hz):"))
        self.control_panel.addWidget(QtWidgets.QLabel("Low Cut"))
        self.control_panel.addWidget(self.low_cut_input)
        self.control_panel.addWidget(QtWidgets.QLabel("High Cut"))
        self.control_panel.addWidget(self.high_cut_input)
        self.control_panel.addStretch()
        self.main_layout.addLayout(self.control_panel)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

    def start(self):
        self.client.start_streaming()
        self.timer.start(self.interval_ms)
        self.show()

    def update_plot(self):
        data = self.client.get_latest_window(200)  # shape: (n_channels, n_samples)

        if self.car_checkbox.isChecked():
            data -= np.mean(data, axis=0, keepdims=True)

        if self.realtime_filter is None:
            self._update_filter()

        if self.realtime_filter:
            self.realtime_filter.toggle_bandpass(self.filter_checkbox.isChecked())
            self.realtime_filter.toggle_notch(self.notch_checkbox.isChecked())
            data = self.realtime_filter.update(data)

        plot_mode = self.mode_selector.currentText()
        rms_grid = np.zeros((8, 8))

        # for ch in range(min(self.n_channels, data.shape[0])):
        #     self.buffers[ch] = np.roll(self.buffers[ch], -data.shape[1])
        #     self.buffers[ch][-data.shape[1]:] = data[ch]
        #
        #     if self.mode_selector.currentText() == "RMS":
        #         signal_to_plot = compute_rolling_rms(self.buffers[ch], window_size=int(0.05 * self.fs))  # 50ms window
        #     else:
        #         signal_to_plot = self.buffers[ch]
        #
        #     self.curves[ch].setData(self.x, signal_to_plot)
        for ch in range(min(self.n_channels, data.shape[0])):
            self.buffers[ch] = np.roll(self.buffers[ch], -data.shape[1])
            self.buffers[ch][-data.shape[1]:] = data[ch]

            row, col = 7 - (ch // self.n_cols), ch % self.n_cols

            if plot_mode == "Waveform":
                self.curves[ch].setData(self.x, self.buffers[ch])
                self.curves[ch].show()
                self.heatmap_images[ch].hide()
            elif plot_mode == "Heatmap":
                rms = compute_rolling_rms(self.buffers[ch], window_size=int(0.05 * self.fs))
                val = rms[-1] if len(rms) > 0 else 0.0
                rms_grid[row, col] = val
                self.curves[ch].hide()
                self.heatmap_images[ch].show()

        if plot_mode == "Heatmap":
            normed = (rms_grid - rms_grid.min()) / (np.ptp(rms_grid) + 1e-6)
            colored = self.color_map.map(normed.flatten(), mode='qcolor')
            for ch in range(self.n_channels):
                img = np.full((10, 10), normed.flatten()[ch])
                self.heatmap_images[ch].setImage(img, autoLevels=False)

    def _update_filter(self):
        try:
            low = float(self.low_cut_input.text())
            high = float(self.high_cut_input.text())
            self.realtime_filter = RealtimeEMGFilter(fs=self.fs, n_channels=self.n_channels,
                                                     lowcut=low, highcut=high,
                                                     enable_bandpass=self.filter_checkbox.isChecked(),
                                                     enable_notch=self.notch_checkbox.isChecked())
        except Exception as e:
            print(f"[Filter Update Error] {e}")

    def closeEvent(self, event):
        self.timer.stop()
        self.client.stop_streaming()
        event.accept()


class GridPlotter(QtWidgets.QMainWindow):
    def __init__(self, client, fs=1000, buffer_ms=200, channels=64):
        super().__init__()
        self.setWindowTitle("Real-time EMG Grid (8x8)")

        self.client = client
        self.fs = fs
        self.n_channels = channels
        self.n_rows, self.n_cols = 8, 8
        self.buffer_size = int((buffer_ms / 1000.0) * fs)
        self.interval_ms = 30  # update every 30 ms
        self.rms_max = 200  # Default max RMS for heatmap
        self.realtime_filter = None
        self.buffers = [np.zeros(self.buffer_size, dtype=np.float32) for _ in range(self.n_channels)]
        self.x = np.linspace(-buffer_ms / 1000.0, 0, self.buffer_size)

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QtWidgets.QHBoxLayout(self.central_widget)

        # Grid layout for waveform plots
        self.grid = QtWidgets.QGridLayout()
        self.plots = []
        self.curves = []
        for idx in range(self.n_channels):
            row = 7 - (idx // self.n_cols)
            col = idx % self.n_cols
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
            curve = plot.plot(self.x, self.buffers[idx], pen=pg.mkPen('k', width=1))
            plot.setTitle(f"{idx}", color='gray', size="8pt")
            self.grid.addWidget(plot, row, col)
            self.plots.append(plot)
            self.curves.append(curve)

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
        self.filter_checkbox = QtWidgets.QCheckBox("Enable Bandpass Filter")
        self.notch_checkbox = QtWidgets.QCheckBox("Enable Notch")
        self.low_cut_input = QtWidgets.QLineEdit("10")
        self.high_cut_input = QtWidgets.QLineEdit("500")
        self.low_cut_input.setValidator(QDoubleValidator(0.0, fs / 2, 2))
        self.high_cut_input.setValidator(QDoubleValidator(0.0, fs / 2, 2))
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
        self.control_panel.addWidget(self.filter_checkbox)
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
        self.control_panel.addStretch()
        self.main_layout.addLayout(self.control_panel)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

    def start(self):
        self.client.start_streaming()
        self.timer.start(self.interval_ms)
        self.show()

    def update_plot(self):
        data = self.client.get_latest_window(200)

        if self.car_checkbox.isChecked():
            data -= np.mean(data, axis=0, keepdims=True)

        if self.realtime_filter is None:
            self._update_filter()

        if self.realtime_filter:
            self.realtime_filter.toggle_bandpass(self.filter_checkbox.isChecked())
            self.realtime_filter.toggle_notch(self.notch_checkbox.isChecked())
            data = self.realtime_filter.update(data)

        for ch in range(min(self.n_channels, data.shape[0])):
            self.buffers[ch] = np.roll(self.buffers[ch], -data.shape[1])
            self.buffers[ch][-data.shape[1]:] = data[ch]

        mode = self.mode_selector.currentText()

        if mode == "Waveform":
            self.heatmap_widget.hide()
            for ch in range(self.n_channels):
                self.plots[ch].show()
                signal = self.buffers[ch]
                self.curves[ch].setData(self.x, signal)
        elif mode == "Heatmap":
            self.update_colormap()
            for ch in range(self.n_channels):
                self.plots[ch].hide()
            self.heatmap_widget.show()

            rms_values = [compute_rolling_rms(self.buffers[ch], window_size=self.buffer_size)[-1] for ch in range(self.n_channels)]
            heatmap = np.reshape(rms_values, (8, 8))
            heatmap = np.flipud(heatmap)

            self.heatmap_img.setImage(heatmap.astype(np.float32), levels=(0, self.rms_max), autoLevels=False, autoDownsample=True)

    def _update_filter(self):
        try:
            low = float(self.low_cut_input.text())
            high = float(self.high_cut_input.text())
            self.realtime_filter = RealtimeEMGFilter(fs=self.fs, n_channels=self.n_channels,
                                                     lowcut=low, highcut=high,
                                                     enable_bandpass=self.filter_checkbox.isChecked(),
                                                     enable_notch=self.notch_checkbox.isChecked())
        except Exception as e:
            print(f"[Filter Update Error] {e}")

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




