from PyQt5 import QtWidgets
import sys
import argparse
from nml_hand_exo.interface import LSLClient
from nml_hand_exo.plotting import PyQtGraphRealtimePlotter

if __name__ == "__main__":

    # Create argument parser for command line options
    parser = argparse.ArgumentParser()
    parser.add_argument("--channels", type=int, nargs='+', default=[0, 1, 2, 3], help="Channels to plot")
    args = parser.parse_args()

    # Initialize the Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create LSL client and PyQtGraph plotter
    client = LSLClient(stream_type="EMG")

    # Create the RealtimePlotter instance
    plotter = PyQtGraphRealtimePlotter(
        client=client,
        channels_to_plot=args.channels,
        sampling_rate=client.fs,
    )

    # Begin the plotter and start listening to teh stream
    plotter.start()
    print("Starting LSL real-time plotting...")

    # Exit when done
    sys.exit(app.exec_())
