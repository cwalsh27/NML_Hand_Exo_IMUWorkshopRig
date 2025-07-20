import time
import numpy as np
from nml_hand_exo.processing import compute_rms, compute_rolling_rms


class RMSGestureTrigger:
    """
    A simple controller that monitors real-time EMG RMS signals and sends gesture commands
    to an exoskeleton based on channel group thresholds.

    Attributes:
        exo: Instance of HandExo.
        client: LSLClient or compatible data stream reader.
        fs: Sampling frequency.
        threshold_percect: Threshold as a percentage above rest mean RMS.
        window_ms: Duration of EMG window for RMS calculation.
        rest_duration_sec: Time to collect baseline (rest) RMS signal.
    """

    def __init__(self, exo, client, fs=1000, threshold_percent=0.1, window_ms=200, rest_duration_sec=5):
        self.exo = exo
        self.client = client
        self.fs = fs
        self.threshold_percent = threshold_percent
        self.window_ms = window_ms
        self.rest_duration_sec = rest_duration_sec

        self.upper_mean_rms = None
        self.lower_mean_rms = None
        self.last_state = None

    def calibrate_rest_state(self):
        """Calibrates the resting EMG signal to compute channel-wise standard deviation."""
        print("[Calibration] Collecting rest-state RMS for threshold estimation...")
        all_rms = []
        start_time = time.monotonic()
        while time.monotonic() - start_time < self.rest_duration_sec:
            window = self.client.get_latest_window(self.window_ms)
            if window is None or len(window) == 0:
                print("[Calibration] No data received. Ensure the client is streaming data.")
                time.sleep(self.window_ms / 1000.0)
                continue
            rms = compute_rms(window)
            all_rms.append(rms)
            time.sleep(self.window_ms / 1000.0)

        all_rms = np.stack(all_rms)  # shape: (n_windows, n_channels)
        mean_rms = np.mean(all_rms, axis=0)
        self.upper_mean_rms = np.mean(mean_rms[:64])
        self.lower_mean_rms = np.mean(mean_rms[64:128])

        print(f"[Calibration] Finished.")
        print(f"  → Mean upper RMS: {self.upper_mean_rms:.2f} uV")
        print(f"  → Mean lower RMS: {self.lower_mean_rms:.2f} uV")
    def start(self):
        """Begins monitoring the EMG stream and sends gesture commands when thresholds are crossed."""
        if self.upper_mean_rms is None or self.lower_mean_rms is None:
            raise RuntimeError("Must calibrate rest state before starting.")
        print("[GestureTrigger] Starting gesture monitoring loop.")

        upper_thresh = self.upper_mean_rms * (1 + self.threshold_percent)
        lower_thresh = self.lower_mean_rms * (1 + self.threshold_percent)
        if not self.client.streaming:
            self.client.start_streaming()
        try:
            while True:
                time.sleep(self.window_ms / 1000.0)
                window = self.client.get_latest_window(self.window_ms)
                rms = compute_rms(window)

                upper_rms_avg = np.mean(rms[:64])
                lower_rms_avg = np.mean(rms[64:128])

                if upper_rms_avg > upper_thresh:
                    if self.last_state != "open":
                        print(f"[Trigger] Upper RMS triggered OPEN ({upper_rms_avg:.2f} > {upper_thresh:.2f})")
                        self.exo.set_gesture_state("open")
                        self.last_state = "open"

                elif lower_rms_avg > lower_thresh:
                    if self.last_state != "close":
                        print(f"[Trigger] Lower RMS triggered CLOSE ({lower_rms_avg:.2f} > {lower_thresh:.2f})")
                        self.exo.set_gesture_state("close")
                        self.last_state = "close"

                else:
                    if self.last_state != "rest":
                        print(f"[Trigger] No gesture detected, resting state.")
                        self.exo.set_gesture_state("rest")
                        self.last_state = "rest"

                time.sleep(self.window_ms / 1000.0)  # Wait for the next window

        except KeyboardInterrupt:
            print("\n[GestureTrigger] Stopped by user.")

        finally:
            self.client.stop_streaming()
            self.exo.close()
