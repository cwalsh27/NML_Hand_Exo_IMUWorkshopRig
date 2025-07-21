import numpy as np
from scipy.signal import hilbert


# Deflationary orthogonality
def orthogonalize(W, wp, i):
    """
    Orthogonalizes the weight vector wp with respect to the first i columns of W.

    Parameters:
        W: Weight matrix of shape (n_features, n_features).
        wp: Weight vector to be orthogonalized of shape (n_features,).
        i: Index of the column in W to orthogonalize against.

    Returns:
        wp: Orthogonalized weight vector of shape (n_features,).
    """
    return wp - ((wp @ W[:i, :].T) @ W[:i, :])


# wp normalization
def normalize(wp):
    """
    Normalizes the weight vector wp.

    Parameters:
        wp: Weight vector to be normalized of shape (n_features,).

    Returns:
        wp: Normalized weight vector of shape (n_features,).
    """
    return wp / np.linalg.norm(wp)


def rectify(emg_data):
    """
    Rectifies EMG data by converting all values to their absolute values.

    Parameters:
        emg_data (numpy array): List of numpy arrays or pandas DataFrame items with filtered EMG data.

    Returns:
        rectified_data: List of rectified numpy arrays (same shape as input data).
    """
    return np.abs(emg_data)


def window_rms(emg_data, window_size=400, verbose=False):
    """
    Apply windowed RMS to each channel in the multichannel EMG data.

    Parameters:
        emg_data: Numpy array of shape (num_samples, num_channels).
        window_size: Size of the window for RMS calculation.
        verbose: Whether to print progress.

    Returns:
        Smoothed EMG data with windowed RMS applied to each channel (same shape as input).
    """
    if verbose: print(f"| Applying windowed RMS with window size {window_size}")
    num_channels, num_samples = emg_data.shape
    rms_data = np.zeros((num_channels, num_samples))

    for i in range(num_channels):
        rms_data[i, :] = window_rms_1D(emg_data[i, :], window_size)

    return rms_data


def window_rms_1D(signal, window_size):
    """
    Compute windowed RMS of the signal.

    Parameters:
        signal: Input EMG signal.
        window_size: Size of the window for RMS calculation.

    Returns:
        Windowed RMS signal.
    """
    return np.sqrt(np.convolve(signal ** 2, np.ones(window_size) / window_size, mode='same'))


def calculate_rms(data, window_size, verbose=False):
    """
    Calculates RMS features for each channel using non-overlapping windows.

    Parameters:
        data: 2D numpy array of EMG data (channels, samples).
        window_size: Size of the window for RMS calculation.
        verbose: Whether to print progress.
    Returns:
        rms_features: 2D numpy array of RMS features (channels, windows).
    """
    if verbose:
        print("| Calculating RMS features...")
    n_channels, n_samples = data.shape
    n_windows = n_samples // window_size
    rms_features = np.zeros((n_channels, n_windows))

    for ch in range(n_channels):
        for i in range(n_windows):
            window = data[ch, i * window_size:(i + 1) * window_size]
            rms_features[ch, i] = np.sqrt(np.mean(window ** 2))

    return rms_features  # Shape (n_channels, n_windows)


def compute_rolling_rms(signal, window_size=50):
    """Compute rolling RMS on a 1D signal."""
    squared = np.square(signal)
    window = np.ones(window_size) / window_size
    return np.sqrt(np.convolve(squared, window, mode='same'))

def downsample(emg_data, sampling_rate, target_fs=1000):
    """
    Downsamples the EMG data to the target sampling rate.

    Parameters:
        emg_data: 2D numpy array of shape (num_channels, num_samples).
        sampling_rate: Sampling rate of the original EMG data.
        target_fs: Target sampling rate for downsampling.

    Returns:
        downsampled_data: 2D numpy array of shape (num_channels, downsampled_samples).
    """
    # Compute the downsampling factor
    downsample_factor = int(sampling_rate / target_fs)

    # Downsample the data by taking every nth sample
    downsampled_data = emg_data[:, ::downsample_factor]

    return downsampled_data


def common_average_reference(emg_data, verbose=False):
    """
    Applies Common Average Referencing (CAR) to the multi-channel EMG data.

    Parameters:
        emg_data: 2D numpy array of shape (num_channels, num_samples).

    Returns:
        car_data: 2D numpy array after applying CAR (same shape as input).
    """
    if verbose:
        print("| Subtracting common average reference")
        print("Shape of input data:", emg_data.shape)
    # Compute the common average (mean across all channels at each time point)
    common_avg = np.mean(emg_data, axis=0)  # Shape: (num_samples,)

    # Subtract the common average from each channel
    car_data = emg_data - common_avg  # Broadcast subtraction across channels

    return car_data


def envelope_extraction(data, method='hilbert'):
    """
    Extracts the envelope of the EMG signal using the Hilbert transform.

    Parameters:
        data: 2D numpy array of EMG data (channels, samples).
        method: Method for envelope extraction ('hilbert' or other).

    Returns:
        envelope: 2D numpy array of the envelope (channels, samples).
    """
    if method == 'hilbert':
        analytic_signal = hilbert(data, axis=1)
        envelope = np.abs(analytic_signal)
    else:
        raise ValueError("Unsupported method for envelope extraction.")
    return envelope


def z_score_norm(data):
    """
    Apply z-score normalization to the input data.

    Parameters:
        data: 2D numpy array of shape (channels, samples).

    Returns:
        normalized_data: 2D numpy array of shape (channels, samples) after z-score normalization.
    """
    mean = np.mean(data, axis=1)[:, np.newaxis]
    std = np.std(data, axis=1)[:, np.newaxis]
    normalized_data = (data - mean) / std
    return normalized_data


# RMS (Root Mean Square)
def compute_rms(emg_window, axis=-1):
    """
    Compute RMS of EMG data along a given axis.

    Parameters:
        emg_window (np.ndarray): EMG data. Can be 1D or 2D.
            - 1D shape: (n_samples,)
            - 2D shape: (n_channels, n_samples)
        axis (int): Axis to compute RMS over. Default is -1 (last axis).

    Returns:
        np.ndarray or float: RMS value(s) along the given axis.
            - If input is 1D: returns float
            - If input is 2D: returns 1D array (n_channels,)
    """
    emg_window = np.asarray(emg_window)
    return np.sqrt(np.mean(emg_window ** 2, axis=axis))


def compute_grid_average(emg_data, grid_spacing=8, axis=0):
    """
    Computes the average of the EMG grids according to the grid spacing. For example, a spacing of 8 means that
    channels 1, 9, 17, etc. will be averaged together to form the first grid, and so on.

    Parameters:
        emg_data (np.ndarray): 2D numpy array of shape (num_channels, num_samples).
        grid_spacing (int): Number of channels to average together.
        axis (int): Axis along which to compute the grid averages.

    Returns:
        grid_averages (np.ndarray): 2D numpy array of shape (num_grids, num_samples).
    """
    num_channels, num_samples = emg_data.shape
    num_grids = num_channels // grid_spacing
    grid_averages = np.zeros((num_grids, num_samples))

    for i in range(num_grids):
        start_idx = i * grid_spacing
        end_idx = (i + 1) * grid_spacing
        grid_averages[i, :] = np.mean(emg_data[start_idx:end_idx, :], axis=axis)

    return grid_averages
