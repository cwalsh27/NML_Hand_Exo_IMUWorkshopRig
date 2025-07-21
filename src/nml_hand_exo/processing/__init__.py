from ._filters import (
    #notch_filter,
    #bandpass_filter,
    #lowpass_filter,
    #filter_emg,
    RealtimeEMGFilter,
)
from ._features import (
    rectify,
    window_rms,
    window_rms_1D,
    compute_rms,
    compute_rolling_rms,
    downsample,
    common_average_reference,
    compute_grid_average,
    z_score_norm,
    #apply_pca,
    orthogonalize,
    normalize,
)
from ._orientation import OrientationFilter
