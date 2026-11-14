"""Small, tested performance metrics for setpoint and path tracking."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import numpy as np

@dataclass(frozen=True)
class TrackingMetrics:
    rmse: float
    mean_absolute_error: float
    maximum_absolute_error: float
    overshoot: float
    settling_time: float | None

    def to_dict(self) -> dict[str, float | None]:
        return asdict(self)

def tracking_metrics(time: np.ndarray, signal: np.ndarray, setpoint: float = 0.0, tolerance: float = 0.02) -> TrackingMetrics:
    """Calculate tracking error and a sustained-entry settling-time estimate.

    Tolerance is a fraction of max(abs(setpoint), 1). Settling time is the
    first sample after which the signal stays inside the tolerance band.
    """
    t = np.asarray(time, dtype=float)
    values = np.asarray(signal, dtype=float)
    if t.ndim != 1 or values.ndim != 1 or t.size != values.size or t.size == 0:
        raise ValueError("time and signal must be non-empty 1-D arrays of equal length")
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(values)):
        raise ValueError("time and signal must contain only finite values")
    if t.size > 1 and np.any(np.diff(t) <= 0):
        raise ValueError("time must be strictly increasing")
    if tolerance <= 0:
        raise ValueError("tolerance must be positive")

    error = values - float(setpoint)
    band = tolerance * max(abs(float(setpoint)), 1.0)
    inside = np.abs(error) <= band
    settling_time = None
    for index in range(inside.size):
        if np.all(inside[index:]):
            settling_time = float(t[index])
            break
    return TrackingMetrics(
        rmse=float(np.sqrt(np.mean(error ** 2))),
        mean_absolute_error=float(np.mean(np.abs(error))),
        maximum_absolute_error=float(np.max(np.abs(error))),
        overshoot=float(max(0.0, np.max(values) - setpoint)),
        settling_time=settling_time,
    )
