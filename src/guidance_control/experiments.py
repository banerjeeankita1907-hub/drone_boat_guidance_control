"""Deterministic disturbance and current sensitivity experiments."""
from __future__ import annotations

from dataclasses import replace
from collections.abc import Iterable

import numpy as np

from .metrics import tracking_metrics
from .simulation import BoatConfig, DroneConfig, simulate_boat, simulate_drone


def _non_negative_levels(values: Iterable[float], name: str) -> tuple[float, ...]:
    """Validate a finite, non-empty sequence of non-negative sweep values."""
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{name} must be a sequence of non-negative numbers")
    try:
        levels = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a sequence of non-negative numbers") from error
    if not levels or any(not np.isfinite(value) or value < 0 for value in levels):
        raise ValueError(f"{name} must contain finite, non-negative values")
    return levels


def run_robustness_sweeps(
    wind_levels: Iterable[float] = (0.0, 2.5, 5.0, 7.5, 10.0),
    current_scales: Iterable[float] = (0.0, 0.5, 1.0, 1.5, 2.0),
    drone_config: DroneConfig = DroneConfig(),
    boat_config: BoatConfig = BoatConfig(),
) -> dict[str, list[dict[str, float | None]]]:
    """Evaluate fixed disturbance/current grids around the baseline scenarios.

    This is a deterministic one-factor-at-a-time sensitivity study, not a
    probabilistic uncertainty analysis or a controller-optimization routine.
    """
    wind_values = _non_negative_levels(wind_levels, "wind_levels")
    current_values = _non_negative_levels(current_scales, "current_scales")

    drone_rows: list[dict[str, float | None]] = []
    for wind_force in wind_values:
        config = replace(drone_config, wind_force=wind_force)
        result = simulate_drone(config)
        metrics = tracking_metrics(
            result.time, result.altitude, result.target_altitude, tolerance=0.02
        )
        drone_rows.append({
            "wind_force_n": wind_force,
            "altitude_rmse_m": metrics.rmse,
            "max_abs_altitude_error_m": metrics.maximum_absolute_error,
            "settling_time_s": metrics.settling_time,
            "max_thrust_saturation_fraction": float(
                np.mean(np.isclose(result.thrust, config.max_thrust, rtol=0.0, atol=1e-9))
            ),
            "zero_thrust_fraction": float(
                np.mean(np.isclose(result.thrust, 0.0, rtol=0.0, atol=1e-9))
            ),
        })

    boat_rows: list[dict[str, float | None]] = []
    for scale in current_values:
        config = replace(
            boat_config,
            current_x=boat_config.current_x * scale,
            current_y=boat_config.current_y * scale,
        )
        result = simulate_boat(config)
        metrics = tracking_metrics(
            result.time, result.cross_track_error, setpoint=0.0, tolerance=0.05
        )
        boat_rows.append({
            "current_scale": scale,
            "current_x_m_s": config.current_x,
            "current_y_m_s": config.current_y,
            "cross_track_rmse_m": metrics.rmse,
            "max_abs_cross_track_error_m": metrics.maximum_absolute_error,
            "settling_time_s": metrics.settling_time,
        })

    return {
        "drone_disturbance_sweep": drone_rows,
        "boat_current_sweep": boat_rows,
    }
