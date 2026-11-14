"""Deterministic, dependency-light guidance and control simulations.

These are educational point-mass and kinematic models, not flight- or
marine-ready control software. See docs/model_assumptions.md before use.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class DroneConfig:
    dt: float = 0.02
    duration: float = 15.0
    mass: float = 1.0
    gravity: float = 9.81
    target_altitude: float = 10.0
    disturbance_time: float = 6.0
    wind_force: float = 5.0
    max_thrust: float = 30.0
    kp: float = 5.0
    ki: float = 0.4
    kd: float = 5.0
    integral_limit: float = 10.0

    def __post_init__(self):
        if self.dt <= 0 or self.duration <= 0:
            raise ValueError("dt and duration must be positive")
        if self.mass <= 0 or self.gravity <= 0:
            raise ValueError("mass and gravity must be positive")
        if self.max_thrust <= self.mass * self.gravity:
            raise ValueError("max_thrust must exceed hover thrust")
        if self.disturbance_time < 0 or self.integral_limit < 0:
            raise ValueError("disturbance_time and integral_limit cannot be negative")
        if min(self.kp, self.ki, self.kd, self.wind_force) < 0:
            raise ValueError("gains and wind_force must be non-negative")

@dataclass(frozen=True)
class DroneResult:
    time: np.ndarray
    altitude: np.ndarray
    velocity: np.ndarray
    thrust: np.ndarray
    disturbance: np.ndarray
    target_altitude: float

@dataclass(frozen=True)
class BoatConfig:
    dt: float = 0.05
    duration: float = 40.0
    path_amplitude: float = 5.0
    path_frequency: float = 0.15
    speed: float = 2.0
    heading_gain: float = 2.5
    cross_track_gain: float = 0.8
    max_yaw_rate: float = 1.0
    max_course_correction: float = 0.7
    current_x: float = 0.3
    current_y: float = 0.2

    def __post_init__(self):
        if self.dt <= 0 or self.duration <= 0 or self.speed <= 0:
            raise ValueError("dt, duration and speed must be positive")
        if self.heading_gain < 0 or self.cross_track_gain < 0:
            raise ValueError("guidance gains must be non-negative")
        if self.max_yaw_rate <= 0 or self.max_course_correction <= 0:
            raise ValueError("yaw-rate and course-correction limits must be positive")

@dataclass(frozen=True)
class BoatResult:
    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    desired_y: np.ndarray
    cross_track_error: np.ndarray

def _time_grid(dt: float, duration: float) -> np.ndarray:
    """Create a grid that includes the requested final time."""
    steps = int(round(duration / dt))
    return np.linspace(0.0, steps * dt, steps + 1)

def simulate_drone(config: DroneConfig = DroneConfig()) -> DroneResult:
    """Simulate 1-D altitude hold with PID feedback and a step wind disturbance.

    Gravity feed-forward and derivative-on-measurement are used. The dynamics
    are integrated with a semi-implicit Euler step.
    """
    time = _time_grid(config.dt, config.duration)
    n = time.size
    altitude = np.zeros(n, dtype=float)
    velocity = np.zeros(n, dtype=float)
    thrust = np.zeros(n, dtype=float)
    disturbance = np.where(time >= config.disturbance_time, config.wind_force, 0.0)
    integral_error = 0.0
    hover_thrust = config.mass * config.gravity

    for i in range(1, n):
        error = config.target_altitude - altitude[i - 1]
        candidate_integral = float(np.clip(
            integral_error + error * config.dt,
            -config.integral_limit, config.integral_limit
        ))
        correction = config.kp * error + config.ki * candidate_integral - config.kd * velocity[i - 1]
        raw_thrust = hover_thrust + correction
        applied_thrust = float(np.clip(raw_thrust, 0.0, config.max_thrust))

        # Conditional integration prevents winding farther into actuator saturation.
        drives_upper = raw_thrust > config.max_thrust and error > 0
        drives_lower = raw_thrust < 0.0 and error < 0
        if not (drives_upper or drives_lower):
            integral_error = candidate_integral
            correction = config.kp * error + config.ki * integral_error - config.kd * velocity[i - 1]
            applied_thrust = float(np.clip(hover_thrust + correction, 0.0, config.max_thrust))

        thrust[i] = applied_thrust
        acceleration = (applied_thrust - hover_thrust - disturbance[i]) / config.mass
        velocity[i] = velocity[i - 1] + acceleration * config.dt
        altitude[i] = altitude[i - 1] + velocity[i] * config.dt
        if altitude[i] < 0.0:
            altitude[i] = 0.0
            velocity[i] = max(0.0, velocity[i])

    return DroneResult(time, altitude, velocity, thrust, disturbance, config.target_altitude)

def desired_path(x: np.ndarray, amplitude: float, frequency: float) -> np.ndarray:
    """Return a sinusoidal reference path y(x)."""
    return amplitude * np.sin(frequency * x)

def _wrap_angle(angle: float) -> float:
    """Wrap radians to the half-open interval [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def simulate_boat(config: BoatConfig = BoatConfig()) -> BoatResult:
    """Simulate a constant-speed boat tracking a sinusoidal path in a current.

    A line-of-sight style course command combines path-tangent direction and
    cross-track correction. The yaw response is rate limited.
    """
    time = _time_grid(config.dt, config.duration)
    n = time.size
    x = np.zeros(n, dtype=float)
    y = np.zeros(n, dtype=float)
    heading = np.zeros(n, dtype=float)

    for i in range(1, n):
        slope = config.path_amplitude * config.path_frequency * np.cos(config.path_frequency * x[i - 1])
        reference_y = float(desired_path(np.asarray([x[i - 1]]), config.path_amplitude, config.path_frequency)[0])
        cross_track = reference_y - y[i - 1]
        tangent_heading = float(np.arctan2(slope, 1.0))
        correction = float(np.clip(
            np.arctan2(config.cross_track_gain * cross_track, config.speed),
            -config.max_course_correction, config.max_course_correction
        ))
        desired_heading = tangent_heading + correction
        heading_error = _wrap_angle(desired_heading - heading[i - 1])
        yaw_rate = float(np.clip(
            config.heading_gain * heading_error,
            -config.max_yaw_rate, config.max_yaw_rate
        ))
        heading[i] = _wrap_angle(heading[i - 1] + yaw_rate * config.dt)
        x[i] = x[i - 1] + (config.speed * np.cos(heading[i]) + config.current_x) * config.dt
        y[i] = y[i - 1] + (config.speed * np.sin(heading[i]) + config.current_y) * config.dt

    desired_y = desired_path(x, config.path_amplitude, config.path_frequency)
    cross_track_error = desired_y - y
    return BoatResult(time, x, y, heading, desired_y, cross_track_error)
