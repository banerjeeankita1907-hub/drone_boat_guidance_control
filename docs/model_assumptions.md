# Model assumptions and limitations

This repository contains transparent teaching models. They are useful for
exploring feedback, disturbances, saturation and tracking metrics; they are
not validated vehicle models and must not be connected to real actuators.

## Drone altitude model

The state is vertical position z and vertical velocity v. The model is

    a = (T - m g - F_w) / m
    v[k+1] = v[k] + a[k] dt
    z[k+1] = z[k] + v[k+1] dt

where T is bounded thrust and F_w is a downward step disturbance. A PID
controller with gravity feed-forward commands thrust. The derivative term is
computed from measured vertical velocity, and conditional integration limits
windup while the actuator is saturated. A simple ground constraint clamps z
to zero.

The model omits attitude, horizontal motion, rotor dynamics, battery limits,
aerodynamic drag, sensor noise, actuator delay, ground effect and motor failure.
The default gains are illustrative starting values, not a tuned flight
controller. There is no claim of flight readiness or hardware validation.

## Autonomous boat model

The boat moves at constant through-water speed and has a heading state. A
line-of-sight style guidance law combines the tangent to a sinusoidal reference
path with a bounded cross-track correction. Heading changes at a bounded yaw
rate. A constant current is added to ground-relative velocity.

The model omits wave loads, added mass, hydrodynamic coefficients, actuator
dynamics, navigation sensor errors, changing currents, collision avoidance and
environmental constraints. The constant-speed assumption is deliberately
simple. Results are simulation outputs, not evidence of autonomous maritime
operation.

## Numerical and evaluation choices

Both models use a fixed time step and semi-implicit Euler integration. The
CSV files preserve each time sample. Tracking metrics include RMSE, mean
absolute error, maximum absolute error, overshoot and a sustained-entry
settling-time estimate. The tolerance band is a fraction of max(abs(setpoint), 1).

For a research-grade extension, report parameter provenance, sensitivity to
time-step size, multiple disturbance scenarios, uncertainty intervals and
comparisons against a higher-fidelity model before drawing performance claims.
