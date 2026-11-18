# Drone & Autonomous Boat Guidance Control

A reproducible Python study of two introductory control problems: vertical
altitude regulation for a drone and sinusoidal path tracking for an autonomous
surface vessel.

The repository retains the original interactive Jupyter notebook and adds a
reusable, tested Python package with command-line experiments, exported
trajectories, quantitative metrics and continuous integration.

## Research questions

1. How does a bounded PID controller respond to a step disturbance in a
   simplified vertical point-mass model?
2. How does a line-of-sight style guidance law respond to a constant current
   while tracking a curved reference path?
3. What do tracking error, overshoot and settling-time metrics reveal that a
   trajectory plot alone may hide?

## Quick start

Python 3.10 or newer is recommended.

```bash
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
python -m pip install -e ".[dev]"
python -m pytest
guidance-control --output-dir results
guidance-control --output-dir results --sensitivity
```

The experiment runner writes:

- `results/drone_trajectory.csv` – altitude, velocity, thrust and disturbance.
- `results/boat_trajectory.csv` – position, heading, reference path and
  cross-track error.
- `results/metrics.json` – RMSE, MAE, maximum absolute error, overshoot and
  sustained-entry settling-time estimates.
- With `--sensitivity`, `drone_disturbance_sweep.csv` and
  `boat_current_sweep.csv` – deterministic one-factor-at-a-time robustness
  sweeps over disturbance strength and current magnitude.
- `results/drone_altitude_response.png` and
  `results/boat_path_tracking.png` – generated figures.

## Package structure

```text
src/guidance_control/
  simulation.py       # drone and boat models
  metrics.py          # reusable tracking metrics
  cli.py              # reproducible experiment runner
tests/
  test_simulation.py
  test_metrics.py
docs/
  model_assumptions.md
.github/workflows/
  tests.yml
drone_boat_guidance_control.ipynb  # original interactive notebook
```

## Model overview

### Drone

The vertical point-mass model includes gravity, bounded thrust, a PID altitude
controller, derivative-on-measurement, conditional integral anti-windup and a
step disturbance. Semi-implicit Euler integration is used.

### Boat

The vessel moves at constant through-water speed. A line-of-sight style
guidance law combines reference-path tangent direction with bounded cross-track
correction; heading rate is limited and a constant current affects ground
velocity.

Model equations, assumptions and omissions are documented in
[docs/model_assumptions.md](docs/model_assumptions.md).

## Interactive notebook

Open `drone_boat_guidance_control.ipynb` in Jupyter or Google Colab to use the
original sliders, response plots and animations. The new package is designed
to make baseline experiments repeatable outside the notebook.

## Sensitivity experiments

The optional sensitivity mode evaluates five fixed downward-wind loads for the
drone and five scales of the baseline two-axis current for the boat. It reports
tracking errors, settling-time estimates and actuator saturation fractions.
These are deterministic scenario sweeps, not Monte Carlo uncertainty bounds,
parameter optimization or proof of robustness outside the selected grid.

## Reproducibility and quality checks

GitHub Actions runs the unit tests on Python 3.10, 3.11 and 3.12 for pushes and
pull requests. The simulations use deterministic defaults; exported CSV and
JSON files retain the numerical data behind the figures.

## Scope and safety

This is an educational simulation project. It is **not** a certified flight
controller, marine autopilot or validated digital twin. The models omit several
important physical effects and have not been validated on hardware. Do not use
them to control real vehicles. Performance results should be described as
outputs of these specified simplified models only.

## License

MIT. See [LICENSE](LICENSE).
