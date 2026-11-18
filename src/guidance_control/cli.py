"""Run reproducible baseline simulations and export CSV, JSON and figures."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt

from .experiments import run_robustness_sweeps
from .metrics import tracking_metrics
from .simulation import BoatConfig, DroneConfig, simulate_boat, simulate_drone


def _write_csv(path: Path, headers: list[str], columns: list[list[float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        writer.writerows(zip(*columns))


def _write_dict_rows(path: Path, rows: list[dict[str, float | None]]) -> None:
    if not rows:
        raise ValueError("cannot write an empty experiment result")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_sensitivity_outputs(output_dir: str | Path = "results") -> dict[str, list[dict[str, float | None]]]:
    """Run the fixed sensitivity grids and export their rows as CSV."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    results = run_robustness_sweeps()
    _write_dict_rows(output / "drone_disturbance_sweep.csv", results["drone_disturbance_sweep"])
    _write_dict_rows(output / "boat_current_sweep.csv", results["boat_current_sweep"])
    return results


def run_experiments(output_dir: str | Path = "results") -> dict[str, dict[str, float | None]]:
    """Run default drone and boat scenarios and save machine-readable outputs."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    drone = simulate_drone(DroneConfig())
    boat = simulate_boat(BoatConfig())
    drone_metrics = tracking_metrics(
        drone.time, drone.altitude, drone.target_altitude, tolerance=0.02
    )
    boat_metrics = tracking_metrics(
        boat.time, boat.cross_track_error, setpoint=0.0, tolerance=0.05
    )
    summary = {
        "drone_altitude": drone_metrics.to_dict(),
        "boat_cross_track_error": boat_metrics.to_dict(),
    }

    _write_csv(
        output / "drone_trajectory.csv",
        ["time_s", "altitude_m", "velocity_m_s", "thrust_n", "wind_force_n"],
        [drone.time, drone.altitude, drone.velocity, drone.thrust, drone.disturbance],
    )
    _write_csv(
        output / "boat_trajectory.csv",
        ["time_s", "x_m", "y_m", "heading_rad", "reference_y_m", "cross_track_error_m"],
        [boat.time, boat.x, boat.y, boat.heading, boat.desired_y, boat.cross_track_error],
    )
    (output / "metrics.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(drone.time, drone.altitude, label="simulated altitude")
    ax.axhline(drone.target_altitude, linestyle="--", label="target altitude")
    ax.axvline(DroneConfig().disturbance_time, linestyle=":", label="wind disturbance")
    ax.set(title="Drone altitude response", xlabel="Time (s)", ylabel="Altitude (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "drone_altitude_response.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(boat.x, boat.desired_y, linestyle="--", label="reference path")
    ax.plot(boat.x, boat.y, label="simulated boat path")
    ax.set(title="Autonomous boat path tracking", xlabel="Along-track position (m)", ylabel="Cross-track position (m)")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "boat_path_tracking.png", dpi=160)
    plt.close(fig)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Run baseline drone and boat guidance simulations.")
    parser.add_argument("--output-dir", default="results", help="Directory for CSV, JSON and PNG outputs.")
    parser.add_argument("--sensitivity", action="store_true", help="Also export disturbance and current sensitivity sweeps.")
    args = parser.parse_args()
    summary = run_experiments(args.output_dir)
    print(json.dumps(summary, indent=2))
    if args.sensitivity:
        print(json.dumps(write_sensitivity_outputs(args.output_dir), indent=2))


if __name__ == "__main__":
    main()
