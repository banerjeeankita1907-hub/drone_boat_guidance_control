import csv
import tempfile
import unittest
from pathlib import Path

from guidance_control.cli import write_sensitivity_outputs

from guidance_control.experiments import run_robustness_sweeps
from guidance_control.simulation import BoatConfig, DroneConfig


class RobustnessSweepTests(unittest.TestCase):
    def test_sweeps_return_one_row_per_requested_scenario(self):
        result = run_robustness_sweeps(
            wind_levels=(0.0, 4.0),
            current_scales=(0.0, 1.0, 2.0),
            drone_config=DroneConfig(duration=1.0),
            boat_config=BoatConfig(duration=1.0),
        )
        self.assertEqual(len(result["drone_disturbance_sweep"]), 2)
        self.assertEqual(len(result["boat_current_sweep"]), 3)
        self.assertEqual(result["drone_disturbance_sweep"][0]["wind_force_n"], 0.0)
        self.assertEqual(result["boat_current_sweep"][0]["current_x_m_s"], 0.0)
        self.assertEqual(result["boat_current_sweep"][0]["current_y_m_s"], 0.0)

    def test_baseline_sweep_is_deterministic(self):
        first = run_robustness_sweeps(
            wind_levels=(5.0,), current_scales=(1.0,),
            drone_config=DroneConfig(duration=1.0),
            boat_config=BoatConfig(duration=1.0),
        )
        second = run_robustness_sweeps(
            wind_levels=(5.0,), current_scales=(1.0,),
            drone_config=DroneConfig(duration=1.0),
            boat_config=BoatConfig(duration=1.0),
        )
        self.assertEqual(first, second)

    def test_sensitivity_export_writes_both_csv_tables(self):
        with tempfile.TemporaryDirectory() as directory:
            result = write_sensitivity_outputs(directory)
            expected_files = {
                "drone_disturbance_sweep.csv": len(result["drone_disturbance_sweep"]),
                "boat_current_sweep.csv": len(result["boat_current_sweep"]),
            }
            for filename, expected_rows in expected_files.items():
                path = Path(directory) / filename
                self.assertTrue(path.is_file())
                with path.open(newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), expected_rows)

    def test_invalid_sweep_values_are_rejected(self):
        for invalid in ((), (-1.0,), (float("nan"),), (float("inf"),)):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    run_robustness_sweeps(wind_levels=invalid)
        with self.assertRaises(ValueError):
            run_robustness_sweeps(current_scales=())


if __name__ == "__main__":
    unittest.main()
