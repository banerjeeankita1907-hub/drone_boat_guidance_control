import unittest
import numpy as np
from guidance_control.metrics import tracking_metrics

class TrackingMetricsTests(unittest.TestCase):
    def test_metrics_and_sustained_settling_time(self):
        result = tracking_metrics(
            np.array([0.0, 1.0, 2.0, 3.0]),
            np.array([0.0, 1.0, 0.05, 0.02]),
            setpoint=0.0,
            tolerance=0.05,
        )
        self.assertAlmostEqual(result.maximum_absolute_error, 1.0)
        self.assertAlmostEqual(result.overshoot, 1.0)
        self.assertEqual(result.settling_time, 2.0)

    def test_rejects_mismatched_or_non_monotonic_inputs(self):
        with self.assertRaises(ValueError):
            tracking_metrics(np.array([0.0, 1.0]), np.array([1.0]))
        with self.assertRaises(ValueError):
            tracking_metrics(np.array([0.0, 0.0]), np.array([1.0, 1.0]))

if __name__ == "__main__":
    unittest.main()
