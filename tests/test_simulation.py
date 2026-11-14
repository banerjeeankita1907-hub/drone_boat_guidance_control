import unittest
import numpy as np
from guidance_control.simulation import BoatConfig, DroneConfig, simulate_boat, simulate_drone

class SimulationTests(unittest.TestCase):
    def test_drone_outputs_are_finite_and_aligned(self):
        result = simulate_drone(DroneConfig(duration=8.0))
        for values in (result.altitude, result.velocity, result.thrust, result.disturbance):
            self.assertEqual(values.shape, result.time.shape)
            self.assertTrue(np.all(np.isfinite(values)))
        self.assertTrue(np.all(result.altitude >= 0.0))
        self.assertTrue(np.all(result.thrust >= 0.0))
        self.assertTrue(np.all(result.thrust <= 30.0))
        self.assertAlmostEqual(result.time[0], 0.0)

    def test_drone_disturbance_is_a_step_input(self):
        result = simulate_drone(DroneConfig(duration=8.0, disturbance_time=2.0, wind_force=4.0))
        self.assertTrue(np.all(result.disturbance[result.time < 2.0] == 0.0))
        self.assertTrue(np.all(result.disturbance[result.time >= 2.0] == 4.0))

    def test_boat_outputs_are_finite_and_aligned(self):
        result = simulate_boat(BoatConfig(duration=10.0))
        for values in (result.x, result.y, result.heading, result.desired_y, result.cross_track_error):
            self.assertEqual(values.shape, result.time.shape)
            self.assertTrue(np.all(np.isfinite(values)))

    def test_boat_heading_is_wrapped(self):
        result = simulate_boat(BoatConfig(duration=10.0))
        self.assertTrue(np.all(result.heading >= -np.pi))
        self.assertTrue(np.all(result.heading < np.pi))

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            DroneConfig(dt=0.0)
        with self.assertRaises(ValueError):
            BoatConfig(speed=0.0)

if __name__ == "__main__":
    unittest.main()
