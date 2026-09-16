import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from filtering import RobustWeightFilter


class RobustWeightFilterTests(unittest.TestCase):
    def test_rejects_an_isolated_large_spike(self):
        weight_filter = RobustWeightFilter()

        values = [weight_filter.update_estimate(value) for value in (0, 0, 0, 20, 0)]

        self.assertEqual(0.0, values[-1])
        self.assertTrue(all(value == 0.0 for value in values))

    def test_uses_faster_response_for_a_real_weight_change(self):
        weight_filter = RobustWeightFilter()
        for _ in range(5):
            weight_filter.update_estimate(0.0)

        estimates = [weight_filter.update_estimate(10.0) for _ in range(5)]

        self.assertEqual(0.0, estimates[0])
        self.assertEqual(0.0, estimates[1])
        self.assertGreater(estimates[2], 5.0)
        self.assertGreater(estimates[-1], 9.0)

    def test_smooths_small_stationary_noise_and_snaps_near_zero(self):
        weight_filter = RobustWeightFilter()

        estimates = [
            weight_filter.update_estimate(value)
            for value in (0.02, -0.03, 0.04, -0.02, 0.01, 0.03, -0.01)
        ]

        self.assertTrue(all(value == 0.0 for value in estimates))

    def test_persistent_small_weight_eventually_leaves_zero_deadband(self):
        weight_filter = RobustWeightFilter()
        for _ in range(5):
            weight_filter.update_estimate(0.0)

        estimates = [weight_filter.update_estimate(0.1) for _ in range(12)]

        self.assertEqual(0.0, estimates[0])
        self.assertGreater(estimates[-1], 0.05)

    def test_reset_discards_pre_tare_history(self):
        weight_filter = RobustWeightFilter()
        for _ in range(5):
            weight_filter.update_estimate(50.0)

        weight_filter.reset(0.0)

        self.assertEqual(0.0, weight_filter.update_estimate(0.0))

    def test_validates_configuration(self):
        with self.assertRaises(ValueError):
            RobustWeightFilter(window_size=4)
        with self.assertRaises(ValueError):
            RobustWeightFilter(stable_alpha=0.8, moving_alpha=0.5)


if __name__ == "__main__":
    unittest.main()
