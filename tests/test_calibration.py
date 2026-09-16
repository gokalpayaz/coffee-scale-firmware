import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from calibration import calculate_scale_factor


class CalibrationTests(unittest.TestCase):
    def test_calculates_raw_counts_per_gram(self):
        self.assertEqual(2175.0, calculate_scale_factor(1000, 218500, 100))

    def test_preserves_negative_load_cell_direction(self):
        self.assertEqual(-2175.0, calculate_scale_factor(218500, 1000, 100))

    def test_rejects_invalid_reference_weight(self):
        with self.assertRaises(ValueError):
            calculate_scale_factor(1000, 218500, 0)

    def test_rejects_unchanged_raw_reading(self):
        with self.assertRaises(ValueError):
            calculate_scale_factor(1000, 1000, 100)


if __name__ == "__main__":
    unittest.main()
