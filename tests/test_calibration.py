import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from calibration import (
    DEFAULT_SCALE_FACTOR,
    CalibrationStore,
    calculate_scale_factor,
)


class FakeNVS:
    def __init__(self, values=None):
        self.values = dict(values or {})
        self.set_calls = []
        self.commit_calls = 0

    def get_i32(self, key):
        if key not in self.values:
            raise OSError("missing")
        return self.values[key]

    def set_i32(self, key, value):
        self.set_calls.append((key, value))
        self.values[key] = value

    def commit(self):
        self.commit_calls += 1


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


class CalibrationStoreTests(unittest.TestCase):
    def test_missing_value_uses_migration_default_without_writing(self):
        nvs = FakeNVS()
        store = CalibrationStore(nvs)

        self.assertEqual(DEFAULT_SCALE_FACTOR, store.load())
        self.assertEqual([], nvs.set_calls)
        self.assertEqual(0, nvs.commit_calls)

    def test_loads_positive_and_negative_fixed_point_values(self):
        self.assertEqual(
            465.6814,
            CalibrationStore(FakeNVS({"cal_factor": 4656814})).load(),
        )
        self.assertEqual(
            -2175.25,
            CalibrationStore(FakeNVS({"cal_factor": -21752500})).load(),
        )

    def test_first_save_persists_even_when_factor_matches_default(self):
        nvs = FakeNVS()
        store = CalibrationStore(nvs)

        self.assertTrue(store.save(DEFAULT_SCALE_FACTOR))
        self.assertEqual([("cal_factor", 4656814)], nvs.set_calls)
        self.assertEqual(1, nvs.commit_calls)

    def test_unchanged_saved_factor_does_not_rewrite_flash(self):
        nvs = FakeNVS({"cal_factor": 4656814})
        store = CalibrationStore(nvs)

        self.assertFalse(store.save(465.6814))
        self.assertEqual([], nvs.set_calls)
        self.assertEqual(0, nvs.commit_calls)

    def test_save_rounds_to_four_decimal_places(self):
        nvs = FakeNVS()
        store = CalibrationStore(nvs)

        self.assertTrue(store.save(123.45678))
        self.assertEqual([("cal_factor", 1234568)], nvs.set_calls)
        self.assertEqual(123.4568, store.load())

    def test_invalid_stored_value_falls_back_and_invalid_save_is_rejected(self):
        nvs = FakeNVS({"cal_factor": 0})
        store = CalibrationStore(nvs)

        self.assertEqual(DEFAULT_SCALE_FACTOR, store.load())
        with self.assertRaises(ValueError):
            store.save(0)
        self.assertEqual([], nvs.set_calls)


if __name__ == "__main__":
    unittest.main()
