import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import (
    MODE_ALL,
    MODE_TIMER_WEIGHT,
    PROFILE_ESPRESSO,
    PROFILE_POUR_OVER,
)
from preferences import Preferences


class FakeNVS:
    def __init__(self, values=None):
        self.values = dict(values or {})
        self.get_calls = []
        self.set_calls = []
        self.commit_calls = 0

    def get_i32(self, key):
        self.get_calls.append(key)
        if key not in self.values:
            raise OSError("missing")
        return self.values[key]

    def set_i32(self, key, value):
        self.set_calls.append((key, value))
        self.values[key] = value

    def commit(self):
        self.commit_calls += 1


class PreferencesTests(unittest.TestCase):
    def test_missing_values_use_safe_defaults(self):
        nvs = FakeNVS()
        preferences = Preferences(nvs)

        self.assertEqual(
            (MODE_TIMER_WEIGHT, PROFILE_ESPRESSO), preferences.load()
        )
        self.assertEqual([], nvs.set_calls)
        self.assertEqual(0, nvs.commit_calls)

    def test_corrupt_values_use_safe_defaults(self):
        nvs = FakeNVS({"mode": 99, "profile": -1})
        preferences = Preferences(nvs)

        self.assertEqual(MODE_TIMER_WEIGHT, preferences.load_mode())
        self.assertEqual(PROFILE_ESPRESSO, preferences.load_profile())

    def test_changed_values_write_and_commit_only_once(self):
        nvs = FakeNVS()
        preferences = Preferences(nvs)

        self.assertTrue(preferences.save_mode(MODE_ALL))
        self.assertFalse(preferences.save_mode(MODE_ALL))
        self.assertTrue(preferences.save_profile(PROFILE_POUR_OVER))
        self.assertFalse(preferences.save_profile(PROFILE_POUR_OVER))

        self.assertEqual(
            [("mode", MODE_ALL), ("profile", PROFILE_POUR_OVER)],
            nvs.set_calls,
        )
        self.assertEqual(2, nvs.commit_calls)
        self.assertEqual({"mode", "profile"}, set(nvs.values))

    def test_saving_effective_default_does_not_write_missing_key(self):
        nvs = FakeNVS()
        preferences = Preferences(nvs)

        self.assertFalse(preferences.save_mode(MODE_TIMER_WEIGHT))
        self.assertFalse(preferences.save_profile(PROFILE_ESPRESSO))
        self.assertEqual([], nvs.set_calls)
        self.assertEqual(0, nvs.commit_calls)

    def test_invalid_values_are_rejected_without_writes(self):
        nvs = FakeNVS()
        preferences = Preferences(nvs)

        with self.assertRaises(ValueError):
            preferences.save_mode(99)
        with self.assertRaises(ValueError):
            preferences.save_profile(99)

        self.assertEqual([], nvs.set_calls)
        self.assertEqual(0, nvs.commit_calls)


if __name__ == "__main__":
    unittest.main()
