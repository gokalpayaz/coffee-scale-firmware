import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import BrewConfig
from tare import TareSettler


class TareSettlerTests(unittest.TestCase):
    def setUp(self):
        self.config = BrewConfig()
        self.settler = TareSettler(self.config)

    def test_waits_for_minimum_delay_and_full_stable_window(self):
        self.settler.request(0)

        self.assertFalse(self.settler.update(10.0, 349))
        self.assertFalse(self.settler.update(0.1, 350))
        self.assertFalse(self.settler.update(0.2, 649))
        self.assertTrue(self.settler.update(0.15, 650))
        self.assertEqual("stable", self.settler.last_reason)
        self.assertFalse(self.settler.pending)

    def test_movement_restarts_stability_window(self):
        self.settler.request(0)
        self.assertFalse(self.settler.update(0.0, 350))
        self.assertFalse(self.settler.update(0.3, 500))
        self.assertFalse(self.settler.update(0.3, 799))
        self.assertTrue(self.settler.update(0.3, 800))

    def test_timeout_completes_when_platform_never_settles(self):
        self.settler.request(0)
        self.assertFalse(self.settler.update(0.0, 350))
        self.assertFalse(self.settler.update(1.0, 1900))
        self.assertTrue(self.settler.update(-1.0, 2000))
        self.assertEqual("timeout", self.settler.last_reason)

    def test_request_restarts_an_existing_wait(self):
        self.settler.request(0)
        self.settler.update(0.0, 350)
        self.settler.request(500)

        self.assertFalse(self.settler.update(0.0, 849))
        self.assertFalse(self.settler.update(0.0, 850))
        self.assertTrue(self.settler.pending)

    def test_update_is_idle_before_request_and_after_completion(self):
        self.assertFalse(self.settler.update(0.0, 0))
        self.settler.request(0)
        self.settler.update(0.0, 350)
        self.assertTrue(self.settler.update(0.0, 650))
        self.assertFalse(self.settler.update(0.0, 651))


if __name__ == "__main__":
    unittest.main()
