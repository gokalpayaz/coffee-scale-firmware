import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import (
    EVENT_AUTO_CANCEL,
    EVENT_AUTO_CONFIRM,
    EVENT_AUTO_MENU,
    EVENT_AUTO_NEXT,
    EVENT_MODE_NEXT,
    EVENT_TARE,
    EVENT_TIMER_RESET,
    EVENT_TIMER_TOGGLE,
)
from input_events import InputInterpreter


class InputInterpreterTests(unittest.TestCase):
    def setUp(self):
        self.inputs = InputInterpreter()
        self.inputs.update(False, False, False, 0)

    def _gesture(self, left, right, hold_ms=100, menu_open=False):
        self.assertEqual(
            (), self.inputs.update(left, right, menu_open, 10)
        )
        self.assertEqual(
            (), self.inputs.update(left, right, menu_open, 50)
        )
        release_ms = 50 + hold_ms
        self.assertEqual(
            (), self.inputs.update(False, False, menu_open, release_ms)
        )
        return self.inputs.update(
            False, False, menu_open, release_ms + 40
        )

    def test_bounce_must_settle_for_40ms_before_short_event(self):
        self.assertEqual((), self.inputs.update(True, False, False, 10))
        self.assertEqual((), self.inputs.update(False, False, False, 25))
        self.assertEqual((), self.inputs.update(True, False, False, 30))
        self.assertEqual((), self.inputs.update(True, False, False, 69))
        self.assertEqual((), self.inputs.update(True, False, False, 70))
        self.assertEqual((), self.inputs.update(False, False, False, 100))
        self.assertEqual((), self.inputs.update(False, False, False, 139))
        self.assertEqual(
            (EVENT_TIMER_TOGGLE,),
            self.inputs.update(False, False, False, 140),
        )

    def test_normal_short_and_long_mappings(self):
        self.assertEqual((EVENT_TARE,), self._gesture(False, True))

        self.setUp()
        self.assertEqual((EVENT_TIMER_TOGGLE,), self._gesture(True, False))

        self.setUp()
        self.assertEqual(
            (EVENT_MODE_NEXT,), self._gesture(False, True, hold_ms=1100)
        )

        self.setUp()
        self.assertEqual(
            (EVENT_TIMER_RESET,), self._gesture(True, False, hold_ms=1100)
        )

    def test_exposes_debounced_right_state_through_release(self):
        self.inputs.update(False, True, False, 10)
        self.inputs.update(False, True, False, 50)
        self.assertTrue(self.inputs.right_is_pressed)

        self.inputs.update(False, False, False, 60)
        self.assertTrue(self.inputs.right_is_pressed)
        self.inputs.update(False, False, False, 100)
        self.assertFalse(self.inputs.right_is_pressed)

    def test_menu_short_and_long_mappings(self):
        self.assertEqual(
            (EVENT_AUTO_NEXT,), self._gesture(False, True, menu_open=True)
        )

        self.setUp()
        self.assertEqual(
            (EVENT_AUTO_CONFIRM,), self._gesture(True, False, menu_open=True)
        )

        self.setUp()
        self.assertEqual(
            (EVENT_AUTO_CANCEL,),
            self._gesture(False, True, hold_ms=1100, menu_open=True),
        )

        self.setUp()
        self.assertEqual(
            (EVENT_AUTO_CANCEL,),
            self._gesture(True, False, hold_ms=1100, menu_open=True),
        )

    def test_chord_emits_once_and_suppresses_release_events(self):
        self.assertEqual((), self.inputs.update(True, True, False, 10))
        self.assertEqual((), self.inputs.update(True, True, False, 50))
        self.assertEqual((), self.inputs.update(True, True, False, 1049))
        self.assertEqual(
            (EVENT_AUTO_MENU,),
            self.inputs.update(True, True, False, 1050),
        )
        self.assertEqual((), self.inputs.update(True, True, True, 1100))
        self.assertEqual((), self.inputs.update(False, False, True, 1110))
        self.assertEqual((), self.inputs.update(False, False, True, 1150))

        self.assertEqual((), self.inputs.update(True, False, False, 1200))
        self.assertEqual((), self.inputs.update(True, False, False, 1240))
        self.assertEqual((), self.inputs.update(False, False, False, 1300))
        self.assertEqual(
            (EVENT_TIMER_TOGGLE,),
            self.inputs.update(False, False, False, 1340),
        )

    def test_chord_cancels_when_menu_is_open(self):
        self.inputs.update(True, True, True, 10)
        self.inputs.update(True, True, True, 50)

        self.assertEqual(
            (EVENT_AUTO_CANCEL,),
            self.inputs.update(True, True, True, 1050),
        )


if __name__ == "__main__":
    unittest.main()
