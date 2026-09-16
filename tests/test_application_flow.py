import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import (
    ACTION_SESSION_STARTED,
    ACTION_SESSION_STOPPED,
    ACTION_TARE,
    BrewConfig,
    DisplayState,
    PROFILE_POUR_OVER,
    SESSION_ARMED,
    SESSION_RUNNING,
    SESSION_STOPPED,
)
from ble_protocol import parse_bookoo_command
from input_events import InputInterpreter
from measurement import MeasurementController


class ApplicationFlowTests(unittest.TestCase):
    def test_button_chord_selects_pour_over_and_auto_starts(self):
        config = BrewConfig()
        config.debounce_ms = 0
        config.combo_hold_ms = 100
        state = DisplayState(config)
        controller = MeasurementController(config, state)
        inputs = InputInterpreter(config)

        inputs.update(False, False, False, 0)
        inputs.update(True, True, False, 1)
        events = inputs.update(True, True, False, 101)
        for event in events:
            controller.handle_event(event, 101)
        self.assertTrue(state.auto_menu_open)

        # Release after the chord is suppressed, then use right short to select
        # Pour-over and left short to confirm the selection.
        self.assertEqual((), inputs.update(False, False, True, 102))
        inputs.update(False, True, True, 110)
        for event in inputs.update(False, False, True, 120):
            controller.handle_event(event, 120)
        self.assertEqual(PROFILE_POUR_OVER, state.auto_profile)

        inputs.update(True, False, True, 130)
        for event in inputs.update(False, False, True, 140):
            controller.handle_event(event, 140)
        self.assertEqual(SESSION_ARMED, state.session_state)

        controller.update(0.0, 140)
        controller.update(0.0, 1140)
        controller.update(0.5, 1200)
        actions = controller.update(0.5, 1500)

        self.assertTrue(actions & ACTION_SESSION_STARTED)
        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_bookoo_start_stop_are_idempotent_and_tare_start_is_ordered(self):
        config = BrewConfig()
        state = DisplayState(config)
        controller = MeasurementController(config, state)

        start = parse_bookoo_command(b"\x03\x0a\x04")
        stop = parse_bookoo_command(b"\x03\x0a\x05")

        self.assertTrue(controller.handle_event(start[0], 0) & ACTION_SESSION_STARTED)
        self.assertEqual(0, controller.handle_event(start[0], 10))
        self.assertEqual(SESSION_RUNNING, state.session_state)

        self.assertTrue(controller.handle_event(stop[0], 20) & ACTION_SESSION_STOPPED)
        self.assertEqual(0, controller.handle_event(stop[0], 30))
        self.assertEqual(SESSION_STOPPED, state.session_state)

        controller.handle_event(parse_bookoo_command(b"\x03\x0a\x06")[0], 40)
        tare_start = parse_bookoo_command(b"\x03\x0a\x07")
        self.assertTrue(controller.handle_event(tare_start[0], 50) & ACTION_TARE)
        controller.on_tare(50)
        self.assertTrue(
            controller.handle_event(tare_start[1], 60) & ACTION_SESSION_STARTED
        )
        self.assertEqual(SESSION_RUNNING, state.session_state)


if __name__ == "__main__":
    unittest.main()
