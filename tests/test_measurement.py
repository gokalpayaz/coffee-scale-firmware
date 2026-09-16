import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import (
    ACTION_NONE,
    ACTION_SAVE_MODE,
    ACTION_SAVE_PROFILE,
    ACTION_SESSION_STARTED,
    ACTION_SESSION_STOPPED,
    ACTION_SHOW_LOCK,
    ACTION_TARE,
    BrewConfig,
    DisplayState,
    EVENT_AUTO_CANCEL,
    EVENT_AUTO_CONFIRM,
    EVENT_AUTO_MENU,
    EVENT_AUTO_NEXT,
    EVENT_MODE_NEXT,
    EVENT_TARE,
    EVENT_TIMER_RESET,
    EVENT_TIMER_START,
    EVENT_TIMER_STOP,
    EVENT_TIMER_TOGGLE,
    EVENT_TRANSIENT_LOCK,
    MODE_ALL,
    MODE_TIMER_WEIGHT,
    PROFILE_ESPRESSO,
    PROFILE_POUR_OVER,
    SESSION_ARMED,
    SESSION_IDLE,
    SESSION_RUNNING,
    SESSION_STOPPED,
)
from measurement import MeasurementController


TICKS_PERIOD = 1 << 30


def make_controller(config=None):
    if config is None:
        config = BrewConfig()
    state = DisplayState(config)
    return config, state, MeasurementController(config, state)


def arm_and_start(controller, state, profile=PROFILE_ESPRESSO, offset=0):
    controller.handle_event(EVENT_AUTO_MENU, offset)
    if profile == PROFILE_POUR_OVER:
        controller.handle_event(EVENT_AUTO_NEXT, offset + 1)
    controller.handle_event(EVENT_AUTO_CONFIRM, offset + 2)
    controller.update(0.0, offset + 2)
    controller.update(0.0, offset + 1002)
    controller.update(0.5, offset + 1102)
    action = controller.update(0.5, offset + 1402)
    if not action & ACTION_SESSION_STARTED:
        raise AssertionError("auto session did not start")
    if state.session_state != SESSION_RUNNING:
        raise AssertionError("auto session is not running")
    return offset + 1402


class ManualLifecycleTests(unittest.TestCase):
    def test_explicit_start_and_stop_are_idempotent(self):
        _, state, controller = make_controller()

        self.assertEqual(
            ACTION_SESSION_STARTED,
            controller.handle_event(EVENT_TIMER_START, 0),
        )
        self.assertEqual(ACTION_NONE, controller.handle_event(EVENT_TIMER_START, 10))
        self.assertEqual(SESSION_RUNNING, state.session_state)

        self.assertEqual(
            ACTION_SESSION_STOPPED,
            controller.handle_event(EVENT_TIMER_STOP, 20),
        )
        self.assertEqual(ACTION_NONE, controller.handle_event(EVENT_TIMER_STOP, 30))
        self.assertEqual(SESSION_STOPPED, state.session_state)

    def test_start_stop_final_hold_resume_and_reset(self):
        _, state, controller = make_controller()
        controller.update(5.0, 0)

        action = controller.handle_event(EVENT_TIMER_TOGGLE, 100)
        self.assertEqual(ACTION_SESSION_STARTED, action)
        self.assertEqual(SESSION_RUNNING, state.session_state)

        controller.update(6.0, 1100)
        self.assertEqual(1000, state.elapsed_ms)
        graph_count = len(state.graph_values)

        action = controller.handle_event(EVENT_TIMER_TOGGLE, 1200)
        self.assertEqual(ACTION_SESSION_STOPPED, action)
        self.assertEqual(SESSION_STOPPED, state.session_state)
        self.assertEqual(1100, state.elapsed_ms)
        final_values = (state.weight_g, state.flow_gps, state.ratio)

        controller.update(0.0, 2000)
        self.assertEqual(final_values, (state.weight_g, state.flow_gps, state.ratio))
        self.assertEqual(graph_count, len(state.graph_values))

        action = controller.handle_event(EVENT_TIMER_TOGGLE, 3000)
        self.assertEqual(ACTION_SESSION_STARTED, action)
        controller.update(7.0, 3500)
        self.assertEqual(1600, state.elapsed_ms)
        self.assertGreaterEqual(len(state.graph_values), graph_count)

        controller.handle_event(EVENT_TIMER_RESET, 4000)
        self.assertEqual(SESSION_IDLE, state.session_state)
        self.assertEqual(0, state.elapsed_ms)
        self.assertEqual(0.0, state.flow_gps)
        self.assertEqual(0.0, state.ratio)
        self.assertEqual(0, len(state.graph_values))

    def test_manual_toggle_while_armed_starts_immediately_and_disarms(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 0)
        controller.handle_event(EVENT_AUTO_CONFIRM, 1)

        action = controller.handle_event(EVENT_TIMER_TOGGLE, 2)

        self.assertEqual(ACTION_SESSION_STARTED, action)
        self.assertEqual(SESSION_RUNNING, state.session_state)
        self.assertFalse(state.auto_enabled)

    def test_elapsed_time_survives_ticks_wrap(self):
        _, state, controller = make_controller()
        start = TICKS_PERIOD - 100

        controller.handle_event(EVENT_TIMER_TOGGLE, start)
        controller.update(1.0, 50)

        self.assertEqual(150, state.elapsed_ms)

    def test_mode_cycles_and_requests_persistence(self):
        _, state, controller = make_controller()

        for expected in (1, 2, 3, MODE_TIMER_WEIGHT):
            action = controller.handle_event(EVENT_MODE_NEXT, 0)
            self.assertEqual(ACTION_SAVE_MODE, action)
            self.assertEqual(expected, state.mode)
        self.assertEqual(MODE_ALL, 3)


class MenuAndTareTests(unittest.TestCase):
    def test_profile_cancel_restores_original_selection(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 100)
        controller.handle_event(EVENT_AUTO_NEXT, 200)
        self.assertEqual(PROFILE_POUR_OVER, state.auto_profile)

        controller.handle_event(EVENT_AUTO_CANCEL, 300)

        self.assertFalse(state.auto_menu_open)
        self.assertEqual(PROFILE_ESPRESSO, state.auto_profile)
        self.assertEqual(250, state.graph_sample_ms)

    def test_confirm_arms_new_session_and_persists_changed_profile(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_TIMER_TOGGLE, 0)
        controller.update(10.0, 500)
        self.assertGreater(len(state.graph_values), 0)
        controller.handle_event(EVENT_TIMER_TOGGLE, 550)

        controller.handle_event(EVENT_AUTO_MENU, 600)
        controller.handle_event(EVENT_AUTO_NEXT, 700)
        action = controller.handle_event(EVENT_AUTO_CONFIRM, 800)

        self.assertEqual(ACTION_SAVE_PROFILE, action)
        self.assertEqual(SESSION_ARMED, state.session_state)
        self.assertTrue(state.auto_enabled)
        self.assertFalse(state.auto_menu_open)
        self.assertEqual(PROFILE_POUR_OVER, state.auto_profile)
        self.assertEqual(0, state.elapsed_ms)
        self.assertEqual(0, len(state.graph_values))
        self.assertEqual(1000, state.graph_sample_ms)

    def test_menu_timeout_is_inclusive_and_restores_profile(self):
        config, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 100)
        controller.handle_event(EVENT_AUTO_NEXT, 200)
        deadline = 200 + config.auto_menu_timeout_ms

        controller.update(0.0, deadline - 1)
        self.assertTrue(state.auto_menu_open)

        controller.update(0.0, deadline)
        self.assertFalse(state.auto_menu_open)
        self.assertEqual(PROFILE_ESPRESSO, state.auto_profile)

    def test_running_tare_is_locked_and_transient_expires(self):
        config, state, controller = make_controller()
        controller.handle_event(EVENT_TIMER_TOGGLE, 0)

        action = controller.handle_event(EVENT_TARE, 100)

        self.assertEqual(ACTION_SHOW_LOCK, action)
        self.assertFalse(action & ACTION_TARE)
        self.assertEqual("LOCK", state.transient_message)

        controller.update(2.0, 100 + config.transient_message_ms - 1)
        self.assertEqual("LOCK", state.transient_message)
        controller.update(2.0, 100 + config.transient_message_ms)
        self.assertEqual("", state.transient_message)

    def test_running_session_cannot_open_auto_menu(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_TIMER_TOGGLE, 0)

        action = controller.handle_event(EVENT_AUTO_MENU, 100)

        self.assertEqual(ACTION_SHOW_LOCK, action)
        self.assertFalse(state.auto_menu_open)
        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_tare_action_and_baseline_reset_when_not_running(self):
        _, state, controller = make_controller()
        controller.update(10.0, 0)

        self.assertEqual(ACTION_TARE, controller.handle_event(EVENT_TARE, 10))
        controller.on_tare(20)

        self.assertEqual(0.0, state.weight_g)
        self.assertEqual(0.0, state.flow_gps)
        self.assertEqual(0.0, state.ratio)

    def test_explicit_transient_lock_event(self):
        _, state, controller = make_controller()
        action = controller.handle_event(EVENT_TRANSIENT_LOCK, 0)
        self.assertEqual(ACTION_SHOW_LOCK, action)
        self.assertEqual("LOCK", state.transient_message)


class MeasurementMathTests(unittest.TestCase):
    def test_ratio_clamps_negative_weight_and_uses_configured_dose(self):
        config, state, controller = make_controller()
        controller.update(-5.0, 0)
        self.assertEqual(0.0, state.ratio)

        controller.update(30.0, 100)
        self.assertEqual(2.0, state.ratio)

        config.dose_g = 0.0
        controller.update(30.0, 200)
        self.assertEqual(0.0, state.ratio)

    def test_linear_ramp_produces_regression_flow(self):
        _, state, controller = make_controller()

        for now_ms in range(0, 2100, 100):
            controller.update(2.0 * now_ms / 1000.0, now_ms)

        self.assertAlmostEqual(2.0, state.flow_gps, delta=0.01)

    def test_small_stationary_noise_stays_near_zero(self):
        _, state, controller = make_controller()

        for index, now_ms in enumerate(range(0, 2100, 100)):
            controller.update(0.01 if index % 2 else -0.01, now_ms)

        self.assertAlmostEqual(0.0, state.flow_gps, delta=0.02)


class GraphTests(unittest.TestCase):
    def test_graph_is_profile_sampled_preallocated_and_capacity_bounded(self):
        config = BrewConfig()
        config.graph_capacity = 3
        _, state, controller = make_controller(config)
        controller.handle_event(EVENT_TIMER_TOGGLE, 0)

        for now_ms in (250, 500, 750, 1000):
            controller.update(now_ms / 1000.0, now_ms)

        self.assertEqual(3, len(state.graph_values))
        self.assertTrue(all(value >= 0.0 for value in state.graph_values))
        graph_before_pause = tuple(state.graph_values)

        controller.handle_event(EVENT_TIMER_TOGGLE, 1100)
        controller.handle_event(EVENT_TIMER_TOGGLE, 2000)
        self.assertEqual(graph_before_pause, tuple(state.graph_values))

        controller.handle_event(EVENT_TIMER_RESET, 2100)
        self.assertEqual(0, len(state.graph_values))

    def test_pour_over_uses_one_second_graph_samples(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state, PROFILE_POUR_OVER)
        self.assertEqual(1, len(state.graph_values))

        controller.update(1.0, started_at + 999)
        self.assertEqual(1, len(state.graph_values))
        controller.update(2.0, started_at + 1000)
        self.assertEqual(2, len(state.graph_values))


class AutoStartTests(unittest.TestCase):
    def test_auto_start_uses_inclusive_weight_and_hold_boundaries(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 0)
        controller.handle_event(EVENT_AUTO_CONFIRM, 0)
        controller.update(0.0, 0)
        controller.update(0.0, 1000)

        controller.update(0.5, 1100)
        controller.update(0.5, 1399)
        self.assertEqual(SESSION_ARMED, state.session_state)

        action = controller.update(0.5, 1400)
        self.assertTrue(action & ACTION_SESSION_STARTED)
        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_start_candidate_resets_below_threshold(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 0)
        controller.handle_event(EVENT_AUTO_CONFIRM, 0)
        controller.update(0.0, 0)
        controller.update(0.0, 1000)

        controller.update(0.5, 1100)
        controller.update(0.49, 1399)
        controller.update(0.5, 1400)
        controller.update(0.5, 1699)
        self.assertEqual(SESSION_ARMED, state.session_state)

        controller.update(0.5, 1700)
        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_unstable_zero_does_not_arm_start_detection(self):
        _, state, controller = make_controller()
        controller.handle_event(EVENT_AUTO_MENU, 0)
        controller.handle_event(EVENT_AUTO_CONFIRM, 0)
        controller.update(-0.3, 0)
        controller.update(0.3, 1000)
        controller.update(0.5, 1300)
        controller.update(0.5, 1700)

        self.assertEqual(SESSION_ARMED, state.session_state)


class EspressoAutoStopTests(unittest.TestCase):
    def test_does_not_stop_below_minimum_weight(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state)

        for now_ms in range(started_at + 100, started_at + 4100, 100):
            controller.update(14.0, now_ms)

        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_three_second_plateau_stops_and_holds_final_values(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state)
        plateau_start = started_at + 100

        controller.update(15.0, plateau_start)
        for now_ms in range(plateau_start + 100, plateau_start + 3000, 100):
            action = controller.update(15.0, now_ms)
            self.assertFalse(action & ACTION_SESSION_STOPPED)

        action = controller.update(15.0, plateau_start + 3000)

        self.assertTrue(action & ACTION_SESSION_STOPPED)
        self.assertEqual(SESSION_STOPPED, state.session_state)
        self.assertFalse(state.auto_enabled)
        self.assertEqual("espresso_plateau", controller.last_stop_reason)
        final = (state.weight_g, state.flow_gps, state.ratio, state.elapsed_ms)
        controller.update(0.0, plateau_start + 4000)
        self.assertEqual(final, (state.weight_g, state.flow_gps, state.ratio, state.elapsed_ms))

    def test_drift_larger_than_plateau_span_is_not_a_stop(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state)
        plateau_start = started_at + 100

        for step in range(31):
            weight = 15.0 + 0.3 * step / 30.0
            controller.update(weight, plateau_start + step * 100)

        self.assertEqual(SESSION_RUNNING, state.session_state)


class PourOverAutoStopTests(unittest.TestCase):
    def test_slow_drop_is_not_a_removal(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state, PROFILE_POUR_OVER)
        controller.update(100.0, started_at + 100)

        for offset, weight in ((600, 95.0), (1100, 90.0), (1600, 85.0), (2100, 80.0)):
            controller.update(weight, started_at + offset)

        self.assertEqual(SESSION_RUNNING, state.session_state)

    def test_brief_drop_recovers_but_persistent_drop_stops(self):
        _, state, controller = make_controller()
        started_at = arm_and_start(controller, state, PROFILE_POUR_OVER)
        controller.update(100.0, started_at + 100)
        controller.update(100.0, started_at + 1000)

        controller.update(80.0, started_at + 1100)
        controller.update(86.0, started_at + 1300)
        controller.update(86.0, started_at + 1800)
        self.assertEqual(SESSION_RUNNING, state.session_state)

        controller.update(100.0, started_at + 1900)
        controller.update(80.0, started_at + 2000)
        controller.update(85.0, started_at + 2499)
        self.assertEqual(SESSION_RUNNING, state.session_state)

        action = controller.update(85.0, started_at + 2500)

        self.assertTrue(action & ACTION_SESSION_STOPPED)
        self.assertEqual(SESSION_STOPPED, state.session_state)
        self.assertFalse(state.auto_enabled)
        self.assertEqual("pour_over_drop", controller.last_stop_reason)


if __name__ == "__main__":
    unittest.main()
