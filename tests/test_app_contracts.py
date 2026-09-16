import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

import app_contracts as contracts


class ConstantsTests(unittest.TestCase):
    def test_constant_groups_are_unique(self):
        modes = {
            contracts.MODE_TIMER_WEIGHT,
            contracts.MODE_TIMER_FLOW_WEIGHT,
            contracts.MODE_TIMER_RATIO_WEIGHT,
            contracts.MODE_ALL,
        }
        states = {
            contracts.SESSION_IDLE,
            contracts.SESSION_ARMED,
            contracts.SESSION_RUNNING,
            contracts.SESSION_STOPPED,
        }
        profiles = {
            contracts.PROFILE_ESPRESSO,
            contracts.PROFILE_POUR_OVER,
        }
        events = {
            contracts.EVENT_TARE,
            contracts.EVENT_MODE_NEXT,
            contracts.EVENT_TIMER_TOGGLE,
            contracts.EVENT_TIMER_RESET,
            contracts.EVENT_AUTO_MENU,
            contracts.EVENT_AUTO_NEXT,
            contracts.EVENT_AUTO_CONFIRM,
            contracts.EVENT_AUTO_CANCEL,
            contracts.EVENT_TRANSIENT_LOCK,
            contracts.EVENT_TIMER_START,
            contracts.EVENT_TIMER_STOP,
        }

        self.assertEqual(4, len(modes))
        self.assertEqual(4, len(states))
        self.assertEqual(2, len(profiles))
        self.assertEqual(11, len(events))
        self.assertNotIn(contracts.EVENT_NONE, events)

    def test_action_flags_do_not_overlap(self):
        actions = (
            contracts.ACTION_TARE,
            contracts.ACTION_SAVE_MODE,
            contracts.ACTION_SAVE_PROFILE,
            contracts.ACTION_SESSION_STARTED,
            contracts.ACTION_SESSION_STOPPED,
            contracts.ACTION_SHOW_LOCK,
        )

        self.assertEqual(0, contracts.ACTION_NONE)
        self.assertEqual(len(actions), len(set(actions)))
        for action in actions:
            self.assertEqual(1, action.bit_count())


class BrewConfigTests(unittest.TestCase):
    def test_agreed_defaults(self):
        config = contracts.BrewConfig()

        self.assertEqual(15.0, config.dose_g)
        self.assertEqual(1000, config.arm_stable_window_ms)
        self.assertEqual(0.2, config.arm_stable_max_span_g)
        self.assertEqual(0.5, config.auto_start_delta_g)
        self.assertEqual(300, config.auto_start_hold_ms)
        self.assertEqual(15.0, config.espresso_min_weight_g)
        self.assertEqual(0.10, config.espresso_stop_flow_gps)
        self.assertEqual(3000, config.espresso_stop_window_ms)
        self.assertEqual(20.0, config.pour_over_drop_g)
        self.assertEqual(1000, config.pour_over_drop_window_ms)
        self.assertEqual(500, config.pour_over_stop_hold_ms)
        self.assertEqual(45000, config.espresso_graph_span_ms)
        self.assertEqual(250, config.espresso_graph_sample_ms)
        self.assertEqual(240000, config.pour_over_graph_span_ms)
        self.assertEqual(1000, config.pour_over_graph_sample_ms)
        self.assertEqual(256, config.graph_capacity)
        self.assertEqual(10000, config.auto_menu_timeout_ms)

    def test_profile_graph_helpers(self):
        config = contracts.BrewConfig()

        self.assertEqual(
            config.espresso_graph_span_ms,
            config.graph_span_for(contracts.PROFILE_ESPRESSO),
        )
        self.assertEqual(
            config.pour_over_graph_span_ms,
            config.graph_span_for(contracts.PROFILE_POUR_OVER),
        )
        self.assertEqual(
            config.espresso_graph_max_gps,
            config.graph_max_for(contracts.PROFILE_ESPRESSO),
        )
        self.assertEqual(
            config.pour_over_graph_max_gps,
            config.graph_max_for(contracts.PROFILE_POUR_OVER),
        )
        self.assertEqual(
            config.espresso_graph_sample_ms,
            config.graph_sample_for(contracts.PROFILE_ESPRESSO),
        )
        self.assertEqual(
            config.pour_over_graph_sample_ms,
            config.graph_sample_for(contracts.PROFILE_POUR_OVER),
        )


class EventQueueTests(unittest.TestCase):
    def test_preserves_order_across_wraparound(self):
        queue = contracts.EventQueue(3)

        self.assertTrue(queue.push(contracts.EVENT_TARE))
        self.assertTrue(queue.push(contracts.EVENT_MODE_NEXT))
        self.assertEqual(contracts.EVENT_TARE, queue.pop())
        self.assertTrue(queue.push(contracts.EVENT_TIMER_TOGGLE))
        self.assertTrue(queue.push(contracts.EVENT_TIMER_RESET))

        self.assertEqual(contracts.EVENT_MODE_NEXT, queue.pop())
        self.assertEqual(contracts.EVENT_TIMER_TOGGLE, queue.pop())
        self.assertEqual(contracts.EVENT_TIMER_RESET, queue.pop())
        self.assertEqual(contracts.EVENT_NONE, queue.pop())
        self.assertFalse(queue)

    def test_rejects_newest_event_when_full(self):
        queue = contracts.EventQueue(2)
        queue.push(contracts.EVENT_TARE)
        queue.push(contracts.EVENT_MODE_NEXT)

        self.assertFalse(queue.push(contracts.EVENT_TIMER_RESET))
        self.assertEqual(1, queue.dropped)
        self.assertEqual(2, len(queue))
        self.assertEqual(contracts.EVENT_TARE, queue.pop())
        self.assertEqual(contracts.EVENT_MODE_NEXT, queue.pop())

    def test_clear_keeps_queue_reusable(self):
        queue = contracts.EventQueue(2)
        queue.push(contracts.EVENT_TARE)
        queue.push(contracts.EVENT_MODE_NEXT)

        queue.clear()

        self.assertEqual(0, len(queue))
        self.assertTrue(queue.push(contracts.EVENT_AUTO_MENU))
        self.assertEqual(contracts.EVENT_AUTO_MENU, queue.pop())

    def test_requires_positive_capacity(self):
        with self.assertRaises(ValueError):
            contracts.EventQueue(0)


class DisplayStateTests(unittest.TestCase):
    def test_defaults_match_first_boot(self):
        state = contracts.DisplayState()

        self.assertEqual(contracts.MODE_TIMER_WEIGHT, state.mode)
        self.assertEqual(contracts.SESSION_IDLE, state.session_state)
        self.assertEqual(contracts.PROFILE_ESPRESSO, state.auto_profile)
        self.assertFalse(state.auto_enabled)
        self.assertFalse(state.auto_menu_open)
        self.assertEqual(0, state.elapsed_ms)
        self.assertEqual(0.0, state.weight_g)
        self.assertEqual(0.0, state.flow_gps)
        self.assertEqual(0.0, state.ratio)
        self.assertEqual(100, state.battery_percent)
        self.assertFalse(state.ble_connected)
        self.assertEqual((), state.graph_values)
        self.assertEqual(250, state.graph_sample_ms)
        self.assertEqual(45000, state.graph_span_ms)
        self.assertEqual(6.0, state.graph_max_gps)
        self.assertEqual("", state.transient_message)
        self.assertEqual(0, state.transient_until_ms)

    def test_uses_supplied_graph_config(self):
        config = contracts.BrewConfig()
        config.espresso_graph_sample_ms = 500
        config.espresso_graph_span_ms = 30000
        config.espresso_graph_max_gps = 8.0

        state = contracts.DisplayState(config)

        self.assertEqual(500, state.graph_sample_ms)
        self.assertEqual(30000, state.graph_span_ms)
        self.assertEqual(8.0, state.graph_max_gps)


if __name__ == "__main__":
    unittest.main()
