"""Shared, device-independent contracts for the scale application.

This module intentionally avoids device-only imports so the state machine,
display renderer, and adapters can share the same vocabulary on MicroPython
and in host-side tests.
"""


# Display modes.
MODE_TIMER_WEIGHT = 0
MODE_TIMER_FLOW_WEIGHT = 1
MODE_TIMER_RATIO_WEIGHT = 2
MODE_ALL = 3


# Measurement session states.
SESSION_IDLE = 0
SESSION_ARMED = 1
SESSION_RUNNING = 2
SESSION_STOPPED = 3


# Automatic measurement profiles.
PROFILE_ESPRESSO = 0
PROFILE_POUR_OVER = 1


# Normalized input and BLE events.
EVENT_NONE = 0
EVENT_TARE = 1
EVENT_MODE_NEXT = 2
EVENT_TIMER_TOGGLE = 3
EVENT_TIMER_RESET = 4
EVENT_AUTO_MENU = 5
EVENT_AUTO_NEXT = 6
EVENT_AUTO_CONFIRM = 7
EVENT_AUTO_CANCEL = 8
EVENT_TRANSIENT_LOCK = 9
EVENT_TIMER_START = 10
EVENT_TIMER_STOP = 11


# Controller action flags. Multiple actions may be ORed together.
ACTION_NONE = 0
ACTION_TARE = 1 << 0
ACTION_SAVE_MODE = 1 << 1
ACTION_SAVE_PROFILE = 1 << 2
ACTION_SESSION_STARTED = 1 << 3
ACTION_SESSION_STOPPED = 1 << 4
ACTION_SHOW_LOCK = 1 << 5


class BrewConfig:
    """Mutable configuration for measurement, input, and display behavior."""

    __slots__ = (
        "dose_g",
        "arm_zero_tolerance_g",
        "arm_stable_window_ms",
        "arm_stable_max_span_g",
        "auto_start_delta_g",
        "auto_start_hold_ms",
        "flow_window_ms",
        "flow_ema_alpha",
        "espresso_min_weight_g",
        "espresso_stop_flow_gps",
        "espresso_stop_window_ms",
        "espresso_stop_max_change_g",
        "pour_over_drop_g",
        "pour_over_drop_window_ms",
        "pour_over_below_peak_g",
        "pour_over_stop_hold_ms",
        "espresso_graph_span_ms",
        "espresso_graph_max_gps",
        "espresso_graph_sample_ms",
        "pour_over_graph_span_ms",
        "pour_over_graph_max_gps",
        "pour_over_graph_sample_ms",
        "graph_capacity",
        "debounce_ms",
        "long_press_ms",
        "combo_hold_ms",
        "auto_menu_timeout_ms",
        "transient_message_ms",
        "tare_min_wait_ms",
        "tare_stable_window_ms",
        "tare_stable_max_span_g",
        "tare_timeout_ms",
        "display_refresh_ms",
        "ble_publish_ms",
    )

    def __init__(self):
        # Coffee dose used by the on-screen 1:x ratio. App configuration is a
        # later milestone, so V2 intentionally starts with one fixed default.
        self.dose_g = 15.0

        # Auto mode first waits for a quiet, near-zero platform. Once stable,
        # a small sustained increase marks the first liquid reaching the cup.
        self.arm_zero_tolerance_g = 0.3
        self.arm_stable_window_ms = 1000
        self.arm_stable_max_span_g = 0.2
        self.auto_start_delta_g = 0.5
        self.auto_start_hold_ms = 300

        # Flow is the slope of recent weight samples; EMA prevents individual
        # load-cell readings from making the numeric value and graph flicker.
        self.flow_window_ms = 1000
        self.flow_ema_alpha = 0.35

        # Espresso may stop only after reaching 15 g and remaining effectively
        # flat for the full window. This avoids stopping during early pauses.
        self.espresso_min_weight_g = 15.0
        self.espresso_stop_flow_gps = 0.10
        self.espresso_stop_window_ms = 3000
        self.espresso_stop_max_change_g = 0.2

        # Pour-over stops when lifting the dripper/server causes a sudden drop.
        # `pour_over_drop_g` is the minimum loss inside the short detection
        # window, not a target beverage weight. The reading must then stay at
        # least `pour_over_below_peak_g` below the captured peak for the hold
        # period, which rejects a single noisy HX711 sample.
        self.pour_over_drop_g = 20.0
        self.pour_over_drop_window_ms = 1000
        self.pour_over_below_peak_g = 15.0
        self.pour_over_stop_hold_ms = 500

        self.espresso_graph_span_ms = 45000
        self.espresso_graph_max_gps = 6.0
        self.espresso_graph_sample_ms = 250
        self.pour_over_graph_span_ms = 240000
        self.pour_over_graph_max_gps = 12.0
        self.pour_over_graph_sample_ms = 1000
        self.graph_capacity = 256

        self.debounce_ms = 40
        self.long_press_ms = 1000
        self.combo_hold_ms = 1000
        self.auto_menu_timeout_ms = 10000
        self.transient_message_ms = 1000

        # The physical tare button is mounted on the weighing surface. Wait
        # after its release, then require a quiet platform before sampling the
        # HX711 offset so button force and mechanical rebound are excluded.
        self.tare_min_wait_ms = 350
        self.tare_stable_window_ms = 300
        self.tare_stable_max_span_g = 0.2
        self.tare_timeout_ms = 2000

        self.display_refresh_ms = 100
        self.ble_publish_ms = 100

    def graph_span_for(self, profile):
        if profile == PROFILE_POUR_OVER:
            return self.pour_over_graph_span_ms
        return self.espresso_graph_span_ms

    def graph_max_for(self, profile):
        if profile == PROFILE_POUR_OVER:
            return self.pour_over_graph_max_gps
        return self.espresso_graph_max_gps

    def graph_sample_for(self, profile):
        if profile == PROFILE_POUR_OVER:
            return self.pour_over_graph_sample_ms
        return self.espresso_graph_sample_ms


class EventQueue:
    """Fixed-capacity FIFO that rejects the newest event when full.

    Storage is allocated once during construction. ``push`` and ``pop`` do
    not grow or shrink the backing list, making the queue appropriate for a
    lightweight BLE callback feeding the main application loop.
    """

    __slots__ = ("_items", "_capacity", "_head", "_tail", "_size", "dropped")

    def __init__(self, capacity=16):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._items = [EVENT_NONE] * capacity
        self._capacity = capacity
        self._head = 0
        self._tail = 0
        self._size = 0
        self.dropped = 0

    def __len__(self):
        return self._size

    def __bool__(self):
        return self._size != 0

    @property
    def capacity(self):
        return self._capacity

    def push(self, event):
        if self._size == self._capacity:
            self.dropped += 1
            return False

        self._items[self._tail] = event
        self._tail = (self._tail + 1) % self._capacity
        self._size += 1
        return True

    def pop(self):
        if self._size == 0:
            return EVENT_NONE

        event = self._items[self._head]
        self._items[self._head] = EVENT_NONE
        self._head = (self._head + 1) % self._capacity
        self._size -= 1
        return event

    def clear(self):
        while self._size:
            self._items[self._head] = EVENT_NONE
            self._head = (self._head + 1) % self._capacity
            self._size -= 1
        self._tail = self._head


class DisplayState:
    """Mutable presentation state shared by controller and renderer."""

    __slots__ = (
        "mode",
        "session_state",
        "auto_profile",
        "auto_enabled",
        "auto_menu_open",
        "auto_menu_deadline_ms",
        "elapsed_ms",
        "weight_g",
        "flow_gps",
        "ratio",
        "battery_percent",
        "ble_connected",
        "graph_values",
        "graph_sample_ms",
        "graph_span_ms",
        "graph_max_gps",
        "transient_message",
        "transient_until_ms",
    )

    def __init__(self, config=None):
        if config is None:
            config = BrewConfig()

        self.mode = MODE_TIMER_WEIGHT
        self.session_state = SESSION_IDLE
        self.auto_profile = PROFILE_ESPRESSO
        self.auto_enabled = False
        self.auto_menu_open = False
        self.auto_menu_deadline_ms = 0

        self.elapsed_ms = 0
        self.weight_g = 0.0
        self.flow_gps = 0.0
        self.ratio = 0.0
        self.battery_percent = 100
        self.ble_connected = False

        # The measurement controller replaces this immutable empty sequence
        # with its chronological graph view.
        self.graph_values = ()
        self.graph_sample_ms = config.graph_sample_for(self.auto_profile)
        self.graph_span_ms = config.graph_span_for(self.auto_profile)
        self.graph_max_gps = config.graph_max_for(self.auto_profile)

        self.transient_message = ""
        self.transient_until_ms = 0
