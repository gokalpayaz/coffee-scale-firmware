"""Pure measurement state machine for the coffee scale.

All time values are supplied by the caller as ``ticks_ms``-style integers.
The module has no hardware imports and is usable from both MicroPython and
host-side CPython tests.
"""

from app_contracts import (
    ACTION_NONE,
    ACTION_SAVE_MODE,
    ACTION_SAVE_PROFILE,
    ACTION_SESSION_STARTED,
    ACTION_SESSION_STOPPED,
    ACTION_SHOW_LOCK,
    ACTION_TARE,
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
    PROFILE_ESPRESSO,
    PROFILE_POUR_OVER,
    SESSION_ARMED,
    SESSION_IDLE,
    SESSION_RUNNING,
    SESSION_STOPPED,
)


_TICKS_PERIOD = 1 << 30
_TICKS_HALF_PERIOD = _TICKS_PERIOD // 2
_HISTORY_SAMPLE_MS = 25


def _ticks_diff(new, old):
    return ((new - old + _TICKS_HALF_PERIOD) % _TICKS_PERIOD) - _TICKS_HALF_PERIOD


def _ticks_add(value, delta):
    return (value + delta) % _TICKS_PERIOD


class _GraphView:
    """Read-only chronological view over the controller's graph ring."""

    __slots__ = ("_controller",)

    def __init__(self, controller):
        self._controller = controller

    def __len__(self):
        return self._controller._graph_count

    def __getitem__(self, index):
        count = self._controller._graph_count
        if index < 0:
            index += count
        if index < 0 or index >= count:
            raise IndexError(index)

        start = (
            self._controller._graph_next - count
        ) % self._controller._graph_capacity
        return self._controller._graph_values[
            (start + index) % self._controller._graph_capacity
        ]


class MeasurementController:
    """Own timer, flow, ratio, graph, menu, and automatic session behavior."""

    def __init__(self, config, state):
        self.config = config
        self.state = state

        self._elapsed_before_run_ms = 0
        self._run_started_ms = 0

        self._history_capacity = max(8, config.graph_capacity)
        self._history_times = [0] * self._history_capacity
        self._history_weights = [0.0] * self._history_capacity
        self._history_next = 0
        self._history_count = 0
        self._last_history_ms = None

        self._flow_ema = 0.0
        self._flow_initialized = False

        self._graph_capacity = max(1, config.graph_capacity)
        self._graph_values = [0.0] * self._graph_capacity
        self._graph_next = 0
        self._graph_count = 0
        self._last_graph_ms = None
        self.state.graph_values = _GraphView(self)

        self._menu_original_profile = state.auto_profile
        self._arm_stable = False
        self._arm_baseline_g = 0.0
        self._auto_start_candidate_ms = None

        self._session_peak_g = state.weight_g
        self._pour_drop_candidate_ms = None
        self._pour_reference_peak_g = state.weight_g
        self.last_stop_reason = ""

        self._apply_profile_display_settings()

    def handle_event(self, event, now_ms):
        """Handle one normalized event and return controller action flags."""
        self._expire_transient(now_ms)

        if event == EVENT_TARE:
            if self.state.session_state == SESSION_RUNNING:
                self._show_lock(now_ms)
                return ACTION_SHOW_LOCK
            return ACTION_TARE

        if event == EVENT_TRANSIENT_LOCK:
            self._show_lock(now_ms)
            return ACTION_SHOW_LOCK

        if event == EVENT_MODE_NEXT:
            self.state.mode = (self.state.mode + 1) % (MODE_ALL + 1)
            return ACTION_SAVE_MODE

        if event == EVENT_TIMER_TOGGLE:
            if self.state.session_state == SESSION_RUNNING:
                return self._stop_running(now_ms)

            if self.state.session_state == SESSION_IDLE:
                return self._start_running(now_ms, new_session=True)

            # STOPPED resumes without clearing elapsed time or graph. ARMED
            # becomes an immediate manual start and abandons automatic mode.
            self.state.auto_enabled = False
            return self._start_running(now_ms, new_session=False)

        if event == EVENT_TIMER_START:
            if self.state.session_state == SESSION_RUNNING:
                return ACTION_NONE
            if self.state.session_state == SESSION_IDLE:
                return self._start_running(now_ms, new_session=True)
            self.state.auto_enabled = False
            return self._start_running(now_ms, new_session=False)

        if event == EVENT_TIMER_STOP:
            if self.state.session_state == SESSION_RUNNING:
                return self._stop_running(now_ms)
            return ACTION_NONE

        if event == EVENT_TIMER_RESET:
            self._reset_session(now_ms)
            self.state.session_state = SESSION_IDLE
            self.state.auto_enabled = False
            self._close_auto_menu(restore_profile=True)
            return ACTION_NONE

        if event == EVENT_AUTO_MENU:
            if self.state.session_state == SESSION_RUNNING:
                self._show_lock(now_ms)
                return ACTION_SHOW_LOCK
            if not self.state.auto_menu_open:
                self._menu_original_profile = self.state.auto_profile
            self.state.auto_menu_open = True
            self.state.auto_menu_deadline_ms = _ticks_add(
                now_ms, self.config.auto_menu_timeout_ms
            )
            return ACTION_NONE

        if event == EVENT_AUTO_NEXT and self.state.auto_menu_open:
            if self.state.auto_profile == PROFILE_ESPRESSO:
                self.state.auto_profile = PROFILE_POUR_OVER
            else:
                self.state.auto_profile = PROFILE_ESPRESSO
            self._apply_profile_display_settings()
            self.state.auto_menu_deadline_ms = _ticks_add(
                now_ms, self.config.auto_menu_timeout_ms
            )
            return ACTION_NONE

        if event == EVENT_AUTO_CONFIRM and self.state.auto_menu_open:
            profile_changed = self.state.auto_profile != self._menu_original_profile
            self._close_auto_menu(restore_profile=False)
            self._reset_session(now_ms)
            self.state.session_state = SESSION_ARMED
            self.state.auto_enabled = True
            if profile_changed:
                return ACTION_SAVE_PROFILE
            return ACTION_NONE

        if event == EVENT_AUTO_CANCEL and self.state.auto_menu_open:
            self._close_auto_menu(restore_profile=True)
            return ACTION_NONE

        return ACTION_NONE

    def update(self, weight_g, now_ms):
        """Process a weight sample and return any automatic action flags."""
        self._expire_transient(now_ms)
        self._expire_auto_menu(now_ms)

        # A stopped session intentionally freezes its final measurement and
        # graph until reset, resume, tare, or a newly confirmed auto session.
        if self.state.session_state == SESSION_STOPPED:
            return ACTION_NONE

        self.state.weight_g = float(weight_g)
        self.state.ratio = self._ratio_for(weight_g)

        recorded = self._record_weight(float(weight_g), now_ms)
        if recorded:
            flow = self._linear_regression_flow(now_ms)
            if flow is not None:
                if self._flow_initialized:
                    alpha = self.config.flow_ema_alpha
                    self._flow_ema = alpha * flow + (1.0 - alpha) * self._flow_ema
                else:
                    self._flow_ema = flow
                    self._flow_initialized = True
                self.state.flow_gps = self._flow_ema

        if self.state.session_state == SESSION_ARMED and self.state.auto_enabled:
            return self._update_armed(weight_g, now_ms)

        if self.state.session_state != SESSION_RUNNING:
            return ACTION_NONE

        self._update_elapsed(now_ms)
        self._sample_graph(now_ms)
        if weight_g > self._session_peak_g:
            self._session_peak_g = weight_g

        if not self.state.auto_enabled:
            return ACTION_NONE

        if self.state.auto_profile == PROFILE_POUR_OVER:
            return self._update_pour_over_stop(weight_g, now_ms)
        return self._update_espresso_stop(weight_g, now_ms)

    def on_tare(self, now_ms):
        """Reset the measurement baseline after the caller tares the HX711."""
        self.state.weight_g = 0.0
        self.state.flow_gps = 0.0
        self.state.ratio = 0.0
        self.state.transient_message = ""
        self.state.transient_until_ms = 0
        self._clear_weight_history()
        self._record_weight(0.0, now_ms)
        self._flow_ema = 0.0
        self._flow_initialized = False
        self._reset_auto_detection()

    def _ratio_for(self, weight_g):
        if self.config.dose_g <= 0:
            return 0.0
        return max(0.0, float(weight_g)) / self.config.dose_g

    def _show_lock(self, now_ms):
        self.state.transient_message = "LOCK"
        self.state.transient_until_ms = _ticks_add(
            now_ms, self.config.transient_message_ms
        )

    def _expire_transient(self, now_ms):
        if self.state.transient_message and _ticks_diff(
            now_ms, self.state.transient_until_ms
        ) >= 0:
            self.state.transient_message = ""
            self.state.transient_until_ms = 0

    def _expire_auto_menu(self, now_ms):
        if self.state.auto_menu_open and _ticks_diff(
            now_ms, self.state.auto_menu_deadline_ms
        ) >= 0:
            self._close_auto_menu(restore_profile=True)

    def _close_auto_menu(self, restore_profile):
        if restore_profile:
            self.state.auto_profile = self._menu_original_profile
            self._apply_profile_display_settings()
        self.state.auto_menu_open = False
        self.state.auto_menu_deadline_ms = 0

    def _apply_profile_display_settings(self):
        profile = self.state.auto_profile
        self.state.graph_sample_ms = self.config.graph_sample_for(profile)
        self.state.graph_span_ms = self.config.graph_span_for(profile)
        self.state.graph_max_gps = self.config.graph_max_for(profile)

    def _reset_session(self, now_ms):
        self._elapsed_before_run_ms = 0
        self._run_started_ms = now_ms
        self.state.elapsed_ms = 0
        self.state.flow_gps = 0.0
        self.state.ratio = 0.0
        self._flow_ema = 0.0
        self._flow_initialized = False
        self.last_stop_reason = ""
        self._clear_weight_history()
        self._clear_graph()
        self._reset_auto_detection()

    def _start_running(self, now_ms, new_session):
        if new_session:
            self._reset_session(now_ms)

        self.state.session_state = SESSION_RUNNING
        self._run_started_ms = now_ms
        self._clear_weight_history()
        self._record_weight(self.state.weight_g, now_ms)
        self._flow_ema = 0.0
        self._flow_initialized = False
        self.state.flow_gps = 0.0
        self._session_peak_g = self.state.weight_g
        self._pour_drop_candidate_ms = None
        self._pour_reference_peak_g = self.state.weight_g
        self._last_graph_ms = now_ms
        if self._graph_count == 0:
            self._append_graph(0.0)
        return ACTION_SESSION_STARTED

    def _stop_running(self, now_ms, reason="manual"):
        self._update_elapsed(now_ms)
        self._elapsed_before_run_ms = self.state.elapsed_ms
        self.state.session_state = SESSION_STOPPED
        self.state.auto_enabled = False
        self.last_stop_reason = reason
        self._auto_start_candidate_ms = None
        self._pour_drop_candidate_ms = None
        return ACTION_SESSION_STOPPED

    def _update_elapsed(self, now_ms):
        running_ms = _ticks_diff(now_ms, self._run_started_ms)
        if running_ms < 0:
            running_ms = 0
        self.state.elapsed_ms = self._elapsed_before_run_ms + running_ms

    def _reset_auto_detection(self):
        self._arm_stable = False
        self._arm_baseline_g = 0.0
        self._auto_start_candidate_ms = None
        self._session_peak_g = self.state.weight_g
        self._pour_drop_candidate_ms = None
        self._pour_reference_peak_g = self.state.weight_g

    def _update_armed(self, weight_g, now_ms):
        if not self._arm_stable:
            stable, baseline = self._zero_is_stable(now_ms)
            if stable:
                self._arm_stable = True
                self._arm_baseline_g = baseline
            return ACTION_NONE

        threshold = self._arm_baseline_g + self.config.auto_start_delta_g
        if weight_g >= threshold:
            if self._auto_start_candidate_ms is None:
                self._auto_start_candidate_ms = now_ms
            elif _ticks_diff(now_ms, self._auto_start_candidate_ms) >= (
                self.config.auto_start_hold_ms
            ):
                return self._start_running(now_ms, new_session=False)
        else:
            self._auto_start_candidate_ms = None
        return ACTION_NONE

    def _zero_is_stable(self, now_ms):
        count = 0
        total = 0.0
        minimum = None
        maximum = None
        oldest_age = 0

        for logical_index in range(self._history_count):
            index = self._history_index(logical_index)
            age = _ticks_diff(now_ms, self._history_times[index])
            if age < 0 or age > self.config.arm_stable_window_ms:
                continue
            value = self._history_weights[index]
            if abs(value) > self.config.arm_zero_tolerance_g:
                return False, 0.0
            if minimum is None or value < minimum:
                minimum = value
            if maximum is None or value > maximum:
                maximum = value
            if age > oldest_age:
                oldest_age = age
            total += value
            count += 1

        if count < 2 or oldest_age < self.config.arm_stable_window_ms:
            return False, 0.0
        if maximum - minimum > self.config.arm_stable_max_span_g:
            return False, 0.0
        return True, total / count

    def _update_espresso_stop(self, weight_g, now_ms):
        if weight_g < self.config.espresso_min_weight_g:
            return ACTION_NONE

        has_window, span = self._weight_span(
            now_ms, self.config.espresso_stop_window_ms
        )
        if not has_window:
            return ACTION_NONE
        if span > self.config.espresso_stop_max_change_g:
            return ACTION_NONE
        if abs(self.state.flow_gps) > self.config.espresso_stop_flow_gps:
            return ACTION_NONE
        return self._stop_running(now_ms, "espresso_plateau")

    def _update_pour_over_stop(self, weight_g, now_ms):
        if weight_g > self._session_peak_g:
            self._session_peak_g = weight_g

        # Phase one detects a fast removal-sized drop. Phase two below keeps
        # the captured peak fixed and requires the lower reading to persist.
        if self._pour_drop_candidate_ms is None:
            recent_max = self._recent_max(
                now_ms, self.config.pour_over_drop_window_ms
            )
            if (
                recent_max is not None
                and recent_max - weight_g >= self.config.pour_over_drop_g
                and self._session_peak_g - weight_g >= self.config.pour_over_drop_g
            ):
                self._pour_drop_candidate_ms = now_ms
                self._pour_reference_peak_g = self._session_peak_g
            return ACTION_NONE

        if weight_g > (
            self._pour_reference_peak_g - self.config.pour_over_below_peak_g
        ):
            self._pour_drop_candidate_ms = None
            return ACTION_NONE

        if _ticks_diff(now_ms, self._pour_drop_candidate_ms) >= (
            self.config.pour_over_stop_hold_ms
        ):
            return self._stop_running(now_ms, "pour_over_drop")
        return ACTION_NONE

    def _record_weight(self, weight_g, now_ms):
        if self._last_history_ms is not None and _ticks_diff(
            now_ms, self._last_history_ms
        ) < _HISTORY_SAMPLE_MS:
            return False

        self._history_times[self._history_next] = now_ms
        self._history_weights[self._history_next] = weight_g
        self._history_next = (self._history_next + 1) % self._history_capacity
        if self._history_count < self._history_capacity:
            self._history_count += 1
        self._last_history_ms = now_ms
        return True

    def _clear_weight_history(self):
        self._history_next = 0
        self._history_count = 0
        self._last_history_ms = None

    def _history_index(self, logical_index):
        start = (self._history_next - self._history_count) % self._history_capacity
        return (start + logical_index) % self._history_capacity

    def _linear_regression_flow(self, now_ms):
        sample_count = 0
        sum_x = 0.0
        sum_y = 0.0
        sum_xx = 0.0
        sum_xy = 0.0

        for logical_index in range(self._history_count):
            index = self._history_index(logical_index)
            age_ms = _ticks_diff(now_ms, self._history_times[index])
            if age_ms < 0 or age_ms > self.config.flow_window_ms:
                continue
            x = -age_ms / 1000.0
            y = self._history_weights[index]
            sample_count += 1
            sum_x += x
            sum_y += y
            sum_xx += x * x
            sum_xy += x * y

        if sample_count < 2:
            return None
        denominator = sample_count * sum_xx - sum_x * sum_x
        if denominator == 0:
            return None
        return (sample_count * sum_xy - sum_x * sum_y) / denominator

    def _weight_span(self, now_ms, window_ms):
        minimum = None
        maximum = None
        oldest_age = 0

        for logical_index in range(self._history_count):
            index = self._history_index(logical_index)
            age = _ticks_diff(now_ms, self._history_times[index])
            if age < 0 or age > window_ms:
                continue
            value = self._history_weights[index]
            if minimum is None or value < minimum:
                minimum = value
            if maximum is None or value > maximum:
                maximum = value
            if age > oldest_age:
                oldest_age = age

        if minimum is None or oldest_age < window_ms:
            return False, 0.0
        return True, maximum - minimum

    def _recent_max(self, now_ms, window_ms):
        maximum = None
        for logical_index in range(self._history_count):
            index = self._history_index(logical_index)
            age = _ticks_diff(now_ms, self._history_times[index])
            if age < 0 or age > window_ms:
                continue
            value = self._history_weights[index]
            if maximum is None or value > maximum:
                maximum = value
        return maximum

    def _sample_graph(self, now_ms):
        if self._last_graph_ms is None:
            self._last_graph_ms = now_ms
            return
        if _ticks_diff(now_ms, self._last_graph_ms) < self.state.graph_sample_ms:
            return
        self._append_graph(max(0.0, self.state.flow_gps))
        self._last_graph_ms = now_ms

    def _append_graph(self, flow_gps):
        self._graph_values[self._graph_next] = flow_gps
        self._graph_next = (self._graph_next + 1) % self._graph_capacity
        if self._graph_count < self._graph_capacity:
            self._graph_count += 1

    def _clear_graph(self):
        self._graph_next = 0
        self._graph_count = 0
        self._last_graph_ms = None
