"""Polled two-button gesture interpreter.

The interpreter is independent of ``machine.Pin`` so physical buttons and
digital touch modules can be aggregated by the application before polling.
"""

import time

from app_contracts import (
    BrewConfig,
    EVENT_AUTO_CANCEL,
    EVENT_AUTO_CONFIRM,
    EVENT_AUTO_MENU,
    EVENT_AUTO_NEXT,
    EVENT_MODE_NEXT,
    EVENT_TARE,
    EVENT_TIMER_RESET,
    EVENT_TIMER_TOGGLE,
)


def _ticks_diff(now_ms, then_ms):
    ticks_diff = getattr(time, "ticks_diff", None)
    if ticks_diff is not None:
        return ticks_diff(now_ms, then_ms)
    return now_ms - then_ms


class _ButtonState:
    __slots__ = (
        "raw_pressed",
        "stable_pressed",
        "raw_changed_ms",
        "pressed_ms",
        "suppress_release",
        "initialized",
    )

    def __init__(self):
        self.raw_pressed = False
        self.stable_pressed = False
        self.raw_changed_ms = 0
        self.pressed_ms = None
        self.suppress_release = False
        self.initialized = False

    def sample(self, pressed, now_ms, debounce_ms):
        pressed = bool(pressed)

        if not self.initialized:
            self.initialized = True
            self.raw_pressed = pressed
            self.raw_changed_ms = now_ms
        elif pressed != self.raw_pressed:
            self.raw_pressed = pressed
            self.raw_changed_ms = now_ms

        if (
            self.raw_pressed != self.stable_pressed
            and _ticks_diff(now_ms, self.raw_changed_ms) >= debounce_ms
        ):
            self.stable_pressed = self.raw_pressed
            return self.stable_pressed

        return None


class InputInterpreter:
    """Convert debounced left/right levels into normalized application events."""

    __slots__ = (
        "debounce_ms",
        "long_press_ms",
        "combo_hold_ms",
        "_left",
        "_right",
        "_combo_started_ms",
        "_combo_emitted",
    )

    def __init__(self, config=None):
        if config is None:
            config = BrewConfig()

        self.debounce_ms = config.debounce_ms
        self.long_press_ms = config.long_press_ms
        self.combo_hold_ms = config.combo_hold_ms
        self._left = _ButtonState()
        self._right = _ButtonState()
        self._combo_started_ms = None
        self._combo_emitted = False

    @property
    def right_is_pressed(self):
        """Expose debounced right-button state for measurement suppression."""

        return self._right.stable_pressed

    def update(self, left_pressed, right_pressed, menu_open, now_ms):
        """Poll aggregate levels and return zero or more normalized events.

        Short and long single-button gestures are emitted on debounced release.
        A two-button gesture is emitted as soon as both buttons have remained
        debounced for ``combo_hold_ms``. Releases following a chord are ignored.
        """

        left_transition = self._left.sample(
            left_pressed, now_ms, self.debounce_ms
        )
        right_transition = self._right.sample(
            right_pressed, now_ms, self.debounce_ms
        )

        if left_transition is True:
            self._left.pressed_ms = now_ms
            self._left.suppress_release = False
        if right_transition is True:
            self._right.pressed_ms = now_ms
            self._right.suppress_release = False

        events = []

        if self._left.stable_pressed and self._right.stable_pressed:
            if self._combo_started_ms is None:
                self._combo_started_ms = now_ms
            elif (
                not self._combo_emitted
                and _ticks_diff(now_ms, self._combo_started_ms)
                >= self.combo_hold_ms
            ):
                events.append(EVENT_AUTO_CANCEL if menu_open else EVENT_AUTO_MENU)
                self._combo_emitted = True
                self._left.suppress_release = True
                self._right.suppress_release = True
        else:
            self._combo_started_ms = None

        if left_transition is False:
            event = self._release_event(self._left, True, menu_open, now_ms)
            if event is not None:
                events.append(event)

        if right_transition is False:
            event = self._release_event(self._right, False, menu_open, now_ms)
            if event is not None:
                events.append(event)

        if not self._left.stable_pressed and not self._right.stable_pressed:
            self._combo_emitted = False

        return tuple(events)

    def _release_event(self, button, is_left, menu_open, now_ms):
        pressed_ms = button.pressed_ms
        button.pressed_ms = None

        if button.suppress_release:
            button.suppress_release = False
            return None
        if pressed_ms is None:
            return None

        is_long = _ticks_diff(now_ms, pressed_ms) >= self.long_press_ms
        if menu_open:
            if is_long:
                return EVENT_AUTO_CANCEL
            return EVENT_AUTO_CONFIRM if is_left else EVENT_AUTO_NEXT

        if is_left:
            return EVENT_TIMER_RESET if is_long else EVENT_TIMER_TOGGLE
        return EVENT_MODE_NEXT if is_long else EVENT_TARE
