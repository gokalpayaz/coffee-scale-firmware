"""Deferred tare detection for a button mounted on the weighing surface."""

try:
    import time
except ImportError:  # pragma: no cover - MicroPython always provides time.
    time = None


def _ticks_diff(newer, older):
    ticks_diff = getattr(time, "ticks_diff", None) if time is not None else None
    if ticks_diff is not None:
        return ticks_diff(newer, older)
    return newer - older


class TareSettler:
    """Wait for release rebound to settle before allowing an HX711 tare."""

    __slots__ = (
        "min_wait_ms",
        "stable_window_ms",
        "stable_max_span_g",
        "timeout_ms",
        "pending",
        "last_reason",
        "_requested_ms",
        "_stable_started_ms",
        "_stable_min_g",
        "_stable_max_g",
    )

    def __init__(self, config):
        self.min_wait_ms = int(config.tare_min_wait_ms)
        self.stable_window_ms = int(config.tare_stable_window_ms)
        self.stable_max_span_g = float(config.tare_stable_max_span_g)
        self.timeout_ms = int(config.tare_timeout_ms)

        if self.min_wait_ms < 0 or self.stable_window_ms <= 0:
            raise ValueError("tare wait durations are invalid")
        if self.stable_max_span_g < 0.0 or self.timeout_ms <= self.min_wait_ms:
            raise ValueError("tare stability limits are invalid")

        self.pending = False
        self.last_reason = ""
        self._requested_ms = 0
        self._reset_stable_window()

    def request(self, now_ms):
        """Start or restart a deferred tare request."""

        self.pending = True
        self.last_reason = ""
        self._requested_ms = now_ms
        self._reset_stable_window()

    def update(self, weight_g, now_ms):
        """Return True once stable or timed out; return True only once."""

        if not self.pending:
            return False

        elapsed_ms = _ticks_diff(now_ms, self._requested_ms)
        if elapsed_ms >= self.timeout_ms:
            return self._finish("timeout")
        if elapsed_ms < self.min_wait_ms:
            return False

        weight_g = float(weight_g)
        if self._stable_started_ms is None:
            self._start_stable_window(weight_g, now_ms)
            return False

        if weight_g < self._stable_min_g:
            self._stable_min_g = weight_g
        if weight_g > self._stable_max_g:
            self._stable_max_g = weight_g

        if self._stable_max_g - self._stable_min_g > self.stable_max_span_g:
            self._start_stable_window(weight_g, now_ms)
            return False

        if _ticks_diff(now_ms, self._stable_started_ms) >= self.stable_window_ms:
            return self._finish("stable")
        return False

    def _start_stable_window(self, weight_g, now_ms):
        self._stable_started_ms = now_ms
        self._stable_min_g = weight_g
        self._stable_max_g = weight_g

    def _reset_stable_window(self):
        self._stable_started_ms = None
        self._stable_min_g = 0.0
        self._stable_max_g = 0.0

    def _finish(self, reason):
        self.pending = False
        self.last_reason = reason
        self._reset_stable_window()
        return True
