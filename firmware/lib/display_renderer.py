"""Full-width presentation renderer for the 256x64 scale display."""

try:
    import time
except ImportError:  # pragma: no cover - MicroPython always provides time.
    time = None

from app_contracts import (
    MODE_ALL,
    MODE_TIMER_FLOW_WEIGHT,
    MODE_TIMER_RATIO_WEIGHT,
    MODE_TIMER_WEIGHT,
    PROFILE_ESPRESSO,
    PROFILE_POUR_OVER,
    SESSION_ARMED,
    SESSION_IDLE,
    SESSION_RUNNING,
    SESSION_STOPPED,
)


DISPLAY_WIDTH = 256
DISPLAY_HEIGHT = 64
STATUS_HEIGHT = 8

COLOR_BACKGROUND = 0
COLOR_GRID = 2
COLOR_DIVIDER = 4
COLOR_SECONDARY = 7
COLOR_GRAPH = 11
COLOR_ALERT = 13
COLOR_PRIMARY = 15

# A compact Bluetooth rune that fits inside the 8-pixel status row. Framebuf
# text fonts do not contain the Unicode Bluetooth symbol, so draw it directly.
_BLUETOOTH_GLYPH = (
    "00100",
    "10110",
    "01101",
    "00110",
    "01101",
    "10110",
    "00100",
)


# Compact 3x5 glyphs. They keep the main values readable even in an 85 px cell.
_BIG_GLYPHS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
    ".": ("000", "000", "000", "000", "010"),
    ":": ("000", "010", "000", "010", "000"),
    "C": ("111", "100", "100", "100", "111"),
    "E": ("111", "100", "110", "100", "111"),
    "K": ("101", "101", "110", "101", "101"),
    "L": ("100", "100", "100", "100", "111"),
    "O": ("111", "101", "101", "101", "111"),
    "P": ("111", "101", "111", "100", "100"),
    "R": ("110", "101", "110", "101", "101"),
    "S": ("111", "100", "111", "001", "111"),
    "U": ("101", "101", "101", "101", "111"),
    " ": ("000", "000", "000", "000", "000"),
}


def _ticks_diff(newer, older):
    ticks_diff = getattr(time, "ticks_diff", None) if time is not None else None
    if ticks_diff is not None:
        return ticks_diff(newer, older)
    return newer - older


def format_elapsed(elapsed_ms):
    """Return a bounded MM:SS timer string."""
    elapsed_ms = max(0, int(elapsed_ms))
    total_seconds = elapsed_ms // 1000
    total_seconds = min(total_seconds, 99 * 60 + 59)
    return "{:02d}:{:02d}".format(total_seconds // 60, total_seconds % 60)


def format_weight(weight_g):
    """Show the realistic 0.1 g resolution of the 1 kg load cell."""
    value = float(weight_g)
    if abs(value) < 0.05:
        value = 0.0
    return "{:.1f}".format(value)


def format_flow(flow_gps):
    value = float(flow_gps)
    if abs(value) < 0.05:
        value = 0.0
    return "{:.1f}".format(value)


def format_ratio(ratio):
    return "1:{:.1f}".format(max(0.0, float(ratio)))


class DisplayRenderer:
    """Render ``DisplayState`` snapshots to a 256x64 framebuffer."""

    def __init__(self, screen, config):
        if screen.width != DISPLAY_WIDTH or screen.height != DISPLAY_HEIGHT:
            raise ValueError("DisplayRenderer requires a 256x64 framebuffer")
        self.screen = screen
        self.config = config

        # Reused work buffers avoid allocating graph buckets every frame.
        self._graph_values = [0.0] * DISPLAY_WIDTH
        self._graph_seen = bytearray(DISPLAY_WIDTH)
        self._graph_overflow = bytearray(DISPLAY_WIDTH)

    def render(self, state, now_ms):
        self.screen.fill(COLOR_BACKGROUND)
        self._draw_status(state)

        if state.auto_menu_open:
            self._draw_auto_menu(state, now_ms)
        elif state.mode == MODE_TIMER_FLOW_WEIGHT:
            self._draw_three_metrics(state, "flow")
        elif state.mode == MODE_TIMER_RATIO_WEIGHT:
            self._draw_three_metrics(state, "ratio")
        elif state.mode == MODE_ALL:
            self._draw_all(state)
        else:
            self._draw_timer_weight(state)

        if self._transient_is_active(state, now_ms):
            self._draw_transient(state.transient_message)

        self.screen.show()

    def _draw_status(self, state):
        mode_names = ("TW", "TFW", "TRW", "ALL")
        session_names = ("IDLE", "ARM", "RUN", "STOP")

        mode = mode_names[state.mode] if 0 <= state.mode < len(mode_names) else "TW"
        session = (
            session_names[state.session_state]
            if 0 <= state.session_state < len(session_names)
            else "IDLE"
        )
        if state.auto_enabled:
            profile = "AE" if state.auto_profile == PROFILE_ESPRESSO else "AP"
            session = session + " " + profile

        battery = max(0, min(100, int(state.battery_percent)))
        battery_text = "{}%".format(battery)
        battery_x = DISPLAY_WIDTH - len(battery_text) * 8

        self.screen.text(mode, 0, 0, COLOR_SECONDARY)
        self.screen.text(session[:12], 80, 0, COLOR_PRIMARY)
        self.screen.text(
            battery_text,
            battery_x,
            0,
            COLOR_SECONDARY,
        )
        if state.ble_connected:
            self._draw_bluetooth(battery_x - 8, 0)

    def _draw_bluetooth(self, x, y):
        for row, pixels in enumerate(_BLUETOOTH_GLYPH):
            for column, pixel in enumerate(pixels):
                if pixel == "1":
                    self.screen.pixel(x + column, y + row, COLOR_PRIMARY)

    def _draw_timer_weight(self, state):
        self._metric_cell(
            0,
            STATUS_HEIGHT,
            128,
            56,
            "TIME",
            format_elapsed(state.elapsed_ms),
            "min",
        )
        self._vline(127, STATUS_HEIGHT, 56, COLOR_DIVIDER)
        self._metric_cell(
            128,
            STATUS_HEIGHT,
            128,
            56,
            "WEIGHT",
            format_weight(state.weight_g),
            "g",
        )

    def _draw_three_metrics(self, state, middle):
        cells = ((0, 85), (85, 85), (170, 86))
        self._metric_cell(
            cells[0][0],
            STATUS_HEIGHT,
            cells[0][1],
            56,
            "TIME",
            format_elapsed(state.elapsed_ms),
            "min",
        )

        if middle == "flow":
            self._flow_cell(
                cells[1][0],
                STATUS_HEIGHT,
                cells[1][1],
                56,
                state,
            )
        else:
            self._metric_cell(
                cells[1][0],
                STATUS_HEIGHT,
                cells[1][1],
                56,
                "RATIO",
                format_ratio(state.ratio),
                "",
            )

        self._metric_cell(
            cells[2][0],
            STATUS_HEIGHT,
            cells[2][1],
            56,
            "WEIGHT",
            format_weight(state.weight_g),
            "g",
        )
        self._vline(84, STATUS_HEIGHT, 56, COLOR_DIVIDER)
        self._vline(169, STATUS_HEIGHT, 56, COLOR_DIVIDER)

    def _draw_all(self, state):
        self._metric_cell(
            0, 8, 128, 28, "TIME", format_elapsed(state.elapsed_ms), ""
        )
        self._metric_cell(
            128, 8, 128, 28, "WEIGHT g", format_weight(state.weight_g), ""
        )
        self._flow_quadrant(0, 36, 128, 28, state)
        self._metric_cell(
            128, 36, 128, 28, "RATIO", format_ratio(state.ratio), ""
        )
        self._vline(127, STATUS_HEIGHT, 56, COLOR_DIVIDER)
        self._hline(0, 35, DISPLAY_WIDTH, COLOR_DIVIDER)

    def _metric_cell(self, x, y, width, height, label, value, unit):
        label_x = x + 2
        self.screen.text(label, label_x, y + 1, COLOR_SECONDARY)

        compact = height < 40
        if compact and unit:
            unit = ""

        top = y + 10
        bottom = y + height - (9 if unit else 2)
        scale = self._best_scale(value, width - 4, bottom - top)
        value_width = self._big_text_width(value, scale)
        value_height = 5 * scale
        value_x = x + max(2, (width - value_width) // 2)
        value_y = top + max(0, (bottom - top - value_height) // 2)
        self._draw_big_text(value, value_x, value_y, scale, COLOR_PRIMARY)

        if unit:
            unit_x = x + width - len(unit) * 8 - 2
            self.screen.text(unit, unit_x, y + height - 8, COLOR_SECONDARY)

    def _flow_cell(self, x, y, width, height, state):
        self.screen.text("FLOW g/s", x + 2, y + 1, COLOR_SECONDARY)
        value = format_flow(state.flow_gps)
        scale = self._best_scale(value, width - 4, 20)
        value_width = self._big_text_width(value, scale)
        self._draw_big_text(
            value,
            x + max(2, (width - value_width) // 2),
            y + 10,
            scale,
            COLOR_PRIMARY,
        )
        self._draw_graph(x + 2, y + 34, width - 4, height - 36, state)

    def _flow_quadrant(self, x, y, width, height, state):
        self.screen.text("FLOW g/s", x + 2, y + 1, COLOR_SECONDARY)
        value = format_flow(state.flow_gps)
        scale = self._best_scale(value, 57, 16)
        self._draw_big_text(value, x + 2, y + 10, scale, COLOR_PRIMARY)
        self._draw_graph(x + 62, y + 9, width - 64, height - 11, state)

    def _draw_auto_menu(self, state, now_ms):
        self.screen.text("AUTO PROFILE", 80, 11, COLOR_SECONDARY)
        profile = "ESPRESSO" if state.auto_profile == PROFILE_ESPRESSO else "POUR"
        scale = self._best_scale(profile, 244, 25)
        text_width = self._big_text_width(profile, scale)
        self._draw_big_text(
            profile,
            (DISPLAY_WIDTH - text_width) // 2,
            24,
            scale,
            COLOR_PRIMARY,
        )

        remaining_ms = _ticks_diff(state.auto_menu_deadline_ms, now_ms)
        remaining_s = max(0, (remaining_ms + 999) // 1000)
        footer = "RIGHT:NEXT  LEFT:OK  {}s".format(remaining_s)
        footer = footer[: DISPLAY_WIDTH // 8]
        self.screen.text(
            footer,
            (DISPLAY_WIDTH - len(footer) * 8) // 2,
            56,
            COLOR_SECONDARY,
        )

    def _transient_is_active(self, state, now_ms):
        return bool(state.transient_message) and _ticks_diff(
            state.transient_until_ms, now_ms
        ) > 0

    def _draw_transient(self, message):
        message = str(message).upper()
        self._fill_rect(0, 20, DISPLAY_WIDTH, 28, COLOR_GRID)

        if all(char in _BIG_GLYPHS for char in message):
            # At scale 1, 61 compact glyphs are the widest possible overlay.
            message = message[:61]
            scale = self._best_scale(message, 244, 20)
            text_width = self._big_text_width(message, scale)
            self._draw_big_text(
                message,
                (DISPLAY_WIDTH - text_width) // 2,
                22,
                scale,
                COLOR_PRIMARY,
            )
        else:
            message = message[: DISPLAY_WIDTH // 8]
            self.screen.text(
                message,
                (DISPLAY_WIDTH - len(message) * 8) // 2,
                30,
                COLOR_PRIMARY,
            )

    def _draw_graph(self, x, y, width, height, state):
        if width <= 1 or height <= 2:
            return

        self._hline(x, y + height - 1, width, COLOR_GRID)
        self._hline(x, y + (height - 1) // 2, width, COLOR_GRID)
        self._vline(x + width // 2, y, height, COLOR_GRID)

        values = state.graph_values
        try:
            value_count = len(values)
        except TypeError:
            value_count = 0
        if value_count == 0:
            return

        sample_ms = max(1, int(state.graph_sample_ms))
        span_ms = max(sample_ms, int(state.graph_span_ms))
        visible_limit = span_ms // sample_ms + 1
        start = max(0, value_count - visible_limit)
        visible_count = value_count - start

        for index in range(width):
            self._graph_seen[index] = 0
            self._graph_overflow[index] = 0
            self._graph_values[index] = 0.0

        graph_max = float(state.graph_max_gps)
        if graph_max <= 0.0:
            graph_max = self.config.graph_max_for(state.auto_profile)
        graph_max = max(0.01, float(graph_max))

        for index in range(visible_count):
            sample = values[start + index]
            if isinstance(sample, (tuple, list)):
                sample = sample[-1] if sample else 0.0
            sample = max(0.0, float(sample))
            column = (
                width - 1
                if visible_count == 1
                else (index * (width - 1)) // (visible_count - 1)
            )
            if not self._graph_seen[column] or sample > self._graph_values[column]:
                self._graph_values[column] = sample
            self._graph_seen[column] = 1
            if sample > graph_max:
                self._graph_overflow[column] = 1

        plot_bottom = y + height - 2
        plot_height = max(1, plot_bottom - y)
        previous_x = -1
        previous_y = plot_bottom
        for column in range(width):
            if not self._graph_seen[column]:
                continue
            sample = min(self._graph_values[column], graph_max)
            point_y = plot_bottom - int((sample / graph_max) * plot_height)
            point_x = x + column
            if previous_x >= 0:
                self._draw_line(
                    previous_x,
                    previous_y,
                    point_x,
                    point_y,
                    COLOR_GRAPH,
                )
            else:
                self.screen.pixel(point_x, point_y, COLOR_GRAPH)
            if self._graph_overflow[column]:
                self.screen.pixel(point_x, y, COLOR_ALERT)
                if y + 1 < y + height:
                    self.screen.pixel(point_x, y + 1, COLOR_ALERT)
            previous_x = point_x
            previous_y = point_y

    def _best_scale(self, value, max_width, max_height):
        if not value:
            return 1
        width_units = len(value) * 4 - 1
        return max(1, min(5, max_width // width_units, max_height // 5))

    def _big_text_width(self, value, scale):
        if not value:
            return 0
        return (len(value) * 4 - 1) * scale

    def _draw_big_text(self, value, x, y, scale, color):
        cursor = x
        for char in value.upper():
            glyph = _BIG_GLYPHS.get(char, _BIG_GLYPHS[" "])
            for row, row_data in enumerate(glyph):
                for column, enabled in enumerate(row_data):
                    if enabled == "1":
                        self._fill_rect(
                            cursor + column * scale,
                            y + row * scale,
                            scale,
                            scale,
                            color,
                        )
            cursor += 4 * scale

    def _draw_line(self, x0, y0, x1, y1, color):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            self.screen.pixel(x0, y0, color)
            if x0 == x1 and y0 == y1:
                return
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x0 += sx
            if doubled <= dx:
                error += dx
                y0 += sy

    def _fill_rect(self, x, y, width, height, color):
        self.screen.fill_rect(x, y, width, height, color)

    def _hline(self, x, y, width, color):
        self.screen.hline(x, y, width, color)

    def _vline(self, x, y, height, color):
        self.screen.vline(x, y, height, color)
