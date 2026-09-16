import importlib.util
import os
import sys
import types
import unittest


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LIB_DIR = os.path.join(ROOT_DIR, "firmware", "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

import app_contracts as contracts
from display_renderer import (
    COLOR_ALERT,
    COLOR_GRAPH,
    COLOR_GRID,
    COLOR_PRIMARY,
    DisplayRenderer,
    format_elapsed,
    format_flow,
    format_ratio,
    format_weight,
)


class FakeScreen:
    width = 256
    height = 64
    color_on = 15

    def __init__(self):
        self.pixels = {}
        self.texts = []
        self.show_count = 0

    def _check(self, x, y):
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise AssertionError("out-of-bounds pixel ({}, {})".format(x, y))

    def fill(self, color):
        self.pixels = {}

    def pixel(self, x, y, color):
        self._check(x, y)
        self.pixels[(x, y)] = color

    def fill_rect(self, x, y, width, height, color):
        if width < 0 or height < 0:
            raise AssertionError("negative rectangle")
        for py in range(y, y + height):
            for px in range(x, x + width):
                self.pixel(px, py, color)

    def hline(self, x, y, width, color):
        self.fill_rect(x, y, width, 1, color)

    def vline(self, x, y, height, color):
        self.fill_rect(x, y, 1, height, color)

    def text(self, value, x, y, color):
        width = len(value) * 8
        if x < 0 or y < 0 or x + width > self.width or y + 8 > self.height:
            raise AssertionError("out-of-bounds text {!r}".format(value))
        self.texts.append((value, x, y, color))

    def show(self):
        self.show_count += 1


class DisplayRendererTests(unittest.TestCase):
    def setUp(self):
        self.config = contracts.BrewConfig()
        self.screen = FakeScreen()
        self.renderer = DisplayRenderer(self.screen, self.config)
        self.state = contracts.DisplayState(self.config)
        self.state.elapsed_ms = 75432
        self.state.weight_g = 18.45
        self.state.flow_gps = 2.3
        self.state.ratio = 1.23
        self.state.battery_percent = 84
        self.state.session_state = contracts.SESSION_RUNNING
        self.state.auto_enabled = True

    def labels(self):
        return [entry[0] for entry in self.screen.texts]

    def test_rejects_non_256_by_64_screen(self):
        screen = FakeScreen()
        screen.width = 128
        with self.assertRaises(ValueError):
            DisplayRenderer(screen, self.config)

    def test_timer_weight_uses_two_full_height_cells(self):
        self.state.mode = contracts.MODE_TIMER_WEIGHT
        self.renderer.render(self.state, 1000)

        self.assertEqual(1, self.screen.show_count)
        self.assertIn("TIME", self.labels())
        self.assertIn("WEIGHT", self.labels())
        self.assertIn("min", self.labels())
        self.assertIn("g", self.labels())
        self.assertTrue(any(x == 127 and y >= 8 for x, y in self.screen.pixels))

    def test_three_cell_modes_render_expected_middle_metric(self):
        for mode, middle_label in (
            (contracts.MODE_TIMER_FLOW_WEIGHT, "FLOW g/s"),
            (contracts.MODE_TIMER_RATIO_WEIGHT, "RATIO"),
        ):
            with self.subTest(mode=mode):
                self.screen = FakeScreen()
                self.renderer = DisplayRenderer(self.screen, self.config)
                self.state.mode = mode
                self.renderer.render(self.state, 1000)
                self.assertIn(middle_label, self.labels())
                self.assertTrue(
                    any(x == 84 and y >= 8 for x, y in self.screen.pixels)
                )
                self.assertTrue(
                    any(x == 169 and y >= 8 for x, y in self.screen.pixels)
                )

    def test_all_mode_uses_four_quadrants_and_flow_graph(self):
        self.state.mode = contracts.MODE_ALL
        self.state.graph_values = (0.0, 1.0, 2.0, 1.5)
        self.renderer.render(self.state, 1000)

        self.assertIn("TIME", self.labels())
        self.assertIn("WEIGHT g", self.labels())
        self.assertIn("FLOW g/s", self.labels())
        self.assertIn("RATIO", self.labels())
        self.assertTrue(any(y == 35 for _, y in self.screen.pixels))
        self.assertTrue(
            any(color == COLOR_GRAPH for color in self.screen.pixels.values())
        )

    def test_status_contains_mode_session_auto_profile_ble_and_battery(self):
        self.state.mode = contracts.MODE_TIMER_FLOW_WEIGHT
        self.state.auto_profile = contracts.PROFILE_POUR_OVER
        self.state.ble_connected = True
        self.renderer.render(self.state, 1000)

        self.assertIn("TFW", self.labels())
        self.assertIn("RUN AP B", self.labels())
        self.assertIn("84%", self.labels())

    def test_auto_selection_screen_and_countdown(self):
        self.state.auto_menu_open = True
        self.state.auto_profile = contracts.PROFILE_ESPRESSO
        self.state.auto_menu_deadline_ms = 6000
        self.renderer.render(self.state, 1000)

        self.assertIn("AUTO PROFILE", self.labels())
        self.assertIn("RIGHT:NEXT  LEFT:OK  5s", self.labels())
        self.assertTrue(
            any(color == COLOR_PRIMARY for color in self.screen.pixels.values())
        )

    def test_active_transient_overlays_content_and_expired_one_does_not(self):
        self.state.transient_message = "LOCK"
        self.state.transient_until_ms = 2000
        self.renderer.render(self.state, 1000)
        active_pixels = dict(self.screen.pixels)

        self.screen = FakeScreen()
        self.renderer = DisplayRenderer(self.screen, self.config)
        self.renderer.render(self.state, 2000)

        self.assertTrue(any(color == COLOR_GRID for color in active_pixels.values()))
        self.assertNotEqual(active_pixels, self.screen.pixels)

    def test_extreme_values_and_long_transient_stay_inside_framebuffer(self):
        self.state.mode = contracts.MODE_TIMER_RATIO_WEIGHT
        self.state.weight_g = -99999.9
        self.state.ratio = 123456789.0
        self.state.transient_message = "LOCK" * 40
        self.state.transient_until_ms = 2000

        self.renderer.render(self.state, 1000)

        self.assertEqual(1, self.screen.show_count)


class GraphTests(unittest.TestCase):
    def setUp(self):
        self.config = contracts.BrewConfig()
        self.screen = FakeScreen()
        self.renderer = DisplayRenderer(self.screen, self.config)
        self.state = contracts.DisplayState(self.config)

    def draw(self, values):
        self.state.graph_values = values
        self.renderer._draw_graph(10, 10, 80, 20, self.state)

    def test_empty_graph_draws_grid_only(self):
        self.draw(())
        colors = set(self.screen.pixels.values())
        self.assertEqual({COLOR_GRID}, colors)

    def test_one_sample_is_drawn_at_right_edge(self):
        self.draw((3.0,))
        self.assertEqual(COLOR_GRAPH, self.screen.pixels[(89, 19)])

    def test_full_graph_is_bucketed_to_available_width(self):
        self.draw(tuple((index % 30) / 5.0 for index in range(300)))
        graph_points = [
            point
            for point, color in self.screen.pixels.items()
            if color == COLOR_GRAPH
        ]
        self.assertTrue(graph_points)
        self.assertTrue(all(10 <= x <= 89 and 10 <= y <= 29 for x, y in graph_points))

    def test_negative_is_clamped_and_over_range_gets_top_marker(self):
        self.draw((-2.0, 3.0, 9.0))
        self.assertEqual(COLOR_ALERT, self.screen.pixels[(89, 10)])
        self.assertEqual(COLOR_ALERT, self.screen.pixels[(89, 11)])
        self.assertTrue(
            any(
                color == COLOR_GRAPH and y == 28
                for (x, y), color in self.screen.pixels.items()
            )
        )


class FormattingTests(unittest.TestCase):
    def test_elapsed_boundaries(self):
        self.assertEqual("00:00", format_elapsed(-1))
        self.assertEqual("00:59", format_elapsed(59999))
        self.assertEqual("01:00", format_elapsed(60000))
        self.assertEqual("99:59", format_elapsed(100 * 60 * 1000))

    def test_weight_precision_boundary_and_negative_zero(self):
        self.assertEqual("0.00", format_weight(-0.001))
        self.assertEqual("99.99", format_weight(99.99))
        self.assertEqual("100.0", format_weight(100.0))

    def test_flow_and_ratio_formatting(self):
        self.assertEqual("0.0", format_flow(-0.01))
        self.assertEqual("-1.2", format_flow(-1.24))
        self.assertEqual("1:0.0", format_ratio(-2.0))
        self.assertEqual("1:2.0", format_ratio(2.0))


def load_ssd1322_module():
    micropython = types.ModuleType("micropython")
    micropython.const = lambda value: value

    framebuf = types.ModuleType("framebuf")
    framebuf.GS4_HMSB = 1

    class FrameBuffer:
        def __init__(self, buffer, width, height, pixel_format):
            self.framebuffer_args = (buffer, width, height, pixel_format)

        def fill(self, color):
            self.fill_color = color

    framebuf.FrameBuffer = FrameBuffer

    previous_micropython = sys.modules.get("micropython")
    previous_framebuf = sys.modules.get("framebuf")
    sys.modules["micropython"] = micropython
    sys.modules["framebuf"] = framebuf
    try:
        path = os.path.join(LIB_DIR, "ssd1322.py")
        spec = importlib.util.spec_from_file_location("ssd1322_under_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous_micropython is None:
            del sys.modules["micropython"]
        else:
            sys.modules["micropython"] = previous_micropython
        if previous_framebuf is None:
            del sys.modules["framebuf"]
        else:
            sys.modules["framebuf"] = previous_framebuf


class FakePin:
    OUT = 1

    def __init__(self):
        self.values = []

    def init(self, mode, value=0):
        self.values.append((mode, value))

    def __call__(self, value):
        self.values.append(value)


class FakeSPI:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))


class SSD1322Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.driver = load_ssd1322_module()

    def make_display(self, rotation=0):
        driver = self.driver

        class RecordingSSD(driver.SSD1322_SPI):
            def reset(self):
                pass

            def write_cmd(self, command, *data):
                self.commands.append((command,) + data)

            def write_data(self, data):
                self.data_lengths.append(len(data))

            def __init__(self, *args, **kwargs):
                self.commands = []
                self.data_lengths = []
                super().__init__(*args, **kwargs)

        return RecordingSSD(
            256,
            64,
            FakeSPI(),
            FakePin(),
            FakePin(),
            FakePin(),
            rotation=rotation,
        )

    def test_rotation_zero_uses_corrected_observed_remap(self):
        display = self.make_display(0)
        self.assertIn((0xA0, 0x06, 0x11), display.commands)

    def test_rotation_180_uses_opposite_remap(self):
        display = self.make_display(180)
        self.assertIn((0xA0, 0x14, 0x11), display.commands)

    def test_rotation_can_change_at_runtime_and_rejects_other_values(self):
        display = self.make_display(0)
        display.set_rotation(180)
        self.assertEqual(180, display.rotation)
        self.assertEqual((0xA0, 0x14, 0x11), display.commands[-1])
        with self.assertRaises(ValueError):
            display.set_rotation(90)

    def test_buffer_and_visible_window_geometry_are_unchanged(self):
        display = self.make_display(0)
        self.assertEqual(8192, len(display.buffer))
        display.commands = []
        display.data_lengths = []
        display.show()

        self.assertEqual(
            [(0x15, 0x1C, 0x5B), (0x75, 0, 63), (0x5C,)],
            display.commands,
        )
        self.assertEqual([8192], display.data_lengths)


if __name__ == "__main__":
    unittest.main()
