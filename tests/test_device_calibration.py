import os
import sys
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from device_calibration import boot_chord_requested, run_device_calibration


class FakeClock:
    def __init__(self):
        self.now_ms = 0

    def ticks_ms(self):
        return self.now_ms

    @staticmethod
    def ticks_diff(newer, older):
        return newer - older

    def sleep_ms(self, duration_ms):
        self.now_ms += duration_ms


class FakeScreen:
    def __init__(self):
        self.pages = []
        self.current_text = []

    def fill(self, _color):
        self.current_text = []

    def text(self, value, x, y, color):
        self.current_text.append((value, x, y, color))

    def show(self):
        self.pages.append(tuple(entry[0] for entry in self.current_text))


class TimedControls:
    def __init__(self, clock, intervals):
        self.clock = clock
        self.intervals = intervals

    def __call__(self):
        now_ms = self.clock.now_ms
        for start_ms, end_ms, value in self.intervals:
            if start_ms <= now_ms < end_ms:
                return value
        return False, False


class FakeHX711:
    def __init__(self, readings):
        self.readings = list(readings)
        self.scale_calls = []
        self.sample_calls = []

    def set_scale(self, factor):
        self.scale_calls.append(factor)

    def read_average(self, times):
        self.sample_calls.append(times)
        return self.readings.pop(0)


class FakeStore:
    def __init__(self, error=None):
        self.error = error
        self.saved = []

    def save(self, factor):
        if self.error is not None:
            raise self.error
        self.saved.append(factor)
        return True


class BootChordTests(unittest.TestCase):
    def test_requires_both_controls_continuously_for_full_hold(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            ((0, 3010, (True, True)),),
        )

        self.assertTrue(
            boot_chord_requested(
                screen,
                controls,
                clock=clock,
                hold_ms=3000,
            )
        )
        self.assertGreaterEqual(clock.now_ms, 3010)
        self.assertIn("KEEP HOLDING BOTH", screen.pages[0])
        self.assertIn("RELEASE BUTTONS", screen.pages[-1])

    def test_releasing_early_keeps_normal_boot(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            ((0, 1200, (True, True)),),
        )

        self.assertFalse(
            boot_chord_requested(
                screen,
                controls,
                clock=clock,
                hold_ms=3000,
            )
        )
        self.assertLess(clock.now_ms, 3000)

    def test_unpressed_boot_returns_immediately_without_drawing(self):
        clock = FakeClock()
        screen = FakeScreen()

        self.assertFalse(
            boot_chord_requested(
                screen,
                lambda: (False, False),
                clock=clock,
            )
        )
        self.assertEqual(0, clock.now_ms)
        self.assertEqual([], screen.pages)


class DeviceCalibrationTests(unittest.TestCase):
    def test_empty_and_reference_captures_are_saved(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            (
                (100, 200, (True, False)),
                (300, 400, (True, False)),
                (500, 600, (True, False)),
            ),
        )
        hx = FakeHX711((1000, 218500))
        store = FakeStore()

        factor = run_device_calibration(
            hx,
            store,
            screen,
            controls,
            clock=clock,
            readings=7,
            settle_ms=0,
        )

        self.assertEqual(2175.0, factor)
        self.assertEqual([2175.0], store.saved)
        self.assertEqual([1.0, 2175.0], hx.scale_calls)
        self.assertEqual([7, 7], hx.sample_calls)
        self.assertTrue(
            any("CALIBRATION SAVED" in page for page in screen.pages)
        )

    def test_right_button_cancels_without_sampling_or_saving(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            ((100, 200, (False, True)),),
        )
        hx = FakeHX711(())
        store = FakeStore()

        result = run_device_calibration(
            hx,
            store,
            screen,
            controls,
            clock=clock,
            settle_ms=0,
        )

        self.assertIsNone(result)
        self.assertEqual([], store.saved)
        self.assertEqual([], hx.sample_calls)
        self.assertTrue(any("CANCELLED" in page for page in screen.pages))

    def test_cancel_after_empty_capture_requires_reference_removal(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            (
                (100, 200, (True, False)),
                (300, 400, (False, True)),
                (500, 600, (True, False)),
            ),
        )
        hx = FakeHX711((1000,))
        store = FakeStore()

        result = run_device_calibration(
            hx,
            store,
            screen,
            controls,
            clock=clock,
            readings=7,
            settle_ms=0,
        )

        self.assertIsNone(result)
        self.assertEqual([7], hx.sample_calls)
        self.assertEqual([], store.saved)
        self.assertTrue(
            any("REMOVE ALL WEIGHT" in page for page in screen.pages)
        )
        self.assertGreaterEqual(clock.now_ms, 600)

    def test_save_failure_requires_empty_scale_before_returning(self):
        clock = FakeClock()
        screen = FakeScreen()
        controls = TimedControls(
            clock,
            (
                (100, 200, (True, False)),
                (300, 400, (True, False)),
                (500, 600, (True, False)),
            ),
        )
        hx = FakeHX711((1000, 218500))
        store = FakeStore(OSError("NVS unavailable"))

        result = run_device_calibration(
            hx,
            store,
            screen,
            controls,
            clock=clock,
            settle_ms=0,
        )

        self.assertIsNone(result)
        self.assertTrue(
            any("CALIBRATION FAILED" in page for page in screen.pages)
        )
        self.assertGreaterEqual(clock.now_ms, 600)


if __name__ == "__main__":
    unittest.main()
