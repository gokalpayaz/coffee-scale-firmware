"""On-device load-cell calibration started with a boot button chord."""

import time

from calibration import REFERENCE_WEIGHT_G, calculate_scale_factor


START_HOLD_MS = 3000

_POLL_MS = 10
_DEBOUNCE_MS = 40
_SETTLE_MS = 2000
_READINGS = 25

_BACKGROUND = 0
_PRIMARY = 15
_SECONDARY = 7


def _ticks_ms(clock):
    ticks_ms = getattr(clock, "ticks_ms", None)
    if ticks_ms is not None:
        return ticks_ms()
    return int(clock.monotonic() * 1000)


def _ticks_diff(clock, newer, older):
    ticks_diff = getattr(clock, "ticks_diff", None)
    if ticks_diff is not None:
        return ticks_diff(newer, older)
    return newer - older


def _sleep_ms(clock, duration_ms):
    sleep_ms = getattr(clock, "sleep_ms", None)
    if sleep_ms is not None:
        sleep_ms(duration_ms)
    else:
        clock.sleep(duration_ms / 1000.0)


def _centered_x(text):
    return max(0, (256 - len(text) * 8) // 2)


def _show(screen, title, line_1="", line_2="", line_3=""):
    """Draw a simple four-line calibration page on the SSD1322."""

    screen.fill(_BACKGROUND)
    for text, y, color in (
        (title, 0, _PRIMARY),
        (line_1, 18, _PRIMARY),
        (line_2, 36, _SECONDARY),
        (line_3, 54, _SECONDARY),
    ):
        if text:
            screen.text(text, _centered_x(text), y, color)
    screen.show()


def _wait_until_released(read_controls, clock):
    while True:
        left_pressed, right_pressed = read_controls()
        if not left_pressed and not right_pressed:
            return
        _sleep_ms(clock, _POLL_MS)


def boot_chord_requested(screen, read_controls, clock=time, hold_ms=START_HOLD_MS):
    """Return true when both controls stay held for ``hold_ms`` at boot.

    The check has no boot delay when the scale starts with either control
    released. Normal runtime uses a shorter two-button chord for the automatic
    profile menu, so calibration is deliberately restricted to boot.
    """

    left_pressed, right_pressed = read_controls()
    if not left_pressed or not right_pressed:
        return False

    started_ms = _ticks_ms(clock)
    shown_seconds = None
    while True:
        left_pressed, right_pressed = read_controls()
        if not left_pressed or not right_pressed:
            return False

        elapsed_ms = _ticks_diff(clock, _ticks_ms(clock), started_ms)
        remaining_ms = max(0, hold_ms - elapsed_ms)
        remaining_seconds = (remaining_ms + 999) // 1000
        if remaining_seconds != shown_seconds:
            _show(
                screen,
                "CALIBRATION",
                "KEEP HOLDING BOTH",
                "{}".format(remaining_seconds),
            )
            shown_seconds = remaining_seconds

        if elapsed_ms >= hold_ms:
            _show(screen, "CALIBRATION", "RELEASE BUTTONS")
            _wait_until_released(read_controls, clock)
            return True

        _sleep_ms(clock, _POLL_MS)


def _wait_for_choice(read_controls, clock, allow_cancel=True):
    """Wait for a debounced press/release; return true for left, false for right."""

    _wait_until_released(read_controls, clock)
    while True:
        left_pressed, right_pressed = read_controls()
        choice = None
        if left_pressed and not right_pressed:
            choice = True
        elif allow_cancel and right_pressed and not left_pressed:
            choice = False

        if choice is not None:
            _sleep_ms(clock, _DEBOUNCE_MS)
            stable_left, stable_right = read_controls()
            if choice and stable_left and not stable_right:
                _wait_until_released(read_controls, clock)
                return True
            if not choice and stable_right and not stable_left:
                _wait_until_released(read_controls, clock)
                return False

        _sleep_ms(clock, _POLL_MS)


def _settle(screen, label, clock, duration_ms):
    remaining_ms = duration_ms
    shown_seconds = None
    while remaining_ms > 0:
        remaining_seconds = (remaining_ms + 999) // 1000
        if remaining_seconds != shown_seconds:
            _show(
                screen,
                "CALIBRATION",
                label,
                "HANDS OFF",
                "{}".format(remaining_seconds),
            )
            shown_seconds = remaining_seconds
        step_ms = min(_POLL_MS, remaining_ms)
        _sleep_ms(clock, step_ms)
        remaining_ms -= step_ms


def _prompt_remove_weight(
    screen,
    read_controls,
    clock,
    title,
    settle_ms=_SETTLE_MS,
):
    _show(screen, title, "REMOVE ALL WEIGHT", "LEFT:CONTINUE")
    _wait_for_choice(read_controls, clock, allow_cancel=False)
    _settle(screen, "ZEROING", clock, settle_ms)


def run_device_calibration(
    hx,
    calibration_store,
    screen,
    read_controls,
    clock=time,
    reference_weight_g=REFERENCE_WEIGHT_G,
    readings=_READINGS,
    settle_ms=_SETTLE_MS,
):
    """Run the display/button calibration wizard and return the saved factor.

    Left confirms each step. Right exits before a reading and leaves the stored
    factor unchanged. The final empty-scale confirmation ensures normal boot
    cannot tare with the reference weight still on the platform.
    """

    _show(
        screen,
        "CALIBRATION",
        "EMPTY THE SCALE",
        "LEFT:OK RIGHT:EXIT",
    )
    if not _wait_for_choice(read_controls, clock):
        _show(screen, "CALIBRATION", "CANCELLED", "KEEPING OLD VALUE")
        _sleep_ms(clock, 1000)
        return None

    _settle(screen, "MEASURING EMPTY", clock, settle_ms)
    try:
        hx.set_scale(1.0)
        empty_average = hx.read_average(times=readings)
    except Exception as error:
        print("calibration failed:", error)
        _prompt_remove_weight(
            screen,
            read_controls,
            clock,
            "CALIBRATION FAILED",
            settle_ms,
        )
        return None

    _show(
        screen,
        "CALIBRATION",
        "PLACE {:.0f} g WEIGHT".format(reference_weight_g),
        "LEFT:OK RIGHT:EXIT",
    )
    if not _wait_for_choice(read_controls, clock):
        _prompt_remove_weight(
            screen,
            read_controls,
            clock,
            "CALIBRATION CANCELLED",
            settle_ms,
        )
        return None

    _settle(screen, "MEASURING {:.0f} g".format(reference_weight_g), clock, settle_ms)

    try:
        loaded_average = hx.read_average(times=readings)
        factor = calculate_scale_factor(
            empty_average,
            loaded_average,
            reference_weight_g,
        )
        calibration_store.save(factor)
    except Exception as error:
        print("calibration failed:", error)
        _prompt_remove_weight(
            screen,
            read_controls,
            clock,
            "CALIBRATION FAILED",
            settle_ms,
        )
        return None

    hx.set_scale(factor)
    _show(
        screen,
        "CALIBRATION SAVED",
        "FACTOR {:.2f}".format(factor),
        "REMOVE {:.0f} g WEIGHT".format(reference_weight_g),
        "LEFT:CONTINUE",
    )
    _wait_for_choice(read_controls, clock, allow_cancel=False)
    _settle(screen, "STARTING SCALE", clock, settle_ms)
    return factor
