"""Interactive HX711 calibration for the current coffee-scale hardware."""

import time

from debounce import DebouncedSwitch
from hx711 import HX711
from machine import Pin
from micropython import const

from calibration import (
    REFERENCE_WEIGHT_G,
    CalibrationStore,
    calculate_scale_factor,
)


# Keep these pins in sync with main.py. DOUT is data from the HX711 and SCK is
# the clock driven by the ESP32; swapping them prevents the ADC from reading.
_HX711_DOUT = const(13)
_HX711_SCK = const(14)
_HX711_GAIN = const(64)

_RIGHT_BUTTON_PIN = const(25)
_LEFT_BUTTON_PIN = const(26)

_SAMPLES = const(10)
_READINGS_PER_SAMPLE = const(100)
_TEST_WEIGHT_G = REFERENCE_WEIGHT_G
_ADJUSTMENT_STEP = 1.0


# The callbacks are registered only after these values are initialized. The
# globals also keep the debounce objects alive for the final adjustment loop.
hx = None
scale_factor = None
left_switch = None
right_switch = None
calibration_store = None


def _wait_countdown(seconds=5):
    for remaining in range(seconds, 0, -1):
        print("{} ".format(remaining), end="")
        time.sleep(1)
    print()


def _average_raw(label):
    total = 0.0
    for index in range(_SAMPLES):
        reading = hx.read_average(times=_READINGS_PER_SAMPLE)
        print(">>> {} [{}]: {}".format(label, index + 1, reading))
        total += reading
    average = total / _SAMPLES
    print("*** {} average: {}".format(label, average))
    return average


def _set_scale_factor(value):
    global scale_factor
    scale_factor = value
    hx.set_scale(scale_factor)
    try:
        calibration_store.save(scale_factor)
        print("Scale factor saved: {}".format(scale_factor))
    except (OSError, ValueError, TypeError) as error:
        print("WARNING: scale factor was not saved: {}".format(error))


def _decrease_scale_factor(_):
    _set_scale_factor(scale_factor - _ADJUSTMENT_STEP)


def _increase_scale_factor(_):
    _set_scale_factor(scale_factor + _ADJUSTMENT_STEP)


def main():
    """Calculate a scale factor, then allow fine adjustment with the buttons."""

    global hx, left_switch, right_switch, calibration_store

    right_button = Pin(_RIGHT_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    left_button = Pin(_LEFT_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

    hx = HX711(dout=_HX711_DOUT, pd_sck=_HX711_SCK, gain=_HX711_GAIN)
    calibration_store = CalibrationStore()
    # Raw calibration sampling does not use get_units(), but a non-zero scale
    # keeps the object in a valid state if it is inspected from the REPL.
    hx.set_scale(1.0)
    hx.tare()

    print("***** Remove all weight from the scale")
    _wait_countdown()
    empty_average = _average_raw("Empty reading")

    print("\n***** Place an accurate {} g weight".format(_TEST_WEIGHT_G))
    _wait_countdown()
    loaded_average = _average_raw("{} g reading".format(_TEST_WEIGHT_G))

    calculated = calculate_scale_factor(
        empty_average,
        loaded_average,
        _TEST_WEIGHT_G,
    )
    _set_scale_factor(calculated)

    print("\n***** Remove the calibration weight")
    _wait_countdown()
    hx.tare()

    # Register adjustment callbacks only now: both hx and scale_factor are
    # valid, so an early button press cannot trigger a NameError.
    left_switch = DebouncedSwitch(sw=left_button, cb=_decrease_scale_factor)
    right_switch = DebouncedSwitch(sw=right_button, cb=_increase_scale_factor)

    print("\n***** Replace the {} g weight".format(_TEST_WEIGHT_G))
    print(
        "Left decreases and right increases the scale factor by {}.".format(
            _ADJUSTMENT_STEP
        )
    )
    print("The final scale factor is saved automatically in ESP32 NVS.")
    _wait_countdown()

    while True:
        print(">>> Measured weight: {} g".format(hx.get_units(5)))
        time.sleep(3)


if __name__ == "__main__":
    main()
