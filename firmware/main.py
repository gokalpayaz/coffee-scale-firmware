"""Main application for the ESP32 coffee scale."""

import gc
import time

import micropython
from bluetooth import BLE
from machine import ADC, Pin, SPI
from micropython import const

from app_contracts import (
    ACTION_SAVE_MODE,
    ACTION_SAVE_PROFILE,
    ACTION_SESSION_STARTED,
    ACTION_SESSION_STOPPED,
    ACTION_TARE,
    BrewConfig,
    DisplayState,
    EventQueue,
)
from ble_scales import BLEScales
from calibration import CalibrationStore, DEFAULT_SCALE_FACTOR
from display_renderer import DisplayRenderer
from filtering import RobustWeightFilter
from hx711 import HX711
from input_events import InputInterpreter
from measurement import MeasurementController
from preferences import Preferences
from ssd1322 import ROTATION_0, SSD1322_SPI
from tare import TareSettler


# Hardware pins.
_BAT_VOLTAGE_PIN = const(33)
_RESET_BUTTON_PIN = const(25)  # Logical right button, active low.
_TIMER_BUTTON_PIN = const(26)  # Logical left button, active low.
_MODE_TARE_TOUCH_PIN = const(4)  # Logical right touch input.
_TIMER_TOUCH_PIN = const(32)  # Logical left touch input.
_TOUCH_ACTIVE_LEVEL = const(1)

_SSD1322_SCK = const(18)  # J1 pin 4: D0/CLK
_SSD1322_MOSI = const(23)  # J1 pin 5: D1/DIN
_SSD1322_CS = const(19)  # J1 pin 16: CS#
_SSD1322_DC = const(21)  # J1 pin 14: D/C#
_SSD1322_RES = const(27)  # J1 pin 15: RES#
_SSD1322_WIDTH = const(256)
_SSD1322_HEIGHT = const(64)

_HX711_DOUT = const(13)
_HX711_SCK = const(14)

_INPUT_POLL_MS = const(10)
_TELEMETRY_PERIOD_MS = const(100)
_TELEMETRY_ENABLED = False
_DEBUG = True


def adc_to_percent(v_adc):
    """Convert the existing voltage-divider ADC scale to a percentage."""

    adc_full = 1200  # 4.2 V - Arbitrary number for now
    adc_empty = 800  # Approximately 3.5 V. - Arbitrary number for now
    if v_adc >= adc_full:
        return 100
    if v_adc <= adc_empty:
        return 0
    return int((v_adc - adc_empty) * 100 // (adc_full - adc_empty))


def read_battery_percent(vsense):
    battery_sum = 0
    time.sleep_ms(10)
    for _ in range(10):
        battery_sum += vsense.read()
    return adc_to_percent(int(battery_sum * 0.10375685943436))


def load_preferences(state):
    """Load persistent UI choices without making boot depend on NVS."""

    try:
        preferences = Preferences()
        state.mode, state.auto_profile = preferences.load()
        return preferences
    except Exception as error:
        if _DEBUG:
            print("preferences unavailable:", error)
        return None


def load_calibration_factor():
    """Load the saved HX711 factor without making boot depend on NVS."""

    try:
        return CalibrationStore().load()
    except Exception as error:
        if _DEBUG:
            print("calibration unavailable:", error)
        return DEFAULT_SCALE_FACTOR


def apply_actions(actions, controller, preferences, tare_settler, now_ms):
    """Apply hardware/persistence side effects requested by the controller."""

    if actions & ACTION_TARE:
        tare_settler.request(now_ms)
        controller.on_tare_pending(now_ms)

    if preferences is not None:
        try:
            if actions & ACTION_SAVE_MODE:
                preferences.save_mode(controller.state.mode)
            if actions & ACTION_SAVE_PROFILE:
                preferences.save_profile(controller.state.auto_profile)
        except Exception as error:
            if _DEBUG:
                print("preference save failed:", error)

    if _DEBUG and actions & ACTION_SESSION_STARTED:
        print("session started")
    if _DEBUG and actions & ACTION_SESSION_STOPPED:
        print("session stopped")


def print_telemetry(
    now_ms,
    raw_weight,
    filtered_weight,
    controller,
    tare_settler,
):
    """Emit replay-friendly diagnostics for physical auto-threshold tuning."""

    state = controller.state
    candidate = "none"
    if tare_settler.pending:
        candidate = "tare_wait"
    elif getattr(controller, "_auto_start_candidate_ms", None) is not None:
        candidate = "auto_start"
    elif getattr(controller, "_pour_drop_candidate_ms", None) is not None:
        candidate = "pour_drop"

    free_heap = gc.mem_free() if hasattr(gc, "mem_free") else -1
    print(
        "telemetry,{},{:.3f},{:.3f},{:.3f},{},{},{:.3f},{},{},{}".format(
            now_ms,
            raw_weight,
            filtered_weight,
            state.flow_gps,
            state.session_state,
            state.auto_profile,
            getattr(controller, "_session_peak_g", state.weight_g),
            candidate,
            getattr(controller, "last_stop_reason", ""),
            free_heap,
        )
    )


def main():
    micropython.alloc_emergency_exception_buf(100)

    config = BrewConfig()
    state = DisplayState(config)
    preferences = load_preferences(state)
    controller = MeasurementController(config, state)
    input_interpreter = InputInterpreter(config)
    event_queue = EventQueue(16)
    tare_settler = TareSettler(config)

    spi = SPI(
        2,
        baudrate=10 * 1024 * 1024,
        polarity=0,
        phase=0,
        sck=Pin(_SSD1322_SCK),
        mosi=Pin(_SSD1322_MOSI),
    )
    screen = SSD1322_SPI(
        width=_SSD1322_WIDTH,
        height=_SSD1322_HEIGHT,
        spi=spi,
        dc=Pin(_SSD1322_DC),
        res=Pin(_SSD1322_RES),
        cs=Pin(_SSD1322_CS),
        rotation=ROTATION_0,
    )
    renderer = DisplayRenderer(screen, config)

    right_button = Pin(_RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    left_button = Pin(_TIMER_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    right_touch = Pin(_MODE_TARE_TOUCH_PIN, Pin.IN)
    left_touch = Pin(_TIMER_TOUCH_PIN, Pin.IN)

    hx = HX711(dout=_HX711_DOUT, pd_sck=_HX711_SCK, gain=64)
    hx.set_scale(load_calibration_factor())
    hx.tare()
    weight_filter = RobustWeightFilter()
    initial_weight = hx.get_units(times=1)
    filtered_weight = weight_filter.update_estimate(initial_weight)

    vsense = ADC(Pin(_BAT_VOLTAGE_PIN))
    vsense.atten(ADC.ATTN_11DB)
    state.battery_percent = read_battery_percent(vsense)

    ble = BLE()
    scales = BLEScales(ble, command_sink=event_queue, debug=_DEBUG)
    scales.set_battery(state.battery_percent)

    now_ms = time.ticks_ms()
    controller.update(filtered_weight, now_ms)
    state.ble_connected = scales.connected
    renderer.render(state, now_ms)

    last_input_ms = time.ticks_add(now_ms, -_INPUT_POLL_MS)
    last_ble_ms = time.ticks_add(now_ms, -config.ble_publish_ms)
    last_render_ms = time.ticks_add(now_ms, -config.display_refresh_ms)
    last_telemetry_ms = now_ms

    gc.collect()

    while True:
        # Read the surface-mounted control before accepting another weight
        # sample. The debounced state stays true during release debounce, so
        # button force never enters the filter history.
        right_pressed = (
            right_button.value() == 0
            or right_touch.value() == _TOUCH_ACTIVE_LEVEL
        )
        tare_surface_active = right_pressed or input_interpreter.right_is_pressed
        raw_weight = hx.get_units(times=1)
        if not tare_surface_active:
            filtered_weight = weight_filter.update_estimate(raw_weight)
        now_ms = time.ticks_ms()
        did_tare = False

        if time.ticks_diff(now_ms, last_input_ms) >= _INPUT_POLL_MS:
            last_input_ms = now_ms
            left_pressed = (
                left_button.value() == 0
                or left_touch.value() == _TOUCH_ACTIVE_LEVEL
            )
            for event in input_interpreter.update(
                left_pressed,
                right_pressed,
                state.auto_menu_open,
                now_ms,
            ):
                event_queue.push(event)

        # A TARE+START BLE command arrives as two ordered events. Stop draining
        # while tare settles so START cannot run against the old HX711 offset.
        while event_queue and not tare_settler.pending:
            event = event_queue.pop()
            actions = controller.handle_event(event, now_ms)
            apply_actions(
                actions,
                controller,
                preferences,
                tare_settler,
                now_ms,
            )

        if tare_settler.update(filtered_weight, now_ms):
            hx.tare(times=5)
            now_ms = time.ticks_ms()
            weight_filter.reset(0.0)
            controller.on_tare(now_ms)
            did_tare = True
            raw_weight = 0.0
            filtered_weight = 0.0

        # Freeze automatic start/stop decisions while button force or platform
        # rebound may still be present in the measurement stream.
        if not tare_settler.pending and not tare_surface_active:
            actions = controller.update(filtered_weight, now_ms)
            apply_actions(
                actions,
                controller,
                preferences,
                tare_settler,
                now_ms,
            )

        state.ble_connected = scales.connected

        if time.ticks_diff(now_ms, last_ble_ms) >= config.ble_publish_ms:
            last_ble_ms = now_ms
            scales.set_measurement(
                state.weight_g,
                state.flow_gps,
                notify=True,
            )

        if time.ticks_diff(now_ms, last_render_ms) >= config.display_refresh_ms:
            last_render_ms = now_ms
            renderer.render(state, now_ms)

        if _TELEMETRY_ENABLED and time.ticks_diff(
            now_ms, last_telemetry_ms
        ) >= _TELEMETRY_PERIOD_MS:
            last_telemetry_ms = now_ms
            print_telemetry(
                now_ms,
                raw_weight,
                filtered_weight,
                controller,
                tare_settler,
            )

        # Keep the variable explicit so a debugger can confirm a tare was
        # physically applied before any ordered BLE START event is processed.
        if did_tare and _DEBUG:
            print("tare applied:", tare_settler.last_reason)

        time.sleep_ms(1)


if __name__ == "__main__":
    main()
