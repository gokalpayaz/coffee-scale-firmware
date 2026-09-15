"""Main file running on the scales ESP32."""
import micropython
import esp32
import machine
import time
import _thread
from bluetooth import BLE
from machine import ADC, Pin, SPI, Timer
from micropython import const
from art import BATTERY, DOT, GRAM, LOGO, show_digit, show_sprite
from ble_scales import BLEScales
from debounce import DebouncedSwitch
from filtering import KalmanFilter
from hx711 import HX711
from ssd1322 import SSD1322_SPI
from smart_switch import SmartSwitch

### constants ###

_BAT_SWITCH_PIN   = const(2) # en/disables A13/IO35 to read battery voltage
_BAT_VOLTAGE_PIN  = const(33) # A13
_RESET_BUTTON_PIN = const(25) # short press to tare scale
_TIMER_BUTTON_PIN = const(26) # short press to arm timer
_SSD1322_SCK = const(18)   # J1 pin 4: D0/CLK
_SSD1322_MOSI = const(23)  # J1 pin 5: D1/DIN
_SSD1322_CS = const(19)    # J1 pin 16: CS#
_SSD1322_DC = const(21)    # J1 pin 14: D/C#
_SSD1322_RES = const(27)   # J1 pin 15: RES#
_SSD1322_WIDTH = const(256)
_SSD1322_HEIGHT = const(64)
_HX711_DOUT = const(13)
_HX711_SCK  = const(14)
# External touch-sensor modules with ordinary digital outputs.
_MODE_TARE_TOUCH_PIN = const(4)   # short press = tare, long press = mode
_TIMER_TOUCH_PIN = const(32)      # short press = timer
_TOUCH_ACTIVE_LEVEL = const(1)    # Change to 0 if the module reports touch as LOW.
_TOUCH_SAMPLE_MS = const(20)
_TOUCH_DEBUG_MS = const(500)
_MODE_LONG_PRESS_MS = const(1000)
_CALIBRATION_FACTOR = 2174.6108 # scale calibration factor
_DEBUG = True

# check if the device woke from a deep sleep
# if machine.reset_cause() == machine.DEEPSLEEP_RESET:
#   if _DEBUG: print('woke up from deepsleep')

micropython.alloc_emergency_exception_buf(100)

### pin/module setup ###

# OLED display: SSD1322 4-wire SPI, powered from the ESP32 3V3 rail.
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
)

# bluetooth
ble = BLE()
if _DEBUG: print('bt loaded')
scales = BLEScales(ble)
kf = KalmanFilter(0.03, q=0.1)

# voltage sense
# switch not needed when using a physical switch
# vsense_switch = Pin(_BAT_SWITCH_PIN, Pin.OUT)
vsense_switch = True
vsense = ADC(Pin(_BAT_VOLTAGE_PIN))  # A13 or PIN 35
vsense.atten(ADC.ATTN_11DB)

bat_percent = 0

# hx711 load cell amp
hx = HX711(dout=_HX711_DOUT, pd_sck=_HX711_SCK, gain=64)
hx.set_scale(_CALIBRATION_FACTOR)
hx.tare()
kf.update_estimate(hx.get_units(times=1))
filtered_weight = 0

# buttons
reset_button = Pin(_RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
timer_button = Pin(_TIMER_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# External touch-sensor digital inputs. No internal pull is used because the
# modules are expected to drive their outputs actively.
mode_tare_touch = Pin(_MODE_TARE_TOUCH_PIN, Pin.IN)
timer_touch = Pin(_TIMER_TOUCH_PIN, Pin.IN)

# timer
tim = Timer(0)
timer_running = False
duration = 0
display_timer = False
arm_tim = Timer(0)
### callback functions ###

def reset_callback(arg):
    global hx, kf
    if _DEBUG: print('tare scale')
    hx.tare(times=3)
    kf.last_estimate = 0.0
    
    
def timer_button_callback(arg):
    global timer_running, tim, duration, display_timer
    if _DEBUG: print('timer button pressed')
    display_timer = True
    if not timer_running:
        tim.init(period=1000, mode=Timer.PERIODIC, callback=timer_tick_callback)
        timer_running = True
    else:
        tim.deinit()
        timer_running = False
        duration = 0
    
def timer_tick_callback(arg):
    global tim, duration
    duration += 1
  
def arm_timer_callback(arg):
    global arm_tim, filtered_weight
    if _DEBUG: print('timer button long pressed') 
    arm_tim.init(period=50, mode=Timer.PERIODIC, callback=arm_timer_tick_callback)


def arm_timer_tick_callback(arg):
    global arm_tim, tim, filtered_weight, display_timer, timer_running
    if timer_running:
        return
    if filtered_weight >= 0.2:
        if _DEBUG: print('Armed timer started')
        arm_tim.deinit()
        tim.init(period=1000, mode=Timer.PERIODIC, callback=timer_tick_callback)
        display_timer = True
        timer_running = True


def mode_touch_long_press():
    # Mode selection is intentionally only a debug event until modes are defined.
    if _DEBUG: print('mode touch long pressed')


def poll_touch_buttons(state, now):
    """Print digital input levels and dispatch simple short/long touch events."""
    mode_value = mode_tare_touch.value()
    timer_value = timer_touch.value()

    if time.ticks_diff(now, state['last_debug']) >= _TOUCH_DEBUG_MS:
        state['last_debug'] = now
        print(
            'touch mode/tare GPIO{}={} timer GPIO{}={}'.format(
                _MODE_TARE_TOUCH_PIN, mode_value, _TIMER_TOUCH_PIN, timer_value
            )
        )

    for name, value, short_callback, long_callback in (
        ('mode/tare', mode_value, reset_callback, mode_touch_long_press),
        ('timer', timer_value, timer_button_callback, None),
    ):
        touched = value == _TOUCH_ACTIVE_LEVEL
        pressed_at = state[name]

        if touched and pressed_at is None:
            state[name] = now
            if _DEBUG: print('{} touch pressed ({})'.format(name, value))
        elif not touched and pressed_at is not None:
            state[name] = None
            held_ms = time.ticks_diff(now, pressed_at)
            if _DEBUG: print('{} touch released after {}ms'.format(name, held_ms))
            if long_callback is not None and held_ms >= _MODE_LONG_PRESS_MS:
                long_callback()
            else:
                short_callback(None)
    
    
# def sleep_callback(arg):
#     global reset_button, sleep_button, hx, kf
#     if _DEBUG: print('deepsleep')
#     # power down display module
#     screen.poweroff()
#     # power down load cell amp module
#     hx.power_down()
#     # disable pull-down on sleep button
#     sleep_button.init(pull=None)
#     # change handler to wake esp32 on button push
#     reset_button.irq(handler=None)
#     esp32.wake_on_ext0(pin=reset_button, level=esp32.WAKEUP_ANY_HIGH)
#     machine.deepsleep()

### interrupts ###

reset_sw = DebouncedSwitch(sw=reset_button, cb=reset_callback)
smart_sw = SmartSwitch(
    pin=timer_button, 
    short_cb=timer_button_callback, 
    long_cb=arm_timer_callback, 
    long_press_ms=1000 # 1 saniye sınırı
)

### functions ###

def adc_to_percent(v_adc):
    # with divider and adc, it seems pretty much linear. No need for maping in this case
    ADC_FULL  = 2530   # 4.2 V
    ADC_EMPTY = 2070   # ~3.5 V (reset eşiği)
    if v_adc >= ADC_FULL:
        return 100
    if v_adc <= ADC_EMPTY:
        return 0
    return int((v_adc-ADC_EMPTY)*100//(ADC_FULL-ADC_EMPTY))


def display_weight():
    global filtered_weight, bat_percent, display_timer, duration
    while True:
        screen.fill(0)
        
        if display_timer:
            time_str = format_time(duration)
            # Sol üst köşeye yerleştir
            screen.text(time_str, 0, 1, screen.color_on)
        else:
            # Sadece timer yokken o büyük G sprite'ını göster
            show_sprite(screen, GRAM, 117, 16)
        
        rounded_weight = round(filtered_weight / 0.05) * 0.05
        if display_timer and rounded_weight >= 100:
            string = '{:.1f}'.format(rounded_weight)
        else:
            string = '{:.2f}'.format(rounded_weight)
            
        if len(string) > 6:
            string = '{:.1f}'.format(rounded_weight)
        if string == '-0.00':
            string = '0.00'
        position = 125 if display_timer else 118
        
        for char in reversed(string):
            if position < 0:
                break
            if char == '-':
                char = 'MINUS'
            if char == '.':
                position -= 7
                if position < 0:
                    break
                show_sprite(screen, DOT, position, 27)
            else:
                position -= 22
                if position < 0:
                    break
                show_digit(screen, char, position, 1)
        if not display_timer:            
            show_sprite(screen, GRAM, 117, 16)
        if bat_percent <= 20:
            show_sprite(screen, BATTERY, 117, 1)
        screen.show()        
        time.sleep_ms(20)

def format_time(seconds):
    minutes = seconds // 60
    secs = seconds % 60
    return "{}:{:02d}".format(minutes, secs)

### main block ###

def main():
    # global filtered_weight, bat_percent, scales, reset_button, hx, kf
    global filtered_weight, bat_percent, vsense_switch, scales, hx, kf

    # uncomment next 2 lines to get a load cell reading for calibration (in the console/serial)
    # while True:
    #    print(hx.read_average(times=100))

    battery_sum = 0

    # read battery voltage (en/disable with IO2)
    # vsense_switch.on()
    time.sleep_ms(10) # wait 10ms for A13(IO35) to turn on
    for i in range(10):
        battery_sum += vsense.read()
    # vsense_switch.off()
    
    # convert huzzah adc reading max 2369 to thing 2458
    bat_percent = adc_to_percent(int(battery_sum * 0.10375685943436))
    if _DEBUG: print('bat_percent={}'.format(bat_percent))
    scales.set_battery(bat_percent)

    # start display_weight() in a thread
    _thread.start_new_thread(display_weight, ())

    last = 0
    touch_last = 0
    touch_state = {
        'mode/tare': None,
        'timer': None,
        'last_debug': 0,
    }
    while True:
        weight = hx.get_units(times=1)
        filtered_weight = kf.update_estimate(weight)
        now = time.ticks_ms()
        if time.ticks_diff(now, touch_last) >= _TOUCH_SAMPLE_MS:
            touch_last = now
            poll_touch_buttons(touch_state, now)
        if time.ticks_diff(now, last) > 100:
            last = now
            rounded_weight = round(filtered_weight / 0.05) * 0.05
            scales.set_weight(rounded_weight, notify=True)
            print(rounded_weight)
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
