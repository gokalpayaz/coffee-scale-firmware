"""Main file running on the scales ESP32."""
import micropython
import esp32
import machine
import time
import _thread
from bluetooth import BLE
from machine import ADC, I2C, Pin
from micropython import const
from art import BATTERY, DOT, GRAM, LOGO, show_digit, show_sprite
from ble_scales import BLEScales
from debounce import DebouncedSwitch
from filtering import KalmanFilter
from hx711 import HX711
from ssd1306 import SSD1306_I2C

### constants ###

_BAT_SWITCH_PIN   = const(2) # en/disables A13/IO35 to read battery voltage
_BAT_VOLTAGE_PIN  = const(33) # A13
_RESET_BUTTON_PIN = const(25) # short press to tare scale
_SLEEP_BUTTON_PIN = const(26) # press 1s to deepsleep, short press to wake
_I2C_SCL = const(22)
_I2C_SDA = const(23)
_SSD1306_WIDTH  = const(128)
_SSD1306_HEIGHT = const(32)
_HX711_DOUT = const(14)
_HX711_SCK  = const(13)
_CALIBRATION_FACTOR = 2174.6108 # scale calibration factor
_DEBUG = True

# check if the device woke from a deep sleep
if machine.reset_cause() == machine.DEEPSLEEP_RESET:
  if _DEBUG: print('woke up from deepsleep')

micropython.alloc_emergency_exception_buf(100)

### pin/module setup ###

# oled display
i2c = I2C(-1, scl=Pin(_I2C_SCL), sda=Pin(_I2C_SDA))
screen = SSD1306_I2C(width=_SSD1306_WIDTH, height=_SSD1306_HEIGHT, i2c=i2c)
# clear screen and display logo
screen.fill(0)
show_sprite(screen, LOGO, 51, 1)
screen.show()

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
sleep_button = Pin(_SLEEP_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

### callback functions ###

def reset_callback(arg):
    global hx, kf
    if _DEBUG: print('tare scale')
    hx.tare(times=3)
    kf.last_estimate = 0.0

def sleep_callback(arg):
    global reset_button, sleep_button, hx, kf
    if _DEBUG: print('deepsleep')
    # power down display module
    screen.poweroff()
    # power down load cell amp module
    hx.power_down()
    # disable pull-down on sleep button
    sleep_button.init(pull=None)
    # change handler to wake esp32 on button push
    reset_button.irq(handler=None)
    esp32.wake_on_ext0(pin=reset_button, level=esp32.WAKEUP_ANY_HIGH)
    machine.deepsleep()

### interrupts ###

reset_sw = DebouncedSwitch(sw=reset_button, cb=reset_callback)
sleep_sw = DebouncedSwitch(sw=sleep_button, cb=sleep_callback, delay=1000)


### functions ###

def adc_to_percent(v_adc):
    # with divider and adc, it seems pretty much linear. No need for maping in this case
    ADC_FULL  = 2530   # 4.2 V
    ADC_EMPTY = 2070   # ~3.5 V (reset eşiği)
    if v_adc >= ADC_FULL:
        return 100
    if adc <= ADC_EMPTY:
        return 0
    return int((v_adc-ADC_EMPTY)*100/(ADC_FULL-ADC_EMPTY))


def display_weight():
    global filtered_weight, bat_percent
    while True:
        screen.fill(0)
        rounded_weight = round(filtered_weight / 0.05) * 0.05
        string = '{:.2f}'.format(rounded_weight)
        if len(string) > 6:
            string = '{:.1f}'.format(rounded_weight)
        if string == '-0.00':
            string = '0.00'
        position = 118
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
        show_sprite(screen, GRAM, 117, 16)
        if bat_percent <= 20:
            show_sprite(screen, BATTERY, 117, 1)
        screen.show()


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
    scales.set_battery_level(bat_percent)

    # start display_weight() in a thread
    _thread.start_new_thread(display_weight, ())

    last = 0
    while True:
        weight = hx.get_units(times=1)
        filtered_weight = kf.update_estimate(weight)
        now = time.ticks_ms()
        if time.ticks_diff(now, last) > 100:
            last = now
            rounded_weight = round(filtered_weight / 0.05) * 0.05
            scales.set_weight(rounded_weight, notify=True)
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
