import utime
from machine import Pin
class SmartSwitch:
    def __init__(self, pin, short_cb, long_cb, long_press_ms=1000):
        self.pin = pin
        self.short_cb = short_cb
        self.long_cb = long_cb
        self.long_press_ms = long_press_ms
        self.press_time = 0
        
        # Interrupt for either and rising edge
        self.pin.irq(handler=self._handle_irq, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

    def _handle_irq(self, pin):
        if pin.value() == 0:  # Button pressed. (Assuming rising edge)
            self.press_time = utime.ticks_ms()
        else:  # Button pressed
            if self.press_time > 0:
                duration = utime.ticks_diff(utime.ticks_ms(), self.press_time)
                self.press_time = 0
                
                try:
                    if duration < self.long_press_ms:
                        self.short_cb(pin)
                    else:
                        self.long_cb(pin)
                except TypeError:
                    # If function doesn't accept arg, send it argless
                    if duration < self.long_press_ms:
                        self.short_cb()
                    else:
                        self.long_cb()
