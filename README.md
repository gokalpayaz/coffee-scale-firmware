# DIY Smart Coffee and Espresso Scale

This is a modified version of the amazing project by **Valentin Bersier**, original repo at https://github.com/beeb/coffee-scale-app. This repo only contains the firmware for the **ESP32 microcontroller**, go to the original repo for the rest of the software.

## Documentation
Refer to the [Wiki](https://github.com/Nkawu/coffee-scale-firmware/wiki) for detailed instructions on how to install this firmware on your scale

## Modification details
For this version of the scale, the 3D printed housing was completely redesigned to fit on a Breville Dual Boiler espresso machine. The 3D printable parts are available from [Printables - Bluetooth Espresso Scale ESP32 Feather based](https://www.printables.com/model/213101-bluetooth-espresso-scale-esp32-feather-based)

An ESP32-based **Feather** microcontroller board is used in place of the **ESP32 Thing** of the original. This board is available from [EzSBC.com](https://www.ezsbc.com/product/esp32-feather/) and vastly reduces the original Adafruit Feather's high deepsleep current draw of 160mA (IIRC) down to 10uA.

The current hardware exposes logical left and right controls through both the
original tactile inputs and external digital-touch modules. The firmware polls
and debounces these inputs together so either control surface produces the same
actions. Deep-sleep behavior is not part of the current V2 control mapping.

The `firmware` folder now contains only the files used by the running scale.
Upload `boot.py` and `main.py` to the ESP32 root, and upload the contents of
`firmware/lib` to `/lib`. Do not upload `extras` or `tests`.

## Firmware V2 controls and display modes

The 256x64 SSD1322 display uses its full width and cycles through four layouts:

- Timer + weight
- Timer + flow + weight, including a flow graph
- Timer + ratio + weight
- Timer + flow + ratio + weight

Weight is displayed at 0.1 g resolution, matching the practical resolution of
the current 1 kg load cell. Internally, unrounded measurements pass through a
five-sample median and adaptive smoothing filter before flow, ratio, automatic
start/stop, BLE, and display processing.

The scale treats the tactile and digital-touch inputs as the same logical left
and right buttons:

- Right short press: tare (blocked while a measurement is running)
- Right long press: next display mode
- Left short press: start/stop or resume the timer
- Left long press: reset timer, flow, ratio, and graph history
- Hold both buttons for one second: open the automatic-profile selector

Because the tare button is mounted on the weighing surface, tare is deferred
after release. The firmware waits at least 350 ms and then requires 300 ms of
stable weight before sampling the HX711 offset; a two-second timeout prevents a
request from waiting indefinitely.

In the automatic-profile selector, use the right button to choose Espresso or
Pour-over and the left button to confirm. Automatic measurement waits for a
stable zero, then starts after a persistent 0.5 g increase. Espresso stops on a
stable plateau after 15 g; Pour-over stops after a sustained large weight drop.
The 15 g dose is also the current ratio denominator and is intentionally fixed
until the companion-app command is designed.

The last display mode and automatic profile are stored in ESP32 NVS. A running
or armed session is never restored after reboot.

### Threshold telemetry and host tests

Set `_TELEMETRY_ENABLED = True` in `firmware/main.py` to emit CSV-like tuning
records containing raw/filtered weight, flow, state, profile, peak weight,
candidate condition, stop reason, and free heap. Keep it disabled for normal
operation.

Run the device-independent test suite before copying the firmware to the ESP32:

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
py -3 -m unittest discover -s tests -v
```

Automatic thresholds, touch polarity, display orientation, and OLED
readability still require validation on the physical scale after flashing.

### Load-cell calibration

The complete calibration can be started and completed on the scale itself:

1. Start with the scale powered off and empty.
2. Hold both logical controls while powering it on. Keep holding for the
   three-second on-screen countdown, then release both controls.
3. With the platform empty, press left and keep hands off while it measures.
4. Place an accurate 100 g reference weight on the platform, press left, and
   keep hands off again.
5. After `CALIBRATION SAVED` appears, remove the 100 g weight and press left to
   continue into normal operation.

Either the tactile inputs or the digital-touch modules can be used. Press right
at either capture prompt to cancel without replacing the stored factor. The
boot-only three-second chord is deliberately longer than the normal one-second
automatic-profile chord.

The serial utility remains available under `extras/calibration` for diagnostics
and one-count-per-gram fine adjustment, but it is not part of the normal device
image. To use it, temporarily upload `extras/calibration/calibrate.py` to the
ESP32 root and `extras/calibration/lib/debounce.py` to `/lib`. Connect to the
ESP32 serial REPL, interrupt the running firmware with `Ctrl+C`, then run:

```python
import calibrate
calibrate.main()
```

Follow the serial prompts using the same accurate 100 g reference weight. Once
the weight is replaced, the left button decreases and the right button
increases the scale factor in one-count-per-gram steps. The utility prints both
the measured weight and the current factor. It saves the calculated value and
every fine adjustment to ESP32 NVS automatically; normal firmware loads the
saved factor at boot.

## Modified Espresso Workflow

- Power on the scale by pressing the right button. It should take a few seconds to start up and enable the display.
- Place a measuring cup on the scale and tare with the right button.
- Place the desired amount of coffee beans into the measuring cup.
- Grind the beans in your grinder.
- Place the empty portafiler on the scale and tare. If you grind directly into the portafilter, do this before grinding the beans.
- Place the portafilter filled with ground coffee back on the portafilter, adjust the required coffee dose, e.g. 18g
- Connect to the [Scale Web App](http://beeb.li/coffee) on your phone or tablet as prompted. If it's an Apple device, you may need to use [Bluefy – Web BLE Browser](https://apps.apple.com/us/app/bluefy-web-ble-browser/id1492822055).
- Enter the parameters for your shot in the app:
  - My espresso machine has automatic pre-infusion, so I just set `Pre-infusion time` to 0 and `Total time` to my required shot time.
  - The "Read" button next to the "Coffee Weight" input can be clicked to read the current scale value into it.
- Place a cup on the scale and tare with the right button
- Press the "Start recording" button (only available when the scale reads ~0g). The app now waits for an increase in weight.
- Start the shot extraction on your espresso machine.
- As soon as the weight exceeds 0.5g, the timer starts counting from the pre-infusion time (default 5s).
- The extraction can be followed in real time and should match the grey reference curve.
- Stop the shot extraction when the required extracted coffee weight is shown on the scale.
- Lift the cup from the scale to stop the recording.

## License

Distributed under the MIT License
