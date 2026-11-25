# Zephyr Object Sorting Station

A one-slot station that identifies objects from a camera snapshot.

The ESP32-S3 with an OV3660 captures a 2048 × 1536 JPEG when you press BOOT.
It stops Wi-Fi during capture, then reconnects to send the JPEG. The Arduino UNO Q
runs object recognition on Debian and the station controller on Zephyr.

## Camera and vision server

Apply `patches/zephyr-camera-jpeg.patch` to a fresh Zephyr checkout at the
revision in `west.yml`. The patch adds the DMA support used by the camera app.
Build `firmware/vision` for `esp32s3_eye/esp32s3/procpu`. Set `QBOT_IP` in
`firmware/vision/src/main.c` to the UNO Q's IP address before building.

Copy `tools/vision_server.py` and `tools/detect_snapshot.py` to the same directory
on the UNO Q. The server needs PyTorch, torchvision, and Pillow. To match the
tested setup, run it with pyserial and `djpeg` installed:

```sh
python vision_server.py --matrix-port /dev/ttyHS1 --jpeg-checker djpeg
```

Connect the ESP32 to Wi-Fi once after boot. Press BOOT to take a photo, then
open `http://<UNO-Q-IP>:8000/latest.jpg` to see the result.

## UNO Q matrix firmware

`firmware/unoq-matrix` runs on the UNO Q's STM32. It reads detection labels and
scores sent by the Linux vision server at 115200 baud and scrolls them on the
LED matrix.

Build it in the separate UNO Q west workspace. Its Zephyr checkout was tested
at commit `270ee49bd8fe412446d862a56aae16a0e245104d`. This checkout provides
the board's `zephyr,display` node, which the camera workspace's Zephyr version
does not define.

From the UNO Q workspace in PowerShell, with this repo in
`..\Personal_Project\zephyr-object-sorting-station`:

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-matrix-repo ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-matrix
```

## UNO Q conveyor firmware

Build `firmware/unoq-conveyor` in the UNO Q west workspace with the same
`arduino_uno_q/stm32u585xx` target used for the matrix firmware. Flash it to the
conveyor UNO Q. The AI board can continue running the matrix firmware and Linux
vision server.

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-conveyor ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-conveyor
```

- L298N channel B uses D3 for ENB (PWM), D6 for IN3, and D7 for IN4.
- The potentiometer wiper connects to A0, with its outer terminals on 3V3 and GND.
- The optical encoder uses 3V3 and GND, with D0 on D2.
- The HC-SR04 uses 5V and GND, with TRIG on D8 and ECHO on D9 through a
  divider (2.2 kΩ from ECHO to D9 and 3.3 kΩ from D9 to GND).

Connect the UNO Q and L298N grounds. Remove the ENB jumper when driving ENB
from D3. Leave motor power off during the first input checks.

The firmware writes knob voltage, distance in millimeters, cumulative encoder
pulses, and PWM duty to the Linux-side MCU UART (`/dev/ttyHS1`, 115200 baud).
The motor stays off until the knob reaches zero after boot. After that the knob
sets 0–50% PWM, and a valid distance below 100 mm stops the belt. A distance of
`-1` indicates no echo.

### Motor step response

Build the test separately from the normal conveyor firmware:

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-conveyor-step ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-conveyor -- -DUNOQ_STEP_TEST=ON
```

The test runs at 35% PWM for five seconds, then 60% for five seconds. Use a
motor supply rated for the motor and keep the belt clear. Turn the knob fully
down, then fully up to start. Turning it fully down or bringing an object within
100 mm aborts the run. Restart the MCU for another run.

Capture the MCU UART in the conveyor UNO Q's Linux terminal before turning
the knob. The Arduino router also uses this UART, so stop it while recording
and restart it afterward.

```bash
(
    sudo systemctl stop arduino-router || exit 1
    trap 'sudo systemctl start arduino-router' EXIT
    stty -F /dev/ttyHS1 raw 115200 -crtscts
    timeout 45s cat /dev/ttyHS1 | tee ~/unoq-step.log
)
```

Copy `unoq-step.log` to your PC and run the plotter with the number of slots
on the encoder disk:

```powershell
& .\.zephyr-venv\Scripts\python.exe .\zephyr-object-sorting-station\tools\plot_motor_step.py --log .\unoq-step.log --pulses-per-rev 20 --output .\unoq-motor-step
```

For the automatic 60% to 90% test, build a separate image:

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-conveyor-auto-step ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-conveyor -- -DUNOQ_AUTO_STEP_TEST=ON
```

This image starts automatically five seconds after boot and holds each duty
for ten seconds. The knob and ultrasonic sensor have no effect in this mode.
Switch off the L298N motor supply if you need to stop the run early. The test
aborts before stepping up if the encoder saw no pulses in the final second at
60%. The same recorder and plotter read both test modes.

### Encoder speed control

[Motor PI tuning notes](docs/motor-pi-tuning.md) show the gain calculation,
the backward-Euler update, and the control loop.

Build the PI controller in a separate directory:

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-conveyor-pi ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-conveyor -- -DUNOQ_SPEED_PI=ON
```

Turn the knob fully down after boot. Above the stop position the knob sets
a target between 105 and 120 RPM. The controller estimates speed over about
500 ms using a 20-slot encoder disk, adds a PI correction to the duty estimated
from the step tests, and limits PWM to 90%. The gains are 0.234 PWM percentage
points per RPM and 0.635 PWM percentage points per RPM-second. A valid object
detection within 100 mm overrides the controller. If encoder pulses stop,
turn the knob to zero before trying again.

## First milestone

Press Start and display the object label, `empty`, or `unknown`.

## Later milestone

Use the SO101 arm to sort recognized objects into taught drop-off locations,
then take a second snapshot to verify that the slot is empty.
