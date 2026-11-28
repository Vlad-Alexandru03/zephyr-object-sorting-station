# Fixed-position sorting

The conveyor UNO Q uses its encoder PI controller to follow the potentiometer
setting. The phone sends images to Qbot, which runs YOLO and serves the
annotated image at `/latest.jpg` and detection labels at `/latest.json`.
The PC reads Qbot over the network or the existing ADB forward. It talks to
the conveyor UNO Q through USB and to the SO-101 through its USB serial port.

An ultrasonic reading under 100 mm stops the belt on the MCU. The PC waits
for zero PWM and quiet encoder feedback, then uses a fresh camera frame.
YOLO identifies the category. The PC subtracts a taught empty-belt image
inside the detection box and fits an OpenCV rotated rectangle. Its center and
angle are compared with the location and orientation taught for that class.
If the box is upstream in the right lane, the MCU nudges the belt forward and
the PC rechecks after it stops. Each jog cuts PWM after the first encoder edge
or 450 ms. Actual travel may include coasting. The arm starts only when the
position and angle are both acceptable. Use a guide to turn or center boxes
that forward motion alone cannot align. The category selects its taught
waypoints. The arm moves to the taught pickup hover, settles 12 mm above the
grasp, then lowers slowly. It closes to the configured servo-load target,
lifts straight up 30 mm, then carries to the bin.
Use a fixed pickup stop or guides so the item lands at the grasp point. The
camera position is used to decide whether the box is in the pickup area. The
pickup and bin positions come from the waypoint files.

The default carry speed is 25 degrees per second and can be set with
`--carry-speed`. The final approach still moves more slowly than the carry.

The belt stays stopped through drop-off and the return to observation. It
waits two more seconds, then checks the ultrasonic sensor for a clear pickup
area before restarting. Camera-alignment mode also checks the camera.
Set `--restart-delay` to change the pause in seconds. A failed grasp or missing
teach file leaves the belt stopped.
The conveyor waits stopped if a fresh image is unavailable after detection,
or if the PC stops renewing its run command. Verify the jog distance. If
the initial stop overshoots, adjust the ultrasonic threshold or move the
sensor upstream.

## Prepare the conveyor UNO Q

Use the existing wiring in the [conveyor README section](../README.md#uno-q-conveyor-firmware).
From the separate UNO Q west workspace, build the sorting firmware:

```powershell
& .\.venv\Scripts\west.exe build -b arduino_uno_q/stm32u585xx -d .\build-unoq-sort `
  ..\Personal_Project\zephyr-object-sorting-station\firmware\unoq-conveyor -- `
  -DUNOQ_SORT_STATION=ON -DUNOQ_SPEED_PI=ON
```

Keep Qbot on its existing matrix firmware. Update `vision_server.py` on Qbot
and restart the vision server so source images carry matching capture IDs.
With conveyor motor power off,
connect both boards and verify their serial numbers with `adb devices`.
From the UNO Q workspace, start the debugger in one PowerShell terminal:

```powershell
$adb = "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\adb\32.0.0\adb.exe"
& $adb -s 744557213 forward tcp:3333 tcp:3333
& $adb -s 744557213 shell arduino-debug
```

From a second PowerShell terminal in the same workspace, load the firmware:

```powershell
& "$env:USERPROFILE\zephyr-sdk-1.0.1\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gdb.exe" `
  .\build-unoq-sort\zephyr\zephyr.elf `
  -ex 'target extended-remote localhost:3333' -ex 'monitor reset halt' `
  -ex 'load' -ex 'monitor reset run' -ex 'disconnect' -ex 'quit'
```

From `Personal_Project`, install the UART bridge on the conveyor UNO Q and
point the PC preview at Qbot:

```powershell
$adb = "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\adb\32.0.0\adb.exe"
$conveyor = '744557213'
$qbot = '2624126501'
& $adb -s $conveyor push .\zephyr-object-sorting-station\tools\conveyor_uart.py /home/arduino/conveyor_uart.py
& $adb -s $qbot forward tcp:18000 tcp:8000
```

If `arduino-router` is running on the **conveyor** UNO Q, stop it first so the
bridge can use `/dev/ttyHS1` exclusively:

```powershell
& $adb -s $conveyor shell
```

```sh
sudo systemctl stop arduino-router
exit
```

If the stop job is canceled on the newer Linux image, run this on the
conveyor UNO Q and verify the router shows `inactive`:

```sh
sudo systemctl mask --runtime --now arduino-app-cli.service arduino-router-serial.service arduino-router.service
systemctl is-active arduino-router.service
```

The bridge stays in `/home/arduino` across reboots. The runtime masks expire on
reboot, so stop the router again before running the station.

## Check and run one cycle

Keep the phone fixed and start live capture. Use an ROI that covers both the
taught pickup position and a short stretch upstream. The crop `500 170 750 420`
was used on Qbot for the latest phone view. This workflow needs no camera-to-arm
registration file.

Keep the `observation` waypoint in `local/station.json`. For every entry
under `objects`, give the matching class file an `object_label`, `above_pickup`,
`grasp`, and `above_bin` as described in the
[waypoint guide](camera-guided-pickup.md#automatic-pickup-with-retries).
The local `Small box` profile can be used for the first run where those poses
still match the hardware. Teach another class before sorting it.

From `Personal_Project`:

```powershell
$python = '.\.so101-venv\Scripts\python.exe'
$repo = '.\zephyr-object-sorting-station'
$sortOptions = @(
    "$repo\local\station.json"
    '--qbot', 'http://127.0.0.1:18000'
)
& $python -B "$repo\tools\sort_station.py" @sortOptions --roi 500 170 750 420
& $python -B "$repo\tools\sort_station.py" @sortOptions `
  --adb $adb --conveyor $conveyor --check-conveyor
```

The preview prints any class in the pickup area. `--check-conveyor` checks
the link and reports an ultrasonic reading while keeping the belt stopped.
Teach the camera with the motor off and the arm out of the camera's view.
Run the first command with the belt empty, the second with the box correctly
positioned and rotated at the taught grasp, and the third with the box a few
centimeters upstream on the same lane. Step away before each capture.

```powershell
$pose = "$repo\local\pickup-pose.json"
& $python -B "$repo\tools\sort_station.py" @sortOptions --pose-config $pose `
  --roi 500 170 750 420 --teach-empty
& $python -B "$repo\tools\sort_station.py" @sortOptions --pose-config $pose `
  --teach-target 'Small box'
& $python -B "$repo\tools\sort_station.py" @sortOptions --pose-config $pose `
  --teach-upstream 'Small box'
```

If quarter turns are interchangeable for your box, add `--angle-period 90`
when teaching the target. Normal rectangular boxes use 180 degrees, which
treats opposite ends as equivalent. Use `--angle-period 0` only if rotation
does not affect the grasp. If the phone, zoom, or lighting changes, teach the
empty image and the two target poses again.

For the powered test, start with the pot at zero, the servos holding, and the
gripper open and empty. The controller holds the belt stopped while it plans
and executes a return to observation. If it cannot plan a return from the
current pose, start from a raised pose with the route clear. Clear the belt
and bin path before turning the pot up.

```powershell
& $python -B -u "$repo\tools\sort_station.py" @sortOptions `
  --pose-config $pose --adb $adb --conveyor $conveyor --port COM7 `
  --pickup-wait 45 --cycles 1 --execute
```

The default is continuous sorting. Use `--cycles 1` for one item.
`--stop-mm` adjusts the ultrasonic threshold.
`--pickup-position-px`, `--pickup-angle-deg`, and `--max-jogs` adjust the pose
tolerances and the number of forward corrections.

## Stop on ultrasound and sort by camera label

With `--ultrasonic-only`, the MCU stops the belt and the PC waits for a fresh
camera frame. Put one item on the belt at a time. A `Small box` or `Tape roll`
detection selects that label's taught pickup and bin. Camera alignment and belt
jogging are skipped, so the stopped item must be within reach of its taught grasp.
If the camera cannot identify one configured object within `--classify-wait`
seconds (default 15), the belt stays stopped. `--object` can specify a fallback
for a controlled single-object test.

```powershell
& $python -B -u "$repo\tools\sort_station.py" "$repo\local\station.json" `
  --ultrasonic-only --roi 500 170 750 420 --max-frame-age 10 `
  --adb $adb --conveyor $conveyor --port COM7 --stop-mm 45 --execute --dashboard 8767
```

The ROI must be the one used when the pickup pose was taught. A different ROI
filters the detection out and the run stops with "Expected one object in the
pickup area. Camera found 0."

Tune `--stop-mm` so the ultrasonic stop happens where the box coasts to the taught
grasp point. A larger cutoff stops the box earlier on the belt.

## Record a demo with the live dashboard

To check the layout before teaching the camera, run the preview from
`Personal_Project` and open `http://127.0.0.1:8766/`:

```powershell
& .\.so101-venv\Scripts\python.exe -B .\zephyr-object-sorting-station\tools\station_dashboard.py
```

Add `--dashboard` to the run command and open `http://127.0.0.1:8765/` on
the PC. The live camera panel refreshes as Qbot receives phone frames.
The inspection snapshot stays visible alongside the ultrasonic stop distance,
encoder speed, and arm destination. The dashboard remains open after the cycle.
Press Ctrl+C to close it.

Encoder speed is shown in RPM by default. If you measure how far the belt
travels per encoder revolution, pass `--belt-mm-per-rev` followed by that
distance in millimeters to display belt travel in mm/s.

## Test the arm on its own

Turn off conveyor motor power and place the Small box at its taught pickup
point. With the servos holding, the gripper open and empty, and the arm path
clear, run one pickup without the camera or conveyor connection:

```powershell
& $python -B -u "$repo\tools\sort_station.py" "$repo\local\station.json" `
  --port COM7 --arm-only 'Small box'
```
