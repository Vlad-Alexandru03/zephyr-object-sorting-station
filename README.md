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

## First milestone

Press Start and display the object label, `empty`, or `unknown`.

## Later milestone

Use the SO101 arm to sort recognized objects into taught drop-off locations,
then take a second snapshot to verify that the slot is empty.
