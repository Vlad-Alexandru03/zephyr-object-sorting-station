"""Receive BOOT-triggered RGB565 snapshots over native USB and save them as PNGs."""

import argparse
from datetime import datetime
from pathlib import Path

import serial
from PIL import Image


def read_frame(port):
    # A short read timeout keeps Ctrl+C responsive while waiting for a press.
    port.timeout = 1
    line = bytearray()
    while True:
        chunk = port.readline()
        line.extend(chunk)
        if not chunk.endswith(b"\n"):
            continue

        if line.lower().startswith(b"error"):
            raise ValueError(line.decode(errors="replace").strip())
        if line.startswith(b"FRAME "):
            width, height, pitch, size = map(int, line.split()[1:])
            if width <= 0 or height <= 0 or pitch < width * 2 or size != pitch * height:
                raise ValueError(f"Invalid frame header {line.strip()!r}")
            print(f"Receiving {width} x {height} snapshot ({size} bytes).", flush=True)
            break
        line.clear()

    port.timeout = 20
    pixels = port.read(size)
    if len(pixels) != size:
        raise ValueError(f"Expected {size} image bytes, received {len(pixels)}.")

    if port.readline().strip() != b"END":
        raise ValueError("Missing end marker after the image.")

    return Image.frombytes("RGB", (width, height), pixels, "raw", "BGR;16", pitch, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Native USB serial port, for example COM11")
    args = parser.parse_args()
    output_dir = Path.cwd()

    try:
        with serial.Serial(baudrate=115200, timeout=1) as port:
            port.dtr = False
            port.rts = False
            port.port = args.port
            port.open()
            print("Press BOOT to take a picture. Press Ctrl+C to stop.", flush=True)
            while True:
                try:
                    image = read_frame(port)
                except ValueError as error:
                    print(f"{error} Press BOOT to try again.", flush=True)
                    continue

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                output = output_dir / f"snapshot_{timestamp}.png"
                image.save(output)
                print(f"Saved {output}", flush=True)
    except KeyboardInterrupt:
        print("\nStopped listening.")
    except OSError as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
