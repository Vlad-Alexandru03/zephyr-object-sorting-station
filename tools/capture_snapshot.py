"""Receive BOOT-triggered JPEG snapshots over native USB."""

import argparse
import io
from datetime import datetime
from pathlib import Path

import serial
from PIL import Image


def read_frame(port):
    port.timeout = 1
    line = bytearray()

    while True:
        chunk = port.readline()
        line.extend(chunk)

        if not chunk.endswith(b"\n"):
            continue

        if line.lower().startswith(b"error"):
            raise ValueError(line.decode(errors="replace").strip())

        if line.startswith(b"JPEG "):
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(f"Invalid JPEG header {bytes(line)!r}")

            width, height, size = map(int, fields[1:])
            if (
                (width, height) not in {(320, 240), (2048, 1536)}
                or not 4 <= size <= 1024 * 1024
            ):
                raise ValueError(f"Invalid JPEG header {bytes(line)!r}")

            print(
                f"Receiving {width} x {height} JPEG ({size} bytes).",
                flush=True,
            )
            break

        line.clear()

    port.timeout = 60
    payload = port.read(size)

    if len(payload) != size:
        raise ValueError(
            f"Expected {size} JPEG bytes, received {len(payload)}."
        )

    if port.readline().strip() != b"END":
        raise ValueError("Missing end marker after the image.")

    jpeg = payload.rstrip(b"\x00")
    if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
        diagnostic = Path("jpeg_failed.bin").resolve()
        diagnostic.write_bytes(payload)
        raise ValueError(
            f"Invalid JPEG boundaries. First bytes {payload[:16].hex()}, "
            f"last bytes {payload[-16:].hex()}, "
            f"SOI offset {payload.find(b'\xff\xd8')}, "
            f"EOI offset {payload.find(b'\xff\xd9')}. Saved {diagnostic}"
        )

    try:
        with Image.open(io.BytesIO(jpeg)) as image:
            if image.format != "JPEG" or image.size != (width, height):
                raise ValueError("JPEG dimensions or format differ from the header.")
            image.load()
    except OSError as error:
        raise ValueError(f"JPEG decoding failed: {error}") from error

    if len(jpeg) != len(payload):
        print(f"Removed {len(payload) - len(jpeg)} trailing padding bytes.", flush=True)

    return jpeg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Native USB serial port, for example COM11")
    args = parser.parse_args()
    output_dir = Path.cwd()

    try:
        with serial.Serial(baudrate=115200, timeout=1) as port:
            port.dtr = True
            port.rts = False
            port.port = args.port
            port.open()

            print("Press BOOT to take a picture. Press Ctrl+C to stop.", flush=True)

            while True:
                try:
                    payload = read_frame(port)
                except ValueError as error:
                    print(f"{error} Press BOOT to try again.", flush=True)
                    continue

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                output = output_dir / f"snapshot_{timestamp}.jpg"
                with Image.open(io.BytesIO(payload)) as image:
                    rotated = image.transpose(Image.Transpose.ROTATE_180)
                    rotated.save(output, format="JPEG", quality=95, subsampling=0)
                print(f"Saved {output}", flush=True)

    except KeyboardInterrupt:
        print("\nStopped listening.")
    except OSError as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
