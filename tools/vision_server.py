"""Receive ESP32 JPEGs and serve the latest MobileNet SSD result."""

import argparse
import io
import json
import subprocess
import zlib
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, HTTPServer
from queue import Empty, Queue
from threading import Thread

from PIL import Image

from detect_snapshot import annotate, load_detector


MAX_JPEG_BYTES = 1024 * 1024
MAX_IMAGE_PIXELS = 4_000_000


class CaptureServer(HTTPServer):
    def __init__(self, address, matrix_port=None, jpeg_checker=None):
        self.model, self.weights = load_detector()
        self.latest_jpeg = None
        self.latest_source_jpeg = None
        self.matrix_port = matrix_port
        self.jpeg_checker = jpeg_checker
        self.matrix_messages = Queue()
        self.matrix_thread = None
        super().__init__(address, CaptureHandler)

        if self.matrix_port is not None:
            self.matrix_thread = Thread(target=self.scroll_matrix_results)
            self.matrix_thread.start()

    def scroll_matrix_results(self):
        messages = self.matrix_messages.get()
        index = 0

        while messages is not None:
            text = messages[index]
            try:
                self.matrix_port.write((text + "\n").encode("ascii", errors="replace"))
            except OSError as error:
                print(f"Could not send matrix result: {error}", flush=True)
            else:
                print(f"Sent to matrix: {text}", flush=True)

            # Count the columns in the MCU font, including spacing.
            columns = sum(1 if char == " " else 3 if char in "CFL1" else 4 for char in text)
            # Allow time for UART and the last letter.
            scroll_seconds = (columns + 2) * 0.120
            try:
                messages = self.matrix_messages.get(
                    timeout=scroll_seconds if len(messages) > 1 else None
                )
            except Empty:
                index = (index + 1) % len(messages)
            else:
                index = 0

    def server_close(self):
        if self.matrix_thread is not None:
            self.matrix_messages.put(None)
            self.matrix_thread.join()
            self.matrix_thread = None
        super().server_close()


class CaptureHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/capture":
            self.send_error(404, "Use POST /capture")
            return
        if self.headers.get_content_type() != "image/jpeg":
            self.send_error(415, "Expected Content-Type: image/jpeg")
            return
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self.send_error(411, "A valid Content-Length is required")
            return
        if length <= 0:
            self.send_error(400, "Empty JPEG")
            return
        if length > MAX_JPEG_BYTES:
            self.send_error(413, "JPEG exceeds 1 MiB")
            return

        self.connection.settimeout(15)
        try:
            payload = self.rfile.read(length)
        except TimeoutError:
            self.send_error(408, "JPEG upload timed out")
            return
        if len(payload) != length:
            self.send_error(400, "Incomplete JPEG upload")
            return

        checksum = self.headers.get("X-JPEG-CRC32")
        if checksum is not None:
            actual_checksum = f"{zlib.crc32(payload):08x}"
            if checksum.lower() != actual_checksum:
                self.log_error("JPEG CRC32 mismatch (ESP32 %s, Qbot %s)", checksum, actual_checksum)
                self.send_error(400, "JPEG checksum mismatch")
                return
            self.log_message("JPEG CRC32 matched (%s, %d bytes)", actual_checksum, length)

        try:
            with Image.open(io.BytesIO(payload), formats=["JPEG"]) as source:
                if source.width * source.height > MAX_IMAGE_PIXELS:
                    self.send_error(413, "JPEG exceeds 4 megapixels")
                    return
                if self.server.jpeg_checker:
                    check = subprocess.run(
                        [self.server.jpeg_checker, "-strict"],
                        input=payload,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        timeout=5,
                    )
                    if check.returncode != 0:
                        self.log_error("JPEG decoder rejected frame: %s",
                                       check.stderr.decode(errors="replace").strip())
                        self.send_error(422, "Damaged JPEG")
                        return
                image = source.convert("RGB")
                if self.headers.get("X-Camera-Orientation") == "rotate-180":
                    image = image.transpose(Image.Transpose.ROTATE_180)
        except Image.DecompressionBombError:
            self.send_error(413, "JPEG exceeds image size limit")
            return
        except (FileNotFoundError, subprocess.TimeoutExpired):
            self.send_error(503, "JPEG checker unavailable")
            return
        except OSError:
            self.send_error(422, "Could not decode JPEG")
            return

        detections = annotate(image, self.server.model, self.server.weights)
        with io.BytesIO() as output:
            image.save(output, format="JPEG", quality=95)
            self.server.latest_jpeg = output.getvalue()
            self.server.latest_source_jpeg = payload

        if self.server.matrix_port is not None:
            messages = [
                f"{label.upper()} {score:.0%}" for label, score in detections
            ] or ["NO OBJECT"]
            self.server.matrix_messages.put(messages)

        response = json.dumps({"detections": len(detections), "image": "/latest.jpg"})
        self.send_body("application/json", response.encode("utf-8"))

    def do_GET(self):
        if self.path == "/debug/source.jpg" and self.client_address[0] == "127.0.0.1":
            if self.server.latest_source_jpeg is None:
                self.send_error(404, "Upload a JPEG to /capture first")
                return
            self.send_body("image/jpeg", self.server.latest_source_jpeg)
            return

        if self.path != "/latest.jpg":
            self.send_error(404, "Use GET /latest.jpg")
            return
        if self.server.latest_jpeg is None:
            self.send_error(404, "Upload a JPEG to /capture first")
            return
        self.send_body("image/jpeg", self.server.latest_jpeg)

    def send_body(self, content_type, body):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--matrix-port", help="Serial device for the UNO Q matrix")
    parser.add_argument("--jpeg-checker", help="Path to djpeg for strict JPEG validation")
    args = parser.parse_args()

    with ExitStack() as stack:
        matrix_port = None
        if args.matrix_port:
            import serial

            matrix_port = stack.enter_context(
                serial.Serial(args.matrix_port, 115200, write_timeout=1, exclusive=True)
            )
        server = stack.enter_context(
            CaptureServer(("0.0.0.0", args.port), matrix_port, args.jpeg_checker)
        )
        print(f"Listening on port {args.port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
