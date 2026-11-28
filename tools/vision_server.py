"""Receive camera JPEGs and serve the latest object detection result."""

import argparse
import io
import json
import subprocess
import zlib
from contextlib import ExitStack
from dataclasses import dataclass
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Empty, Queue
from threading import Lock, Thread
from time import monotonic
from uuid import uuid4

from PIL import Image

from detect_snapshot import draw_detections, load_detector, predict


MAX_JPEG_BYTES = 1024 * 1024
MAX_IMAGE_PIXELS = 4_000_000
PHONE_PAGE = Path(__file__).with_name("phone_capture.html")


def make_detector(yolo_weights, yolo_confidence, yolo_crop):
    if yolo_weights is None:
        model, weights = load_detector()
        return partial(predict, model=model, weights=weights)

    from ultralytics import YOLO

    model = YOLO(str(yolo_weights), task="detect")

    def detect(image):
        left = top = 0
        if yolo_crop is not None:
            left, top, right, bottom = yolo_crop
            if right > image.width or bottom > image.height:
                return []
            image = image.crop((left, top, right, bottom))
        result = model.predict(
            image, imgsz=640, conf=yolo_confidence, agnostic_nms=True, device="cpu", verbose=False
        )[0]
        return [
            (result.names[class_id], confidence,
             [box[0] + left, box[1] + top, box[2] + left, box[3] + top])
            for class_id, confidence, box in zip(
                result.boxes.cls.int().tolist(),
                result.boxes.conf.tolist(),
                result.boxes.xyxy.tolist(),
            )
        ]

    return detect


@dataclass(frozen=True)
class Capture:
    received_at: float
    result: dict
    jpeg: bytes
    source: bytes
    orientation: str | None


class CaptureServer(ThreadingHTTPServer):
    def __init__(self, address, matrix_port=None, jpeg_checker=None, yolo_weights=None,
                 yolo_confidence=0.5, yolo_crop=None):
        self.detect = make_detector(yolo_weights, yolo_confidence, yolo_crop)
        self.latest_capture = None
        self.inference_lock = Lock()
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
    timeout = 10

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
        received_at = monotonic()
        orientation = self.headers.get("X-Camera-Orientation")

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
                if orientation == "rotate-180":
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

        with self.server.inference_lock:
            predictions = self.server.detect(image)
            detections = draw_detections(image, predictions)
            with io.BytesIO() as output:
                image.save(output, format="JPEG", quality=95)
                capture = Capture(received_at, {
                    "capture_id": uuid4().hex,
                    "detections": len(detections),
                    "image": "/latest.jpg",
                    "image_size_px": image.size,
                    "objects": [
                        {"label": label, "confidence": confidence, "box_xyxy": box}
                        for label, confidence, box in predictions
                    ],
                }, output.getvalue(), payload, orientation if orientation == "rotate-180" else None)

            latest = self.server.latest_capture
            if latest is None or capture.received_at >= latest.received_at:
                self.server.latest_capture = capture
                if self.server.matrix_port is not None:
                    self.server.matrix_messages.put([
                        f"{label.upper()} {score:.0%}" for label, score in detections
                    ] or ["NO OBJECT"])

        self.send_capture_result(capture)

    def do_GET(self):
        if self.path == "/phone":
            try:
                page = PHONE_PAGE.read_bytes()
            except FileNotFoundError:
                self.send_error(404, "Phone capture page is unavailable")
                return
            self.send_body("text/html; charset=utf-8", page)
            return

        source = self.path == "/debug/source.jpg" and self.client_address[0] == "127.0.0.1"
        if not source and self.path not in ("/latest.json", "/latest.jpg"):
            self.send_error(404, "Use GET /latest.jpg")
            return
        capture = self.server.latest_capture
        if capture is None:
            self.send_error(404, "Upload a JPEG to /capture first")
        elif self.path == "/latest.json":
            self.send_capture_result(capture)
        else:
            headers = {"X-Capture-ID": capture.result["capture_id"]}
            if source and capture.orientation:
                headers["X-Camera-Orientation"] = capture.orientation
            self.send_body("image/jpeg", capture.source if source else capture.jpeg, headers)

    def send_capture_result(self, capture):
        result = dict(capture.result, age_seconds=monotonic() - capture.received_at)
        self.send_body("application/json", json.dumps(result).encode("utf-8"))

    def send_body(self, content_type, body, headers=None):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--matrix-port", help="Serial device for the UNO Q matrix")
    parser.add_argument("--jpeg-checker", help="Path to djpeg for strict JPEG validation")
    parser.add_argument("--yolo-weights", type=Path, help="Local YOLO detection checkpoint")
    parser.add_argument("--yolo-confidence", type=float, default=0.5)
    parser.add_argument("--yolo-crop", type=int, nargs=4, metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
                        help="Run YOLO within this rectangle while reporting full-image coordinates")
    args = parser.parse_args()
    if args.yolo_weights is not None and not args.yolo_weights.is_file():
        parser.error("--yolo-weights must point to an existing checkpoint")
    if not 0 < args.yolo_confidence <= 1:
        parser.error("--yolo-confidence must be greater than 0 and at most 1")
    if args.yolo_crop is not None:
        left, top, right, bottom = args.yolo_crop
        if args.yolo_weights is None or not (0 <= left < right and 0 <= top < bottom):
            parser.error("--yolo-crop requires YOLO weights and a valid rectangle")

    with ExitStack() as stack:
        matrix_port = None
        if args.matrix_port:
            import serial

            matrix_port = stack.enter_context(
                serial.Serial(args.matrix_port, 115200, write_timeout=1, exclusive=True)
            )
        server = stack.enter_context(
            CaptureServer(("0.0.0.0", args.port), matrix_port, args.jpeg_checker,
                          args.yolo_weights, args.yolo_confidence, args.yolo_crop)
        )
        detector = args.yolo_weights.name if args.yolo_weights else "MobileNet SSD"
        print(f"Listening on port {args.port} with {detector}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
