"""Serve live sorting telemetry on the Windows PC's loopback interface."""

import argparse
from collections import deque
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from threading import Lock, Thread
from time import monotonic, sleep
from urllib.parse import parse_qs, urlsplit
from urllib.request import urlopen

import cv2
import numpy as np

from pickup_pose import annotated_jpeg


PAGE = Path(__file__).with_suffix(".html")
PULSES_PER_REV = 20


class Dashboard:
    def __init__(self, enabled=False, belt_mm_per_rev=None):
        self.enabled = enabled
        self.belt_mm_per_rev = belt_mm_per_rev
        self.lock = Lock()
        self.started = monotonic()
        self.samples = deque()
        self.history = deque(maxlen=120)
        self.events = deque(maxlen=10)
        self.frame = None
        self.frame_id = 0
        self.belt = None
        self.belt_received = None
        self.details = {"phase": "ready", "title": "Preparing station", "detail": "",
                        "object": None, "destination": None, "correction": None,
                        "stop_mm": None, "jog": None}

    def stage(self, phase, title, detail="", **details):
        if not self.enabled:
            return
        with self.lock:
            self.details.update(phase=phase, title=title, detail=detail, **details)
            self.events.append({"at": monotonic() - self.started, "title": title})

    def next_item(self):
        if not self.enabled:
            return
        with self.lock:
            self.frame = None
            self.frame_id += 1
            self.details.update(object=None, destination=None, correction=None, stop_mm=None, jog=None)

    def belt_sample(self, status):
        if not self.enabled:
            return
        now = monotonic()
        with self.lock:
            if self.samples:
                previous, pulses = self.samples[-1]
                delta = (status.pulses - pulses) % (1 << 32)
                if delta > 300 or now - previous > 1.5:
                    self.samples.clear()
            self.samples.append((now, status.pulses))
            while len(self.samples) > 1 and now - self.samples[0][0] > .8:
                self.samples.popleft()
            started, pulses = self.samples[0]
            rpm = None
            if now - started >= .25:
                rpm = 60 * ((status.pulses - pulses) % (1 << 32)) / (PULSES_PER_REV * (now - started))
                self.history.append((round(now - self.started, 2), round(rpm, 1)))
            self.belt_received = now
            self.belt = {"rpm": rpm, "pwm": status.duty, "running": status.run_allowed,
                         "distance_mm": status.distance_mm if status.distance_mm > 0 else None,
                         "mm_per_s": rpm * self.belt_mm_per_rev / 60
                         if rpm is not None and self.belt_mm_per_rev is not None else None}

    def detection(self, image, objects, roi, selected=None):
        """Show the detected boxes without the taught pickup overlay."""
        if not self.enabled or not objects:
            return
        if roi:
            left, top, right, bottom = roi
            left, top = max(0, left - 42), max(0, top - 42)
            right, bottom = min(image.shape[1], right + 42), min(image.shape[0], bottom + 42)
        else:
            left, top = 0, 0
            right, bottom = image.shape[1], image.shape[0]
        view = image[top:bottom, left:right].copy()
        for obj in objects:
            x1, y1, x2, y2 = np.rint(obj["box_xyxy"]).astype(int)
            start = (x1 - left, y1 - top)
            end = (x2 - left, y2 - top)
            color = (213, 235, 107) if obj is selected else (75, 189, 255)
            cv2.rectangle(view, start, end, color, 2)
            caption = f"{obj['label']} {obj['confidence']:.0%}"
            cv2.putText(view, caption, (max(0, start[0]), max(12, start[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
        encoded, jpeg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 90])
        with self.lock:
            if encoded:
                self.frame = jpeg.tobytes()
                self.frame_id += 1
            if selected is not None:
                self.details.update(object={"label": selected["label"],
                                            "confidence": selected["confidence"]})

    def classified(self, image, obj, pose, target, roi, along, across, angle):
        if not self.enabled:
            return
        left, top, right, bottom = roi
        left, top = max(0, left - 42), max(0, top - 42)
        right, bottom = min(image.shape[1], right + 42), min(image.shape[0], bottom + 42)
        view = image[top:bottom, left:right].copy()
        origin = np.array([left, top])
        x1, y1, x2, y2 = np.rint(obj["box_xyxy"]).astype(int)
        start = (x1 - left, y1 - top)
        cv2.rectangle(view, start, (x2 - left, y2 - top), (213, 235, 107), 2)
        measured = tuple(np.rint(pose.center - origin).astype(int))
        taught = tuple(np.rint(np.asarray(target["center"]) - origin).astype(int))
        cv2.line(view, measured, taught, (75, 189, 255), 2, cv2.LINE_AA)
        if target["angle_period"]:
            for center, heading, color in ((measured, pose.angle, (213, 235, 107)),
                                           (taught, target["angle"], (75, 189, 255))):
                radians = np.deg2rad(heading)
                axis = 13 * np.array([np.cos(radians), np.sin(radians)])
                cv2.line(view, tuple(np.rint(center - axis).astype(int)),
                         tuple(np.rint(center + axis).astype(int)), color, 2, cv2.LINE_AA)
        cv2.circle(view, measured, 4, (213, 235, 107), -1, cv2.LINE_AA)
        cv2.drawMarker(view, taught, (75, 189, 255), cv2.MARKER_CROSS, 15, 2, cv2.LINE_AA)
        encoded, jpeg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 90])
        with self.lock:
            if encoded:
                self.frame = jpeg.tobytes()
                self.frame_id += 1
            self.details.update(object={"label": obj["label"], "confidence": obj["confidence"]},
                                correction={"along_px": along, "across_px": across, "angle_deg": -angle})

    def snapshot(self):
        with self.lock:
            now = monotonic()
            belt = dict(self.belt, age_s=now - self.belt_received) if self.belt is not None else None
            return {**self.details, "belt": belt, "history": list(self.history), "events": list(self.events),
                    "frame_id": self.frame_id if self.frame is not None else None,
                    "elapsed": now - self.started}

    def read_frame(self, version):
        with self.lock:
            return self.frame if version is None or version == str(self.frame_id) else None

    def store_frame(self, jpeg):
        with self.lock:
            self.frame = jpeg
            self.frame_id += 1


class DashboardHandler(BaseHTTPRequestHandler):
    timeout = 3

    def do_GET(self):
        url = urlsplit(self.path)
        path = url.path
        if path == "/":
            self.send_content("text/html; charset=utf-8", self.server.page)
        elif path == "/state.json":
            self.send_content("application/json", json.dumps(self.server.dashboard.snapshot()).encode())
        elif path == "/live.jpg":
            after = parse_qs(url.query).get("after", [None])[0]
            self.send_live_frame(after)
        elif path == "/frame.jpg":
            version = parse_qs(url.query).get("v", [None])[0]
            frame = self.server.dashboard.read_frame(version)
            if frame is None:
                self.send_error(404, "No classified frame yet")
            else:
                self.send_content("image/jpeg", frame)
        else:
            self.send_error(404)

    def send_live_frame(self, after):
        qbot = self.server.qbot
        if qbot is None:
            self.send_error(503, "Camera unavailable")
            return
        try:
            with urlopen(qbot + "/latest.json", timeout=1) as response:
                frame = json.load(response)
            capture_id = frame["capture_id"]
            if not capture_id or not 0 <= frame["age_seconds"] <= 10:
                raise ValueError("No recent capture")
        except (OSError, ValueError, KeyError, TypeError):
            self.send_error(503, "Waiting for a fresh phone frame")
            return
        if capture_id == after:
            self.send_content("image/jpeg", b"", status=204)
            return
        jpeg = annotated_jpeg(qbot, capture_id)
        if jpeg is None:
            self.send_error(409, "Camera frame changed during refresh")
            return
        self.send_content("image/jpeg", jpeg, capture_id=capture_id)

    def send_content(self, content_type, body, status=200, capture_id=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if status != 204:
            self.send_header("Content-Length", str(len(body)))
        if capture_id is not None:
            self.send_header("X-Capture-ID", capture_id)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        pass


@contextmanager
def dashboard_server(dashboard, port, qbot=None):
    if port is None:
        yield
        return
    page = PAGE.read_bytes()
    with ThreadingHTTPServer(("127.0.0.1", port), DashboardHandler) as server:
        server.dashboard = dashboard
        server.page = page
        server.qbot = qbot.rstrip("/") if qbot else None
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        print(f"Station dashboard: http://127.0.0.1:{server.server_port}/", flush=True)
        try:
            yield server.server_port
        finally:
            server.shutdown()
            worker.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preview the station dashboard without connecting to hardware.")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--qbot", help="Qbot URL for a live camera preview")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("Use a port between 1024 and 65535")
    dashboard = Dashboard(enabled=True)
    dashboard.stage("ready", "Dashboard preview", "Camera and belt readings appear during a live sorting run.")
    try:
        with dashboard_server(dashboard, args.port, args.qbot):
            while True:
                sleep(.25)
    except KeyboardInterrupt:
        pass
