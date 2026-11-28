"""Own the sorting station process and start it from a web page."""

import argparse
import json
import os
import signal
import subprocess
import sys
from collections import deque
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from time import monotonic, sleep

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "zephyr-object-sorting-station" / "tools"
STATION = ROOT / "zephyr-object-sorting-station" / "local" / "station.json"
PYTHON = ROOT / ".so101-venv" / "Scripts" / "python.exe"
PAGE = Path(__file__).with_suffix(".html")

DEFAULT_ADB = (Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15" / "packages"
               / "arduino" / "tools" / "adb" / "32.0.0" / "adb.exe")
QBOT_SERIAL = "2624126501"
CONVEYOR_SERIAL = "744557213"
QBOT_FORWARD = ["forward", "tcp:18000", "tcp:8000"]
DASHBOARD_PORT = 8767
# Kept in step with local/pickup-pose-smallerbox.json, which was taught with this ROI.
DEFAULT_ROI = (500, 170, 750, 420)


class Station:
    def __init__(self, adb=DEFAULT_ADB, roi=DEFAULT_ROI, port="COM7"):
        self.lock = Lock()
        self.process = None
        self.started_at = None
        self.exit_code = None
        self.stop_requested = False
        self.log = deque(maxlen=200)
        self.adb = Path(adb)
        self.roi = roi
        self.port = port

    def record(self, line):
        with self.lock:
            self.log.append(line.rstrip())
            return len(self.log)

    def running(self):
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def snapshot(self):
        with self.lock:
            running = self.process is not None and self.process.poll() is None
            return {
                "running": running,
                "pid": self.process.pid if self.process is not None else None,
                "uptime_s": round(monotonic() - self.started_at, 1)
                if running and self.started_at is not None else None,
                "exit_code": None if running else self.exit_code,
                "stopped_by_user": self.stop_requested,
                "dashboard_port": 8767,
                "log": list(self.log),
            }

    def start(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return False, "The sorter is already running."
        if not PYTHON.is_file():
            return False, f"Python is missing at {PYTHON}."
        if not STATION.is_file():
            return False, f"Station file is missing at {STATION}."
        if not self.adb.is_file():
            return False, f"adb is missing at {self.adb}. Pass --adb or install the SDK."
        forward = subprocess.run([str(self.adb), "-s", QBOT_SERIAL, *QBOT_FORWARD],
                                 capture_output=True, text=True, timeout=20)
        left, top, right, bottom = self.roi
        command = [
            str(PYTHON), "-B", "-u", str(TOOLS / "sort_station.py"), str(STATION),
            "--ultrasonic-only",
            "--roi", str(left), str(top), str(right), str(bottom),
            "--max-frame-age", "10",
            "--adb", str(self.adb), "--conveyor", CONVEYOR_SERIAL, "--port", self.port,
            "--stop-mm", "45", "--cycles", "0", "--execute",
            "--dashboard", str(DASHBOARD_PORT),
        ]
        with self.lock:
            self.log.clear()
            self.log.append(f"$ adb -s {QBOT_SERIAL} {' '.join(QBOT_FORWARD)}")
            if forward.returncode != 0:
                for line in (forward.stdout + forward.stderr).splitlines():
                    self.log.append(line)
                self.exit_code = forward.returncode
                return False, ("Could not forward the Qbot port. Check that the Qbot "
                               "UNO Q is connected over USB.")
            self.log.append("$ " + " ".join(command[1:]))
            options = {}
            if sys.platform == "win32":
                options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                options["start_new_session"] = True
            self.process = subprocess.Popen(
                command, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, **options)
            self.started_at = monotonic()
            self.exit_code = None
            self.stop_requested = False
        Thread(target=self.collect, args=(self.process,), daemon=True).start()
        return True, "Starting the sorter."

    def collect(self, process):
        for line in process.stdout:
            self.record(line)
        code = process.wait()
        with self.lock:
            self.exit_code = code
            stopped = self.stop_requested
        self.record("Stopped by request." if stopped else f"The sorter stopped (exit code {code}).")

    def stop(self):
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                return False, "The sorter is not running."
            self.stop_requested = True
        process.send_signal(signal.CTRL_BREAK_EVENT if sys.platform == "win32" else signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.record("The sorter did not stop in time. Ending it forcibly.")
            process.kill()
            process.wait(timeout=5)
        return True, "Stopped the sorter."


class LauncherHandler(BaseHTTPRequestHandler):
    timeout = 5

    def do_GET(self):
        if self.path == "/":
            self.send_content("text/html; charset=utf-8", PAGE.read_bytes())
        elif self.path == "/status.json":
            self.send_json(self.server.station.snapshot())
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path not in ("/start", "/stop"):
            self.send_error(404)
            return
        station = self.server.station
        if self.path == "/start":
            ok, message = station.start()
        else:
            ok, message = station.stop()
        self.send_json({"ok": ok, "message": message, **station.snapshot()},
                       status=200 if ok else 409)

    def send_json(self, payload, status=200):
        self.send_content("application/json", json.dumps(payload).encode(), status)

    def send_content(self, content_type, body, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        pass


@contextmanager
def launcher_server(port, station):
    with ThreadingHTTPServer(("127.0.0.1", port), LauncherHandler) as server:
        server.station = station
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        print(f"Station launcher: http://127.0.0.1:{server.server_port}/", flush=True)
        try:
            yield server.server_port
        finally:
            server.shutdown()
            worker.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--arm-port", default="COM7", help="Serial port of the SO-101 follower")
    parser.add_argument("--adb", default=str(DEFAULT_ADB), help="Path to the adb executable")
    parser.add_argument("--roi", type=int, nargs=4, default=DEFAULT_ROI,
                        metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
                        help="Pickup area in camera pixels. Must match the taught pickup pose.")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("Use a port between 1024 and 65535")
    if not (0 <= args.roi[0] < args.roi[2] and 0 <= args.roi[1] < args.roi[3]):
        parser.error("Use an ROI with left < right and top < bottom")
    station = Station(adb=args.adb, roi=tuple(args.roi), port=args.arm_port)
    try:
        with launcher_server(args.port, station):
            while True:
                sleep(.25)
    except KeyboardInterrupt:
        station.stop()


if __name__ == "__main__":
    main()
