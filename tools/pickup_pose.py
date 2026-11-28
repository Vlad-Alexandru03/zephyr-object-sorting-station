"""Measure a box's center and orientation against a taught phone-camera view."""

from dataclasses import dataclass
import json
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import numpy as np


@dataclass(frozen=True)
class BoxPose:
    center: np.ndarray
    angle: float
    aspect: float


def source_image(qbot, frame):
    try:
        with urlopen(qbot.rstrip("/") + "/debug/source.jpg", timeout=2) as response:
            capture = response.headers.get("X-Capture-ID")
            if capture is None:
                raise RuntimeError("Qbot needs the updated vision server for matching source images.")
            if capture != frame["capture_id"]:
                return None
            orientation = response.headers.get("X-Camera-Orientation")
            jpeg = np.frombuffer(response.read(), dtype=np.uint8)
    except (URLError, OSError):
        return None
    image = cv2.imdecode(jpeg, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None:
        return None
    if orientation == "rotate-180":
        image = cv2.rotate(image, cv2.ROTATE_180)
    if list(image.shape[1::-1]) != frame["image_size_px"]:
        raise ValueError("Qbot source image size differs from its detection frame.")
    return image


def annotated_jpeg(qbot, capture_id):
    """Fetch the labeled JPEG for a specific capture."""
    try:
        with urlopen(qbot.rstrip("/") + "/latest.jpg", timeout=2) as response:
            if response.headers.get("X-Capture-ID") == capture_id:
                return response.read()
    except (URLError, OSError):
        return None
    return None


def box_pose(image, background, box):
    if image.shape != background.shape:
        raise ValueError("Camera size changed. Teach the pickup again.")
    x1, y1, x2, y2 = map(float, box)
    width, height = x2 - x1, y2 - y1
    if not np.isfinite([x1, y1, x2, y2]).all() or min(width, height) < 15:
        return None
    left = max(0, int(x1 - width * .12))
    top = max(0, int(y1 - height * .12))
    right = min(image.shape[1], int(np.ceil(x2 + width * .12)))
    bottom = min(image.shape[0], int(np.ceil(y2 + height * .12)))
    if left >= right or top >= bottom:
        return None
    frame = cv2.GaussianBlur(image[top:bottom, left:right], (5, 5), 0)
    empty = cv2.GaussianBlur(background[top:bottom, left:right], (5, 5), 0)
    difference = np.max(cv2.absdiff(frame, empty), axis=2)
    mask = np.uint8(difference >= 28) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    center, (w, h), angle = cv2.minAreaRect(contour)
    if (area < .18 * width * height or min(w, h) < 12
            or area < .55 * w * h):
        return None
    center = np.asarray(center) + (left, top)
    if np.linalg.norm(center - ((x1 + x2) / 2, (y1 + y2) / 2)) > .3 * max(width, height):
        return None
    return BoxPose(center, (angle + (90 if h > w else 0)) % 180, max(w, h) / min(w, h))


def placement_error(target, pose):
    offset = np.asarray(target["center"]) - pose.center
    forward = np.asarray(target["forward"])
    period = target["angle_period"]
    if (period not in (0, 90, 180) or offset.shape != (2,) or forward.shape != (2,)
            or not np.isfinite([*offset, *forward, target["angle"], pose.angle]).all()
            or not np.isclose(np.linalg.norm(forward), 1)):
        raise ValueError("The taught pickup direction or angle is invalid.")
    angle = (pose.angle - target["angle"] + period / 2) % period - period / 2 if period else 0.0
    along = float(offset @ forward)
    across = float(offset @ np.array([-forward[1], forward[0]]))
    return along, across, angle


def background_path(path):
    return path.with_suffix(".empty.png")


def load_pose_config(path, ready=False):
    data = json.loads(path.read_text(encoding="utf-8"))
    background = cv2.imread(str(background_path(path)), cv2.IMREAD_COLOR)
    if background is None or list(background.shape[1::-1]) != data["image_size_px"]:
        raise ValueError("Teach an empty pickup image at the current camera position.")
    if ready and not any("forward" in target for target in data["targets"].values()):
        raise ValueError("Teach a pickup target and an upstream position before running.")
    return data, background


def save_pose_config(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
