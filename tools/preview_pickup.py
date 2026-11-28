"""Report the latest detected object position in the calibrated belt plane."""

import argparse
import json
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


def latest_frame(qbot):
    with urlopen(qbot.rstrip("/") + "/latest.json", timeout=10) as response:
        return json.load(response)


def belt_objects(frame, calibration, max_age, margin_mm=0):
    if not 0 <= frame["age_seconds"] <= max_age:
        raise ValueError("The last frame is stale. Start the live camera and try again.")
    if frame["image_size_px"] != calibration["image_size_px"]:
        raise ValueError("The frame size differs from the calibration photo.")

    matrix = calibration["pixel_to_belt_mm"]
    half_side = calibration["square_side_mm"] / 2 + margin_mm
    candidates = []
    for obj in frame["objects"]:
        x1, y1, x2, y2 = obj["box_xyxy"]
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        x, y, w = [a*u + b*v + c for a, b, c in matrix]
        if abs(w) < 1e-9:
            continue
        x, y = x / w, y / w
        if abs(x) <= half_side and abs(y) <= half_side:
            candidates.append((obj, (u, v), (x, y)))
    return candidates


def pickup_target(frame, calibration, label, max_age, margin_mm=0):
    candidates = [
        item for item in belt_objects(frame, calibration, max_age, margin_mm)
        if item[0]["label"].casefold() == label.casefold()
    ]
    if len(candidates) != 1:
        raise ValueError(
            f"Expected one {label} centered in the calibrated square, found {len(candidates)}."
        )
    return candidates[0]


def arm_target(belt_target, registration):
    import numpy as np

    belt = np.asarray(registration["belt_xy_mm"], dtype=float)
    arm = np.asarray(registration["arm_xy_mm"], dtype=float)
    if belt.ndim != 2 or belt.shape[1] != 2 or len(belt) < 3 or arm.shape != belt.shape:
        raise ValueError("Registration needs at least three matched belt and arm XY points.")
    if not np.isfinite(belt).all() or not np.isfinite(arm).all():
        raise ValueError("Registration points must be finite.")

    belt_center = belt.mean(axis=0)
    arm_center = arm.mean(axis=0)
    belt_offsets = belt - belt_center
    arm_offsets = arm - arm_center
    if np.linalg.matrix_rank(belt_offsets) < 2 or np.linalg.matrix_rank(arm_offsets) < 2:
        raise ValueError("Registration points must span both axes.")

    # Either axis convention is valid, so the fit may include a reflection.
    u, _, vt = np.linalg.svd(belt_offsets.T @ arm_offsets)
    transform = u @ vt
    fit = belt_offsets @ transform
    error = np.linalg.norm(fit - arm_offsets, axis=1).max()
    return (np.asarray(belt_target) - belt_center) @ transform + arm_center, error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("--qbot", default="http://192.168.0.121:8000")
    parser.add_argument("--label", default="Tape roll")
    parser.add_argument("--max-age", type=float, default=8)
    parser.add_argument(
        "--registration", type=Path,
        help="JSON with matched belt_xy_mm and arm_xy_mm point arrays",
    )
    args = parser.parse_args()

    try:
        calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
        frame = latest_frame(args.qbot)
        obj, (u, v), (x, y) = pickup_target(frame, calibration, args.label, args.max_age)
        if args.registration:
            registration = json.loads(args.registration.read_text(encoding="utf-8"))
            (arm_x, arm_y), registration_error = arm_target((x, y), registration)
    except HTTPError as error:
        if error.code == 404:
            parser.exit(1, "No detection frame available. Start live capture on Qbot.\n")
        parser.exit(1, f"Qbot returned HTTP {error.code}.\n")
    except (URLError, OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Pickup preview unavailable: {error}\n")

    print(f"{obj['label']} ({obj['confidence']:.0%}) at pixel ({u:.1f}, {v:.1f})")
    print(f"Estimated belt offset: X {x:+.1f} mm, Y {y:+.1f} mm")
    if args.registration:
        print(f"Estimated arm-base target: X {arm_x:+.1f} mm, Y {arm_y:+.1f} mm")
        print(f"Maximum registration residual: {registration_error:.1f} mm")


if __name__ == "__main__":
    main()
