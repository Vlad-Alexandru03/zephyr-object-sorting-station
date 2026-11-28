"""Calibrate the camera, record SO-101 references, and arm a supported pose."""

import argparse
import hashlib
import json
from pathlib import Path
from time import sleep

import numpy as np

from preview_pickup import arm_target
from so101_pick import JOINTS, arm_bus, grip, read_arm, tool_pose


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def camera_calibration(image_size, pixels, side):
    pixels = np.asarray(pixels, dtype=float).reshape(4, 2)
    if (not np.isfinite(pixels).all() or min(image_size) <= 0 or not (np.isfinite(side) and side > 0)
            or (pixels < 0).any() or (pixels >= image_size).any()):
        raise ValueError("Enter four finite pixel pairs inside the frame and a positive square size.")
    edges = np.roll(pixels, -1, axis=0) - pixels
    cross = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if not (np.all(cross > 0) or np.all(cross < 0)):
        raise ValueError("Enter the four corners in order around the square.")
    belt = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * side / 2
    center = pixels.mean(axis=0)
    rows, rhs = [], []
    for (u, v), (x, y) in zip(pixels - center, belt):
        rows.extend(((u, v, 1, 0, 0, 0, -u*x, -v*x), (0, 0, 0, u, v, 1, -u*y, -v*y)))
        rhs.extend((x, y))
    homography = np.r_[np.linalg.solve(rows, rhs), 1].reshape(3, 3)
    homography = homography @ [[1, 0, -center[0]], [0, 1, -center[1]], [0, 0, 1]]
    origin = np.linalg.solve(homography, [0, 0, 1])
    return {
        "image_size_px": image_size,
        "square_side_mm": side,
        "origin_px": (origin[:2] / origin[2]).tolist(),
        "pixel_to_belt_mm": homography.tolist(),
        "points": [
            {"id": i, "pixel_uv": pixel.tolist(), "belt_xy_mm": point.tolist()}
            for i, (pixel, point) in enumerate(zip(pixels, belt), 1)
        ],
    }


def read_steady_pose(port, robot_id):
    with arm_bus(port, robot_id) as (bus, _):
        samples = []
        for _ in range(6):
            samples.append(read_arm(bus))
            sleep(.12)
        opening, _ = grip(bus)
    positions = np.array([tool_pose(sample)[0] for sample in samples])
    if np.linalg.norm(positions - positions.mean(axis=0), axis=1).max() > 1:
        raise ValueError("The tool moved during the sample. Hold it steady and try again.")
    angles = np.median(samples, axis=0)
    return angles, tool_pose(angles)[0], opening


def save_reference(args):
    raw = args.calibration.read_bytes()
    calibration = json.loads(raw)
    digest = hashlib.sha256(raw).hexdigest()
    data = {
        "calibration": args.calibration.name,
        "calibration_sha256": digest,
        "robot_id": args.robot_id,
        "samples": {},
    }
    if args.output.exists():
        data = json.loads(args.output.read_text(encoding="utf-8"))
        if data["calibration_sha256"] != digest or data["robot_id"] != args.robot_id:
            raise ValueError("Use a new registration file for this calibration or robot.")
    _, position, _ = read_steady_pose(args.port, args.robot_id)
    data["samples"][str(args.point)] = position.tolist()
    print(f"Point {args.point} arm XYZ (mm): {position.round(1)}")
    if len(data["samples"]) == 4:
        belt = [p["belt_xy_mm"] for p in calibration["points"]]
        arm = [data["samples"][str(i)] for i in range(1, 5)]
        data["belt_xy_mm"] = belt
        data["arm_xy_mm"] = [p[:2] for p in arm]
        estimate, _ = arm_target(
            belt[3], {"belt_xy_mm": belt[:3], "arm_xy_mm": data["arm_xy_mm"][:3]}
        )
        data["independent_check_error_mm"] = float(np.linalg.norm(estimate - arm[3][:2]))
        data["height_spread_mm"] = float(np.ptp(np.asarray(arm)[:, 2]))
        _, error = arm_target((0, 0), data)
        print(f"Fit error {error:.1f} mm, point 4 check {data['independent_check_error_mm']:.1f} mm, "
              f"height spread {data['height_spread_mm']:.1f} mm")
    write_json(args.output, data)


def save_waypoint(args):
    data = {"robot_id": args.robot_id, "waypoints": {}}
    if args.output.exists():
        data = json.loads(args.output.read_text(encoding="utf-8"))
        if data["robot_id"] != args.robot_id:
            raise ValueError("This waypoint file belongs to a different robot.")
    angles, position, opening = read_steady_pose(args.port, args.robot_id)
    pose = {joint + ".pos": value for joint, value in zip(JOINTS, angles.tolist())}
    pose["gripper.pos"] = opening
    data["waypoints"][args.name] = pose
    if args.name == "grasp":
        data["object_label"] = args.label
    write_json(args.output, data)
    print(f"Saved {args.name}, arm XYZ (mm) {position.round(1)}, gripper {opening:.1f}% open.")


def release_hold(args):
    if input("Switch off belt motor power and place a stable prop under the arm. "
             "Keep hands clear and type RELEASE to disable servo torque: ") != "RELEASE":
        return
    with arm_bus(args.port, args.robot_id) as (bus, _):
        bus.sync_write("Torque_Enable", 0, normalize=False, num_retry=3)
        torque = set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values())
        if torque != {0}:
            raise RuntimeError("Keep the support in place. Some servos did not confirm torque off.")
        print("All six servos have torque off. Keep the arm supported while positioning it.")


def enable_hold(args):
    if input("Support the arm, switch off belt motor power, and keep hands clear. "
             "Type HOLD to engage the servos at their current positions: ") != "HOLD":
        return
    with arm_bus(args.port, args.robot_id) as (bus, _):
        torque = set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values())
        if torque == {1}:
            print("The servos are already holding.")
            return
        if torque != {0}:
            raise RuntimeError("Servo torque states disagree. Check the arm before arming it.")
        positions = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        sleep(.2)
        current = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        if max(abs(current[j] - positions[j]) for j in current) > 5:
            raise RuntimeError("The arm moved. Wait until the support holds it steady.")
        bus.sync_write("Goal_Position", current, normalize=False, num_retry=3)
        goals = bus.sync_read("Goal_Position", normalize=False, num_retry=3)
        if goals != current:
            raise RuntimeError("Could not verify the holding targets. Torque remains off.")
        bus.sync_write("Torque_Enable", 1, normalize=False, num_retry=3)
        if set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values()) != {1}:
            raise RuntimeError("Keep the support in place. Some servos did not confirm torque.")
        print("Servos engaged. Check that the arm is holding before removing the support.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    camera = commands.add_parser("camera")
    camera.add_argument("output", type=Path)
    camera.add_argument("--image-size", nargs=2, type=int, required=True)
    camera.add_argument("--corners", nargs=8, type=float, required=True,
                        help="Four u v pairs in clockwise order, starting from point 1")
    camera.add_argument("--side-mm", type=float, default=30)
    reference = commands.add_parser("point")
    reference.add_argument("calibration", type=Path)
    reference.add_argument("output", type=Path)
    reference.add_argument("point", type=int, choices=range(1, 5))
    waypoint = commands.add_parser("teach")
    waypoint.add_argument("output", type=Path)
    waypoint.add_argument("name", choices=("observation", "above_pickup", "grasp", "above_bin"))
    waypoint.add_argument("--label", help="Object held at the taught grasp pose")
    hold = commands.add_parser("hold", help="Engage servos at a supported, stationary pose")
    release = commands.add_parser("release", help="Disable servo torque on a supported arm")
    for command in (reference, waypoint, hold, release):
        command.add_argument("--robot-id", default="sorting_arm")
        command.add_argument("--port", default="COM7")
    args = parser.parse_args()
    try:
        if args.command == "camera":
            data = camera_calibration(args.image_size, args.corners, args.side_mm)
            write_json(args.output, data)
            print(f"Saved {args.output}; center at pixel {np.round(data['origin_px'], 1)}.")
        elif args.command == "point":
            save_reference(args)
        elif args.command == "teach":
            if args.name == "grasp" and not args.label:
                parser.error("Teaching a grasp requires --label.")
            save_waypoint(args)
        elif args.command == "hold":
            enable_hold(args)
        else:
            release_hold(args)
    except (OSError, KeyError, ValueError, RuntimeError, EOFError, np.linalg.LinAlgError) as error:
        parser.exit(1, f"Calibration stopped: {error}\n")


if __name__ == "__main__":
    main()
