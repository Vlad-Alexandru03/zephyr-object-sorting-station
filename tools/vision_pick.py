"""Preview or supervise one camera-guided SO-101 pickup."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from time import monotonic, sleep
from urllib.error import URLError

import numpy as np

from preview_pickup import arm_target, latest_frame, pickup_target
from so101_pick import (
    add_motion_arguments, arm_bus, grip, hold_position, joint_values, move_arm,
    pick_and_place, read_arm, require_hover, settle_tool, tool_pose,
    validate_motion_arguments, vertical_path,
)


def hover_path(start, target_xy, max_offset_mm=25):
    origin, attitude = tool_pose(start)
    offset = np.asarray(target_xy) - origin[:2]
    distance = np.linalg.norm(offset)
    if not np.isfinite(distance) or distance > max_offset_mm:
        raise ValueError(f"The target is more than {max_offset_mm:g} mm from the taught hover position.")

    path = []
    angles = start.copy()
    pitch_axis = attitude[:, 2]
    for fraction in np.linspace(0, 1, max(1, math.ceil(distance / 5)) + 1)[1:]:
        point = origin + np.r_[offset * fraction, 0]

        def residual(q):
            position, orientation = tool_pose(q)
            return np.r_[position - point, 20 * (orientation[:, 2] - pitch_axis),
                         .12 * (q - start)]

        for _ in range(20):
            error = residual(angles)
            jacobian = np.column_stack([
                (residual(angles + step) - error) / .01 for step in np.eye(5) * .01
            ])
            update = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            angles -= np.clip(update, -2, 2)
            if np.linalg.norm(update) < .01:
                break
        position, orientation = tool_pose(angles)
        if (np.linalg.norm(position - point) > 1
                or np.dot(orientation[:, 2], pitch_axis) < math.cos(math.radians(8))
                or np.max(abs(angles - start)) > 15):
            raise ValueError("Could not find a small hover move preserving the tool direction.")
        path.append((point, angles.copy()))
    return path


def check_segments(start, path, limits, floor, check_point=None):
    for _, goal in path:
        if np.any(abs(goal) > limits):
            raise RuntimeError("A planned waypoint is outside the calibrated joint limits.")
        for fraction in np.linspace(0, 1, 12):
            point = tool_pose(start + fraction * (goal - start))[0]
            if point[2] < floor:
                raise RuntimeError("A planned segment passes below its clearance floor.")
            if check_point is not None:
                check_point(point)
        start = goal


def track_pickup_path(bus, path, limits, floor, descent_xy=None):
    for point, goal in path:
        current = read_arm(bus)
        origin_xy = tool_pose(current)[0][:2] if descent_xy is None else descent_xy
        delta = point[:2] - origin_xy
        length_squared = float(delta @ delta)
        lateral_limit = 12 if descent_xy is None else 6

        def check_point(position):
            offset = position[:2] - origin_xy
            if descent_xy is None and length_squared > 0:
                offset = offset - np.clip((offset @ delta) / length_squared, 0, 1) * delta
            if position[2] < floor or np.linalg.norm(offset) > lateral_limit:
                raise RuntimeError("The tool left the allowed height or lateral corridor.")

        def monitor():
            check_point(tool_pose(read_arm(bus))[0])

        def validate_command(actual, command):
            check_segments(actual, [(point, command)], limits, floor, check_point)

        validate_command(current, goal)
        move_arm(bus, goal, limits, monitor, peak_speed=5)
        position = settle_tool(bus, goal, point, floor, limits, monitor, validate_command)
        error = position - point
        print(f"Tracked tool error (mm): X {error[0]:+.1f}, Y {error[1]:+.1f}, Z {error[2]:+.1f}.",
              flush=True)


def plan_descent(hover, grasp, park, lift_mm, limits):
    hover_point, hover_orientation = tool_pose(hover)
    grasp_point, grasp_orientation = tool_pose(grasp)
    height = float(grasp_point[2] - hover_point[2])
    if not -70 <= height <= -15:
        raise ValueError("The taught grasp must be 15–70 mm below hover.")
    if np.dot(hover_orientation[:, 2], grasp_orientation[:, 2]) < math.cos(math.radians(12)):
        raise ValueError("The taught grasp and hover have different approach directions.")
    descent = vertical_path(hover, height)
    lift = vertical_path(descent[-1][1], lift_mm)
    check_segments(hover, descent + lift, limits, grasp_point[2] - 1)
    check_segments(lift[-1][1], [(tool_pose(park)[0], park)], limits, hover_point[2] - 5)
    return descent


def get_target(args, calibration, registration):
    frame = latest_frame(args.qbot)
    obj, _, belt = pickup_target(frame, calibration, args.label, args.max_age)
    target, error = arm_target(belt, registration)
    return obj, target, error


def verify_clear_slot(args, calibration, before_id, margin_mm=0):
    deadline = monotonic() + 15
    observed = set()
    while monotonic() < deadline:
        try:
            frame = latest_frame(args.qbot)
        except URLError:
            sleep(.5)
            continue
        capture_id = frame.get("capture_id")
        if (not capture_id or capture_id == before_id or capture_id in observed
                or frame["age_seconds"] > args.max_age
                or frame["image_size_px"] != calibration["image_size_px"]):
            sleep(.5)
            continue
        observed.add(capture_id)
        for obj in frame["objects"]:
            if obj["label"].casefold() != args.label.casefold():
                continue
            try:
                pickup_target(dict(frame, objects=[obj]), calibration, args.label, args.max_age, margin_mm)
            except ValueError:
                continue
            print("The part is still detected in the pickup area.")
            return
        if len(observed) >= 2:
            print("Two new frames had no matching detection in the pickup area. Check the bin before restarting the belt.")
            return
        sleep(.5)
    print("No fresh camera confirmation. Check the belt and bin before restarting the conveyor.")


def check_start_pose(bus, taught, opening, pick=False, inspect_only=False):
    actual = read_arm(bus)
    difference = tool_pose(actual)[0] - tool_pose(taught)[0]
    joint_error = abs(actual - taught)
    max_error = np.max(joint_error if pick else joint_error[:4])
    print(f"Gripper offset from taught hover (mm): X {difference[0]:+.1f}, "
          f"Y {difference[1]:+.1f}, Z {difference[2]:+.1f}.", flush=True)
    if max_error > 4.5 or (not pick and np.linalg.norm(difference) > 20):
        if inspect_only:
            print("Joint offsets in degrees (pan, lift, elbow, wrist flex, roll): "
                  f"{(actual - taught).round(1)}.", flush=True)
        raise RuntimeError(
            f"Arm is outside the taught starting area (joint error {max_error:.1f} degrees). "
            "With the arm supported and torque off, reposition it at the taught hover, then engage hold."
        )
    if grip(bus)[0] < opening - 2:
        raise RuntimeError("Open the gripper to the taught width before starting.")
    torque = set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values())
    if torque != {1}:
        if inspect_only and torque == {0}:
            print("Position is in the starting area. Keep the support in place and engage hold.")
            return actual
        raise RuntimeError("The servos must already be holding at the starting pose.")
    if inspect_only:
        print("Starting pose is ready for a hover trial.")
    return actual


def run_motion(args, calibration, registration, data, start, grasp, park, opening):
    max_error = 5 if args.pick else 10
    independent_error = float(registration.get("independent_check_error_mm", math.inf))
    if not 0 <= independent_error <= max_error:
        raise ValueError(f"Motion needs an independent registration check within {max_error} mm.")
    if args.pick and data.get("object_label", "").casefold() != args.label.casefold():
        raise ValueError("Teach and label a grasp for this object before running a pickup.")
    if args.pick and not 0 <= registration.get("height_spread_mm", math.inf) <= 5:
        raise ValueError("Record all registration points at a consistent height before picking.")
    with arm_bus(args.port, data["robot_id"]) as (bus, limits):
        check_start_pose(bus, start, opening, args.pick)
        if input("Confirm belt motor power is off and the whole path is clear. "
                 "Type MOVE: ").strip().upper() != "MOVE":
            print("Move cancelled.")
            return
        obj, target, fit_error = get_target(args, calibration, registration)
        if fit_error > max_error:
            raise ValueError(f"Registration fit exceeds {max_error} mm.")
        actual = check_start_pose(bus, start, opening, args.pick)
        path = hover_path(actual, target)
        hover_point, hover = path[-1]
        hover_floor = tool_pose(actual)[0][2] - 3
        check_segments(actual, path, limits, hover_floor)
        if args.pick:
            plan_descent(hover, grasp, park, args.lift_mm, limits)

        print(f"Moving to the {obj['label']} hover target at arm XY {target.round(1)} mm.",
              flush=True)
        try:
            track_pickup_path(bus, path, limits, hover_floor)
            print("Hover reached. Gripper is open.", flush=True)
            if not args.pick:
                return
            confirmation = input("Check that the fingers straddle the part and the belt "
                                 "and arm path are clear. Type PICK to continue: ")
            if confirmation.strip().upper() != "PICK":
                print("Pickup cancelled at hover.")
                return
            _, updated_target, _ = get_target(args, calibration, registration)
            if np.linalg.norm(updated_target - target) > 3:
                raise RuntimeError("The detected target moved since the hover began.")
            require_hover(bus, hover)
            measured_hover = read_arm(bus)
            measured_point = tool_pose(measured_hover)[0]
            delta = measured_point - hover_point
            if abs(delta[2]) > 3 or np.linalg.norm(delta[:2]) > 5:
                raise RuntimeError("The tool moved since the hover was checked.")
            descent = plan_descent(measured_hover, grasp, park, args.lift_mm, limits)
            grasp_floor = descent[-1][0][2] - 3

            def approach(path):
                track_pickup_path(bus, path, limits, grasp_floor, descent_xy=measured_point[:2])

            pick_and_place(args, bus, limits, descent, park, opening, approach=approach)
        except BaseException:
            hold_position(bus, limits)
            raise

    # Check only frames received after the arm has reached the bin.
    try:
        before_id = latest_frame(args.qbot).get("capture_id")
    except URLError:
        print("Camera unavailable after drop-off. Check the belt and bin before restarting the conveyor.")
        return
    verify_clear_slot(args, calibration, before_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("registration", type=Path)
    parser.add_argument("waypoints", type=Path)
    parser.add_argument("--qbot", default="http://192.168.0.121:8000")
    parser.add_argument("--label", default="Tape roll")
    parser.add_argument("--max-age", type=float, default=8)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check-start", action="store_true", help="Read the starting pose without moving")
    modes.add_argument("--hover", action="store_true", help="Execute an open-gripper hover move")
    modes.add_argument("--pick", action="store_true", help="Execute one supervised pickup")
    add_motion_arguments(parser)
    args = parser.parse_args()
    validate_motion_arguments(parser, args)
    if not 0 < args.max_age <= 15:
        parser.error("--max-age must be greater than zero and at most 15 seconds")

    try:
        data = json.loads(args.waypoints.read_text(encoding="utf-8"))
        poses = data["waypoints"]
        start = joint_values(poses["above_pickup"])
        opening = max(poses["grasp"]["gripper.pos"], poses["above_pickup"]["gripper.pos"])
        if args.check_start:
            with arm_bus(args.port, data["robot_id"]) as (bus, _):
                check_start_pose(bus, start, opening, inspect_only=True)
            return
        calibration, registration = (
            json.loads(path.read_text(encoding="utf-8"))
            for path in (args.calibration, args.registration)
        )
        if registration.get("calibration") != args.calibration.name:
            raise ValueError("Registration references a different camera calibration file.")
        digest = registration.get("calibration_sha256")
        if digest and digest != hashlib.sha256(args.calibration.read_bytes()).hexdigest():
            raise ValueError("The camera calibration changed after registration.")
        if registration.get("robot_id", data["robot_id"]) != data["robot_id"]:
            raise ValueError("The registration and waypoints belong to different robots.")
        grasp, park = (joint_values(poses[name]) for name in ("grasp", "above_bin"))
        if args.grasp_offset_mm:
            grasp = vertical_path(grasp, args.grasp_offset_mm)[-1][1]
        obj, target, error = get_target(args, calibration, registration)
        path = hover_path(start, target)
        print(f"{obj['label']} ({obj['confidence']:.0%}); arm XY {target.round(1)} mm")
        print(f"Registration fit error {error:.1f} mm; {len(path)} planned hover steps")
        if args.hover or args.pick:
            if opening <= args.close_limit:
                raise ValueError("Taught opening must exceed the jaw closing limit.")
            run_motion(args, calibration, registration, data, start, grasp, park, opening)
    except (URLError, OSError, KeyError, ValueError, RuntimeError, EOFError) as error:
        parser.exit(1, f"Pickup workflow stopped: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Pickup workflow interrupted. Check the arm before restarting the conveyor.\n")


if __name__ == "__main__":
    main()
