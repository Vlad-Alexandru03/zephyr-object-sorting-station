"""Run one automatic camera-guided pickup with re-detection after a missed grip."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from time import monotonic, sleep
from urllib.error import URLError

import numpy as np

from preview_pickup import arm_target, belt_objects, latest_frame
from so101_pick import (
    CONTACT_LOAD, GripMiss, add_motion_arguments, arm_bus, grip, hold_position, joint_values,
    move_arm, pick_and_place, read_arm, require_hover, set_gripper, tool_pose,
    validate_motion_arguments, vertical_path,
)
from vision_pick import (
    check_segments, hover_path, plan_descent, track_pickup_path, verify_clear_slot,
)

MAX_ALIGNMENT_ERROR_MM = 10
MAX_HEIGHT_SPREAD_MM = 5
HOVER_HEADROOM_MM = 12


class TransferClearanceError(RuntimeError):
    pass


class HoverTrackingError(RuntimeError):
    pass


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_station(path):
    station = read_json(path)
    observation = joint_values(station["waypoints"]["observation"])
    if not np.isfinite(observation).all():
        raise ValueError("The observation waypoint must contain finite joint angles.")
    profiles = {}
    for label, filename in station["objects"].items():
        profile = read_json(path.parent / filename)
        if profile["robot_id"] != station["robot_id"]:
            raise ValueError(f"The {label} waypoint file belongs to a different arm.")
        if profile["object_label"].casefold() != label.casefold():
            raise ValueError(f"The {label} waypoint file contains a different grasp label.")
        if label.casefold() in profiles:
            raise ValueError(f"Duplicate object label {label}.")
        poses = profile["waypoints"]
        hover, grasp, bin_pose = (joint_values(poses[key])
                                 for key in ("above_pickup", "grasp", "above_bin"))
        opening = max(poses["grasp"]["gripper.pos"], poses["above_pickup"]["gripper.pos"])
        if not np.isfinite([hover, grasp, bin_pose]).all() or not 0 < opening <= 100:
            raise ValueError(f"The {label} waypoint file contains invalid joint or gripper positions.")
        profiles[label.casefold()] = (hover, grasp, bin_pose, opening)
    if not profiles:
        raise ValueError("Define at least one object and its waypoint file in the station file.")
    return station["robot_id"], observation, profiles


def wait_for_object(args, calibration, profiles, after, expected_label=None):
    """Find one configured object in a fresh frame from the pickup area."""
    deadline = monotonic() + args.wait_seconds
    previous = None
    seen = set()
    reason = "No new camera frame."
    while monotonic() < deadline:
        request_started = monotonic()
        try:
            frame = latest_frame(args.qbot)
        except (URLError, OSError):
            sleep(.5)
            continue
        capture_id = frame.get("capture_id")
        age = frame["age_seconds"]
        if (not capture_id or capture_id in seen or not 0 <= age <= args.max_age
                or request_started - age < after):
            sleep(.5)
            continue
        seen.add(capture_id)
        candidates = belt_objects(frame, calibration, args.max_age, args.pickup_margin_mm)
        if len(candidates) != 1:
            reason = f"Expected one object in the pickup area, found {len(candidates)}."
            previous = None
            continue
        obj, _, belt_xy = candidates[0]
        name = obj["label"].casefold()
        if name not in profiles or (expected_label is not None and name != expected_label):
            reason = f"Unexpected object in the pickup area: {obj['label']}."
            previous = None
            continue
        if not args.confirm_detection:
            return obj, belt_xy
        if previous is not None and previous[0] == name:
            if np.linalg.norm(np.asarray(previous[1]) - belt_xy) <= args.frame_agreement_mm:
                return obj, (np.asarray(previous[1]) + belt_xy) / 2
        previous = name, belt_xy
        reason = "Waiting for two frames that agree on the object position."
        sleep(.5)
    raise RuntimeError(reason)


def move_clear(bus, current, goal, limits, floor, speed=8):
    check_segments(current, [(tool_pose(goal)[0], goal)], limits, floor)

    def monitor():
        if tool_pose(read_arm(bus))[0][2] < floor - 3:
            raise TransferClearanceError("The arm dropped below the transfer clearance.")

    move_arm(bus, goal, limits, monitor, peak_speed=speed)
    sleep(.4)
    monitor()
    require_hover(bus, goal)


def move_to_observation(bus, observation, limits, profiles):
    current = read_arm(bus)
    if np.max(abs(current - observation)) <= 4.5:
        require_hover(bus, observation)
        return

    point = tool_pose(current)[0]
    observation_point = tool_pose(observation)[0]
    clearance = max(tool_pose(profile[0])[0][2] for profile in profiles.values())
    clearance = min(clearance + 10, observation_point[2])
    if np.linalg.norm(point[:2] - observation_point[:2]) > 60 or point[2] < clearance - 50:
        raise RuntimeError("The arm is outside the recovery area. Start near the raised observation pose.")

    # A single rising move avoids settling at each 5 mm lift waypoint.
    if point[2] < clearance and observation_point[2] >= clearance:
        route = np.array([tool_pose(current + t * (observation - current))[0]
                          for t in np.linspace(0, 1, 41)])
        low = route[route[:, 2] < clearance]
        if (np.all(np.diff(route[:, 2]) >= 0)
                and np.max(np.linalg.norm(low[:, :2] - point[:2], axis=1)) <= 12):
            floor = point[2] - 2
            check_segments(current, [(observation_point, observation)], limits, floor)

            def monitor_rise():
                actual = tool_pose(read_arm(bus))[0]
                if actual[2] < floor or np.min(np.linalg.norm(route - actual, axis=1)) > 7:
                    raise RuntimeError("The arm drifted away from its rising path to observation.")

            print("Raising the arm in one move to the saved observation pose.", flush=True)
            move_arm(bus, observation, limits, monitor_rise, peak_speed=6)
            sleep(.4)
            monitor_rise()
            require_hover(bus, observation)
            return

    lift = vertical_path(current, clearance - point[2]) if point[2] < clearance else []
    raised = lift[-1][1] if lift else current
    transfer_floor = clearance - 4
    check_segments(current, lift, limits, point[2] - 2)
    check_segments(raised, [(observation_point, observation)], limits, transfer_floor)
    print("Returning to the saved observation pose.", flush=True)
    if lift:
        print(f"Lifting {clearance - point[2]:.0f} mm before moving to observation.", flush=True)
        track_pickup_path(bus, lift, limits, point[2] - 2, descent_xy=point[:2])
    print("Moving to observation at low speed.", flush=True)
    move_clear(bus, read_arm(bus), observation, limits, transfer_floor, speed=6)


def retreat_after_miss(bus, observation, hover_height, opening, limits):
    # A clean miss permits reopening. A servo error or an unstable grip is
    # handled separately and leaves the current jaw command in place.
    set_gripper(bus, opening)
    sleep(.6)
    if abs(grip(bus)[0] - opening) > 2:
        raise RuntimeError("The gripper did not reopen after the missed grip.")
    start = read_arm(bus)
    point = tool_pose(start)[0]
    rise = max(0, hover_height - point[2])
    if rise > 70:
        raise RuntimeError("The retreat is outside the planned pickup height range.")
    if rise > 0:
        path = vertical_path(start, rise)
        track_pickup_path(bus, path, limits, point[2] - 3, descent_xy=point[:2])
    current = read_arm(bus)
    move_clear(bus, current, observation, limits, hover_height - 4)


def move_to_pickup_hover(args, bus, limits, profile, observation, target):
    taught_hover, grasp, bin_pose, opening = profile
    if opening <= args.close_limit:
        raise ValueError("The taught opening must exceed the jaw closing limit.")
    if not args.hover_only and args.grasp_offset_mm:
        grasp = vertical_path(grasp, args.grasp_offset_mm)[-1][1]
    height = tool_pose(taught_hover)[0][2]
    transfer_floor = height - 4
    current = read_arm(bus)
    _, nominal_hover = hover_path(taught_hover, target, args.max_hover_offset_mm)[-1]
    point, raised_hover = vertical_path(nominal_hover, HOVER_HEADROOM_MM)[-1]
    if not args.hover_only:
        expected_descent = plan_descent(raised_hover, grasp, bin_pose, args.lift_mm, limits)
        planned_lift = vertical_path(expected_descent[-1][1], args.lift_mm)
        check_segments(planned_lift[-1][1], [(tool_pose(bin_pose)[0], bin_pose)], limits, transfer_floor)
        check_segments(bin_pose, [(tool_pose(observation)[0], observation)], limits, transfer_floor)
    check_segments(raised_hover, [(tool_pose(observation)[0], observation)], limits, transfer_floor)
    check_segments(current, [(point, raised_hover)], limits, transfer_floor)
    set_gripper(bus, opening)
    sleep(.6)
    if abs(grip(bus)[0] - opening) > 2:
        raise RuntimeError("The gripper did not open to the taught width.")
    print(f"Moving above the box with {HOVER_HEADROOM_MM} mm of extra height.", flush=True)
    try:
        move_clear(bus, current, raised_hover, limits, transfer_floor, speed=6)
    except TransferClearanceError as error:
        raise HoverTrackingError(str(error)) from error
    measured_hover = read_arm(bus)
    point = tool_pose(measured_hover)[0]
    if np.linalg.norm(point[:2] - target) > 5:
        raise HoverTrackingError("The gripper did not reach the detected object's hover target.")

    return measured_hover, point, grasp, transfer_floor


def attempt_pick(args, bus, limits, profile, observation, target):
    measured_hover, point, grasp, transfer_floor = move_to_pickup_hover(
        args, bus, limits, profile, observation, target
    )
    if args.hover_only:
        return

    taught_hover, _, bin_pose, opening = profile

    descent = plan_descent(measured_hover, grasp, bin_pose, args.lift_mm, limits)
    floor = descent[-1][0][2] - 3

    def approach(path):
        track_pickup_path(bus, path, limits, floor, descent_xy=point[:2])

    try:
        pick_and_place(args, bus, limits, descent, bin_pose, opening, approach=approach,
                       carry_floor=transfer_floor)
    except GripMiss:
        retreat_after_miss(bus, observation, tool_pose(taught_hover)[0][2], opening, limits)
        return False
    move_clear(bus, read_arm(bus), observation, limits, transfer_floor)
    return True


def run_cycle(args, bus, limits, calibration, registration, observation, profiles):
    previous_xy = None
    label = None
    after = monotonic()
    hover_recovered = False
    for attempt in range(1, args.attempts + 1):
        obj, belt_xy = wait_for_object(args, calibration, profiles, after, label)
        label = obj["label"].casefold()
        if max(abs(np.asarray(belt_xy))) > calibration["square_side_mm"] / 2:
            print(f"Using the measured offset outside the square: "
                  f"belt X {belt_xy[0]:+.1f} mm, Y {belt_xy[1]:+.1f} mm.", flush=True)
        target, residual = arm_target(belt_xy, registration)
        if residual > MAX_ALIGNMENT_ERROR_MM:
            raise RuntimeError(f"Registration fit error {residual:.1f} mm exceeds "
                               f"{MAX_ALIGNMENT_ERROR_MM} mm.")
        if previous_xy is not None:
            offset = np.asarray(belt_xy) - previous_xy
            print(f"Re-detected {obj['label']}; belt shift X {offset[0]:+.1f} mm, "
                  f"Y {offset[1]:+.1f} mm.", flush=True)
        print(f"Attempt {attempt}/{args.attempts}: {obj['label']} ({obj['confidence']:.0%}), "
              f"arm XY {target.round(1)} mm.", flush=True)
        try:
            picked = attempt_pick(args, bus, limits, profiles[label], observation, target)
        except HoverTrackingError as error:
            if hover_recovered or attempt == args.attempts:
                raise
            hold_position(bus, limits)
            opening, load = grip(bus)
            grasp_height = tool_pose(profiles[label][1])[0][2]
            actual_height = tool_pose(read_arm(bus))[0][2]
            if (opening <= args.close_limit + 2 or load >= CONTACT_LOAD
                    or actual_height < grasp_height + 5):
                raise
            print(f"{error} Returning to observation to retry with a fresh detection.", flush=True)
            move_to_observation(bus, observation, limits, profiles)
            hover_recovered = True
            previous_xy = np.asarray(belt_xy)
            after = monotonic()
            continue
        if args.hover_only:
            print(f"Hover reached above {obj['label']}. Gripper is open.", flush=True)
            return None
        if picked:
            print(f"Released {obj['label']} above its taught bin and returned to observation.", flush=True)
            return obj["label"]
        previous_xy = np.asarray(belt_xy)
        after = monotonic()
        print("Grip missed. Gripper reopened and retreated to the camera observation pose.", flush=True)
    raise RuntimeError(f"No confirmed grip after {args.attempts} attempts.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("registration", type=Path)
    parser.add_argument("station", type=Path)
    parser.add_argument("--qbot", default="http://192.168.0.121:8000")
    parser.add_argument("--max-age", type=float, default=8)
    parser.add_argument("--wait-seconds", type=float, default=25)
    parser.add_argument("--pickup-margin-mm", type=float, default=10,
                        help="Search this far beyond each edge of the calibrated square (0 to 15 mm)")
    parser.add_argument("--confirm-detection", action="store_true",
                        help="Require two fresh frames with matching detections")
    parser.add_argument("--frame-agreement-mm", type=float, default=3,
                        help="Center tolerance for --confirm-detection, between 1 and 6 mm")
    parser.add_argument("--max-hover-offset-mm", type=float, default=25,
                        help="Maximum XY correction from taught hover, between 1 and 40 mm")
    parser.add_argument("--attempts", type=int, default=3, choices=range(1, 6))
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--execute", action="store_true", help="Pick and place after detection")
    modes.add_argument("--hover-only", action="store_true", help="Move above the detected object and stop")
    parser.add_argument("--from-current", action="store_true",
                        help="Return a nearby arm with servos holding to observation before detection")
    add_motion_arguments(parser)
    parser.set_defaults(grasp_offset_mm=0, lift_mm=30 + HOVER_HEADROOM_MM)
    args = parser.parse_args()
    validate_motion_arguments(parser, args)
    if not 0 < args.max_age <= 15 or not 0 < args.wait_seconds <= 120:
        parser.error("Use a frame age of at most 15 seconds and a wait of at most 120 seconds")
    if not 1 <= args.max_hover_offset_mm <= 40:
        parser.error("Use a hover offset limit between 1 and 40 mm")
    if not 1 <= args.frame_agreement_mm <= 6:
        parser.error("Use a frame agreement limit between 1 and 6 mm")
    if not 0 <= args.pickup_margin_mm <= 15:
        parser.error("Use a pickup margin between 0 and 15 mm")
    if args.from_current and not (args.execute or args.hover_only):
        parser.error("--from-current requires --execute or --hover-only")
    try:
        calibration = read_json(args.calibration)
        registration = read_json(args.registration)
        robot_id, observation, profiles = load_station(args.station)
        if (registration.get("calibration") != args.calibration.name
                or registration.get("robot_id", robot_id) != robot_id):
            raise ValueError("Registration belongs to a different calibration or robot.")
        digest = registration.get("calibration_sha256")
        if digest and digest != hashlib.sha256(args.calibration.read_bytes()).hexdigest():
            raise ValueError("The camera calibration changed after registration.")
        if not (args.execute or args.hover_only):
            obj, belt_xy = wait_for_object(args, calibration, profiles, after=0)
            target, error = arm_target(belt_xy, registration)
            print(f"{obj['label']} ({obj['confidence']:.0%}): arm XY {target.round(1)} mm. "
                  f"Registration fit error {error:.1f} mm. Its taught bin would be selected.")
            return
        _, fit_error = arm_target((0, 0), registration)
        check_error = registration.get("independent_check_error_mm", math.inf)
        if not all(0 <= error <= MAX_ALIGNMENT_ERROR_MM for error in (fit_error, check_error)):
            raise ValueError(f"Automatic pickup needs a registration fit and independent check "
                             f"within {MAX_ALIGNMENT_ERROR_MM} mm.")
        if not 0 <= registration.get("height_spread_mm", math.inf) <= MAX_HEIGHT_SPREAD_MM:
            raise ValueError(f"Automatic pickup needs a recorded reference height spread "
                             f"within {MAX_HEIGHT_SPREAD_MM} mm.")

        with arm_bus(args.port, robot_id) as (bus, limits):
            if set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values()) != {1}:
                raise RuntimeError("All servos must already be holding.")
            opening, load = grip(bus)
            if opening <= args.close_limit + 2 or load >= CONTACT_LOAD:
                raise RuntimeError("Start with an open, empty gripper.")
            try:
                if args.from_current:
                    move_to_observation(bus, observation, limits, profiles)
                else:
                    require_hover(bus, observation)
                detection = "two fresh detections that agree" if args.confirm_detection else "one fresh detection"
                print(f"At observation. Waiting up to {args.wait_seconds:g} seconds for {detection}.",
                      flush=True)
                label = run_cycle(args, bus, limits, calibration, registration, observation, profiles)
            except BaseException:
                hold_position(bus, limits)
                raise

        if args.hover_only:
            return
        args.label = label
        try:
            before_id = latest_frame(args.qbot).get("capture_id")
        except (URLError, OSError):
            print("Drop command completed. Camera response unavailable for the pickup area check.")
            return
        verify_clear_slot(args, calibration, before_id, args.pickup_margin_mm)
    except (URLError, OSError, KeyError, ValueError, RuntimeError) as error:
        parser.exit(1, f"Automatic sort stopped: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Automatic sort interrupted. Check the arm and belt.\n")


if __name__ == "__main__":
    main()
