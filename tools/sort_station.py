"""Run the conveyor, classify the stopped item, and sort it using taught poses."""

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
from secrets import randbelow
from subprocess import PIPE, Popen
from threading import Thread
from time import monotonic, sleep
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import numpy as np

from auto_sort import load_station, move_to_observation
from pickup_pose import (
    annotated_jpeg, background_path, box_pose, load_pose_config, placement_error, save_pose_config,
    source_image,
)
from preview_pickup import belt_objects
from so101_pick import (
    CONTACT_LOAD, JOINTS, add_motion_arguments, arm_bus, close_gripper, follow_lift, grip, hold_position,
    lift_path, move_arm, read_arm, require_hover, send_arm, set_gripper, settle_tool, tool_pose,
    validate_motion_arguments, vertical_path,
)
from station_dashboard import Dashboard, dashboard_server
from vision_pick import check_segments

PICKUP_LIFT_MM = 30
PREGRASP_MM = 12
HOVER_RAISE_MM = 10
GRASP_RAISE_MM = 10
BIN_RAISE_MM = 20
DEFAULT_STOP_MM = 25


@dataclass(frozen=True)
class BeltStatus:
    command: int
    distance_mm: int
    duty: int
    pulses: int
    run_allowed: bool


class Conveyor:
    def __init__(self, process, dashboard):
        self.process = process
        self.dashboard = dashboard
        self.status = SimpleQueue()
        self.command = randbelow(900_000_000) + 1
        self.last_status = None
        Thread(target=self.read_status, daemon=True).start()

    def read_status(self):
        for line in self.process.stdout:
            fields = line.split()
            if len(fields) != 6 or fields[0] != b"SORT":
                continue
            try:
                command, distance, duty, pulses, run_allowed = map(int, fields[1:])
            except ValueError:
                continue
            if 0 <= duty <= 100 and run_allowed in (0, 1) and pulses >= 0:
                status = BeltStatus(command, distance, duty, pulses, bool(run_allowed))
                self.dashboard.belt_sample(status)
                self.status.put(status)
        self.status.put(None)

    def next_status(self, timeout):
        try:
            status = self.status.get(timeout=max(0, timeout))
        except Empty as error:
            raise RuntimeError("No reply from the conveyor MCU. Check the bridge and firmware.") from error
        if status is None:
            raise RuntimeError("The conveyor UART bridge disconnected.")
        return status

    def command_status(self, mode, stop_mm=DEFAULT_STOP_MM):
        self.command = self.command % 999_999_999 + 1
        threshold = f" {stop_mm}" if mode == "R" else ""
        command = f"{mode} {self.command}{threshold}\n".encode("ascii")
        try:
            self.process.stdin.write(command)
            self.process.stdin.flush()
        except OSError as error:
            raise RuntimeError("The conveyor UART bridge closed. Check its terminal output.") from error
        deadline = monotonic() + 1
        while monotonic() < deadline:
            status = self.next_status(deadline - monotonic())
            if status.command != self.command:
                continue
            if mode == "R":
                acknowledged = status.run_allowed or (status.duty == 0 and 0 < status.distance_mm < stop_mm)
            else:
                acknowledged = not status.run_allowed and status.duty == 0
            if acknowledged:
                self.last_status = status
                return status
        raise RuntimeError("The conveyor did not acknowledge the requested state.")

    def set_running(self, running, stop_mm=DEFAULT_STOP_MM):
        return self.command_status("R" if running else "S", stop_mm)

    def stop_and_wait(self):
        return self.wait_for_quiet(self.set_running(False))

    def jog_and_wait(self):
        start = self.last_status.pulses
        stopped_at = self.wait_for_quiet(self.command_status("J"))
        pulses = (self.last_status.pulses - start) % (1 << 32)
        if not 1 <= pulses <= 4:
            raise RuntimeError(f"Expected a small forward jog, but the encoder recorded {pulses} pulses.")
        return stopped_at, pulses

    def wait_for_quiet(self, status):
        quiet_since = monotonic()
        deadline = quiet_since + 3
        while monotonic() < deadline:
            previous = status
            status = self.next_status(deadline - monotonic())
            if status.command != self.command or status.run_allowed or status.duty != 0:
                raise RuntimeError("Conveyor moved after the stop acknowledgment.")
            if status.pulses != previous.pulses:
                quiet_since = monotonic()
            elif monotonic() - quiet_since >= .4:
                self.last_status = status
                return monotonic()
        raise RuntimeError("The encoder did not settle after stopping the conveyor.")


@contextmanager
def conveyor_session(args):
    command = [args.adb, "-s", args.conveyor, "shell", "-T", "python3", "/home/arduino/conveyor_uart.py"]
    with Popen(command, stdin=PIPE, stdout=PIPE) as process:
        belt = Conveyor(process, args.telemetry)
        try:
            belt.set_running(False)
            yield belt
        finally:
            try:
                belt.set_running(False)
            except (OSError, RuntimeError):
                print("Conveyor link unavailable. The MCU run permission expires on its own.", flush=True)
            finally:
                process.terminate()


def camera_frame(args):
    try:
        with urlopen(args.qbot.rstrip("/") + "/latest.json", timeout=.6) as response:
            frame = json.load(response)
    except (URLError, OSError):
        return None
    if not (frame.get("capture_id") and 0 <= frame["age_seconds"] <= args.max_frame_age):
        return None
    return frame


def objects_at_pickup(args, frame, calibration):
    if args.roi:
        left, top, right, bottom = args.roi
        return [obj for obj in frame["objects"]
                if left <= (obj["box_xyxy"][0] + obj["box_xyxy"][2]) / 2 <= right
                and top <= (obj["box_xyxy"][1] + obj["box_xyxy"][3]) / 2 <= bottom]
    return [obj for obj, _, _ in belt_objects(frame, calibration, args.max_frame_age)]


def wait_for_clear(args, calibration, after):
    deadline = monotonic() + args.camera_wait
    while monotonic() < deadline:
        request_started = monotonic()
        frame = camera_frame(args)
        if (frame is not None and request_started - frame["age_seconds"] >= after
                and not objects_at_pickup(args, frame, calibration)):
            return
        sleep(.15)
    raise RuntimeError("Camera did not confirm the pickup area was clear.")


def seek_item(args, belt):
    args.telemetry.next_item()
    args.telemetry.stage("seek", "Conveyor looking for a part", "Waiting for the ultrasonic trigger.")
    print("Waiting for the ultrasonic sensor. The MCU stops the belt when it sees an item.", flush=True)
    pickup_deadline = monotonic() + args.pickup_wait if args.pickup_wait else math.inf
    status = belt.set_running(False)
    start_pulses = status.pulses
    while monotonic() < pickup_deadline:
        if 0 < status.distance_mm < args.stop_mm:
            args.telemetry.stage("inspect", f"Object detected {status.distance_mm} mm away",
                                 "Stopping the belt and waiting for a fresh camera frame.",
                                 stop_mm=status.distance_mm)
            print(f"Pickup detected at {status.distance_mm} mm. Stopping the belt.", flush=True)
            stopped_at = belt.stop_and_wait()
            pulses = (belt.last_status.pulses - start_pulses) % (1 << 32)
            print(f"The belt encoder recorded {pulses} pulses before the stop.", flush=True)
            return stopped_at
        status = belt.set_running(True, args.stop_mm)
    raise RuntimeError("The ultrasonic sensor did not see an item within the pickup wait.")


def wait_for_pose(args, calibration, background, after, expected_label=None):
    deadline = monotonic() + args.camera_wait
    seen = set()
    while monotonic() < deadline:
        requested_at = monotonic()
        frame = camera_frame(args)
        if (frame is not None and frame["capture_id"] not in seen
                and requested_at - frame["age_seconds"] >= after):
            objects = objects_at_pickup(args, frame, calibration)
            if len(objects) == 1:
                obj = objects[0]
                if expected_label and obj["label"].casefold() != expected_label:
                    raise RuntimeError(f"Expected {expected_label}, but the camera sees {obj['label']}.")
                image = source_image(args.qbot, frame)
                if image is not None:
                    seen.add(frame["capture_id"])
                    pose = box_pose(image, background, obj["box_xyxy"])
                    if pose is not None:
                        return obj, pose, image
        sleep(.15)
    raise RuntimeError("No fresh, reliable box outline after the belt stopped. Check the camera and box.")


def align_item(args, belt, calibration, pose_config, profiles, stopped_at):
    data, background = pose_config
    label = None
    for jog in range(args.max_jogs + 1):
        obj, pose, image = wait_for_pose(args, calibration, background, stopped_at, label)
        label = obj["label"].casefold()
        target = data["targets"].get(label)
        if label not in profiles or not target or "forward" not in target:
            raise RuntimeError(f"No complete pickup and bin setup for {obj['label']}.")
        along, across, angle = placement_error(target, pose)
        args.telemetry.classified(image, obj, pose, target, args.roi, along, across, angle)
        args.telemetry.stage("inspect", f"Classified {obj['label']}", "Checking the taught pickup position and angle.")
        if not .75 <= pose.aspect / target["aspect"] <= 1.33:
            raise RuntimeError("The box outline differs from the taught shape. Belt stays stopped.")
        print(f"{obj['label']} ({obj['confidence']:.0%}): forward {along:+.1f} px, "
              f"sideways {across:+.1f} px, angle {angle:+.1f} degrees.", flush=True)
        if abs(across) > args.pickup_position_px:
            raise RuntimeError("The item is too far sideways for the belt to correct.")
        if abs(along) <= args.pickup_position_px:
            if abs(angle) <= args.pickup_angle_deg:
                return obj
            raise RuntimeError("The box reached pickup at the wrong angle. Align it or use a guide.")
        if along < 0:
            raise RuntimeError("The box has passed the pickup point. Forward jogging cannot correct it.")
        if jog == args.max_jogs:
            break
        args.telemetry.stage("jog", "Nudging the conveyor forward", "The camera will recheck after the belt stops.",
                             jog={"index": jog + 1, "limit": args.max_jogs})
        stopped_at, pulses = belt.jog_and_wait()
        print(f"Jog {jog + 1}/{args.max_jogs}: {pulses} encoder pulse(s). Checking a new frame.", flush=True)
    raise RuntimeError("The box did not reach the pickup point within the jog limit.")


def taught_object(args, profiles):
    objects = json.loads(args.station.read_text(encoding="utf-8"))["objects"]
    label = args.object.casefold()
    if label not in profiles:
        raise ValueError(f"No taught pickup and bin for {args.object}.")
    display = next((name for name in objects if name.casefold() == label), label)
    return label, display


def show_detection(args, frame, objects, selected, source=False):
    """Publish a classified frame to the dashboard without letting it block the run."""
    if not args.telemetry.enabled:
        return
    if source:
        image = source_image(args.qbot, frame)
        if image is not None:
            args.telemetry.detection(image, objects, args.roi, selected)
            return
    jpeg = annotated_jpeg(args.qbot, frame["capture_id"])
    if jpeg is not None:
        args.telemetry.store_frame(jpeg)


def classify_object(args, calibration, profiles, stopped_at):
    """Choose a taught route using a frame received after the belt settled."""
    deadline = monotonic() + args.classify_wait
    seen = set()
    reason = "No fresh camera frame after the conveyor stopped."
    while monotonic() < deadline:
        requested_at = monotonic()
        frame = camera_frame(args)
        if (frame is not None and requested_at - frame["age_seconds"] >= stopped_at
                and frame["capture_id"] not in seen):
            seen.add(frame["capture_id"])
            objects = (objects_at_pickup(args, frame, calibration)
                       if args.roi or calibration is not None else frame["objects"])
            if len(objects) == 1:
                obj = objects[0]
                label = obj["label"].casefold()
                if label in profiles:
                    show_detection(args, frame, objects, obj, source=True)
                    return label, obj["label"], obj
                reason = f"No taught pickup and bin for {obj['label']}."
            else:
                reason = f"Expected one object in the pickup area. Camera found {len(objects)}."
            show_detection(args, frame, objects, None)
        sleep(.15)
    if args.object is None:
        raise RuntimeError(reason)
    label, display = taught_object(args, profiles)
    print(f"{reason} Using the explicit {display} fallback.", flush=True)
    return label, display, None


def teach_pose(args, pose_config):
    path = args.pose_config
    after = monotonic()
    if args.teach_empty:
        deadline = after + args.camera_wait
        while monotonic() < deadline:
            requested_at = monotonic()
            frame = camera_frame(args)
            if (frame is not None and requested_at - frame["age_seconds"] >= after
                    and not objects_at_pickup(args, frame, None)):
                image = source_image(args.qbot, frame)
                if image is not None:
                    if args.roi[2] > image.shape[1] or args.roi[3] > image.shape[0]:
                        raise ValueError("The pickup ROI extends outside the camera image.")
                    path.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(background_path(path)), image):
                        raise OSError("Could not save the empty belt reference image.")
                    save_pose_config(path, {"image_size_px": frame["image_size_px"],
                                            "roi": args.roi, "targets": {}})
                    print("Saved the empty belt image. Teach the target and upstream positions next.")
                    return
            sleep(.15)
        raise RuntimeError("No fresh empty pickup image. Clear the belt and start live capture.")

    data, background = pose_config
    label = args.teach_target or args.teach_upstream
    key = label.casefold()
    if args.teach_upstream and key not in data["targets"]:
        raise ValueError(f"Teach the pickup target for {label} first.")
    _, pose, _ = wait_for_pose(args, None, background, after, key)
    if args.teach_target:
        if args.angle_period == 180 and pose.aspect < 1.10:
            raise ValueError("The reference looks square. Teach again with --angle-period 90 if quarter turns are equivalent.")
        data["targets"][key] = {"center": pose.center.tolist(), "angle": pose.angle,
                                "angle_period": args.angle_period, "aspect": pose.aspect}
        print(f"Saved {label} at {pose.center.round(1)} px and {pose.angle:.1f} degrees.")
    else:
        target = data["targets"][key]
        direction = np.asarray(target["center"]) - pose.center
        length = np.linalg.norm(direction)
        if length < 12:
            raise ValueError("Move the box farther upstream while keeping it in the camera ROI.")
        target["forward"] = (direction / length).tolist()
        print(f"Saved the forward direction from the upstream position {pose.center.round(1)} px.")
    save_pose_config(path, data)


def move_fixed(bus, goal, limits, floor, speed, check_grip=None, settle=False, reuse_trim=False,
               smooth=False):
    point = tool_pose(goal)[0]
    current = read_arm(bus)
    held = bus.sync_read("Goal_Position", list(JOINTS), num_retry=3)
    held_angles = np.array([held[joint] for joint in JOINTS])
    if (not np.isfinite(held_angles).all() or np.any(abs(held_angles) > limits)
            or np.max(abs(held_angles - current)) > 4.5):
        raise RuntimeError("The current servo goals are too far from the measured holding pose.")
    command_goal = goal
    check_segments(current, [(point, goal)], limits, floor)
    if reuse_trim:
        trim = np.clip(held_angles - current, -4, 4)
        command_goal = np.clip(goal + trim, -limits, limits)
        check_segments(current, [(point, command_goal)], limits, floor)

    def monitor_grip():
        if check_grip is not None:
            check_grip()

    def check_height(actual):
        actual_height = tool_pose(actual)[0][2]
        if actual_height < floor:
            raise RuntimeError(f"Arm height {actual_height:.1f} mm fell below clearance {floor:.1f} mm.")

    def sample_arm():
        monitor_grip()
        actual = read_arm(bus)
        check_height(actual)
        return actual

    if smooth:
        check_segments(held_angles, [(point, command_goal)], limits, floor)
        move_arm(bus, command_goal, limits, monitor_grip, peak_speed=speed, preserve_hold=True,
                 check_position=check_height)
    else:
        step_degrees = float(np.max(abs(command_goal - current)))
        count = max(1, math.ceil(step_degrees / 5))
        dwell = max(.12, step_degrees / count / speed)
        actual = current
        for index, waypoint in enumerate(np.linspace(current, command_goal, count + 1)[1:], start=1):
            check_segments(actual, [(tool_pose(waypoint)[0], waypoint)], limits, floor)
            send_arm(bus, waypoint, limits)
            sent_at = monotonic()
            while True:
                actual = sample_arm()
                error = abs(actual - waypoint)
                age = monotonic() - sent_at
                if age >= dwell and np.max(error) <= 4:
                    break
                if age >= 2:
                    joint = int(np.argmax(error))
                    raise RuntimeError(f"Arm waypoint {index}/{count} did not arrive. "
                                       f"{JOINTS[joint]} is at {actual[joint]:.1f} degrees, "
                                       f"target {waypoint[joint]:.1f} degrees.")
                sleep(.04)

    if not settle:
        deadline = monotonic() + 1.5
        settled = 0
        while monotonic() < deadline:
            actual = sample_arm()
            settled = settled + 1 if np.max(abs(actual - goal)) <= 2.5 else 0
            if settled >= 3:
                return
            sleep(.05)

    def validate_correction(actual, command):
        check_segments(actual, [(point, command)], limits, floor)

    settle_tool(bus, goal, point, floor, limits, sample_arm, validate_correction,
                lateral_tolerance_mm=5, height_tolerance_mm=2.5, initial_command=command_goal)


def raised_pose(taught, height_mm, limits, label):
    """Raise the tool by height_mm, tilting only as far as the joint limits require."""
    origin, attitude = tool_pose(taught)
    target = origin + (0, 0, height_mm)
    pose = taught
    for step in range(1, int(round(height_mm)) + 1):
        candidate = vertical_path(taught, step)[-1][1]
        if np.any(np.abs(candidate) > limits):
            break
        pose = candidate
    if np.allclose(tool_pose(pose)[0], target, atol=.2):
        return pose

    radial = origin[:2] / max(np.linalg.norm(origin[:2]), 1e-9)
    for tilt_deg in (4, 8, 12, 16, 20):
        tilt = np.deg2rad(tilt_deg)
        for side in (-1, 1):
            axis = attitude[:, 2] * np.cos(tilt) + np.r_[radial * side, 0] * np.sin(tilt)
            axis /= np.linalg.norm(axis)
            angles = pose.copy()
            for _ in range(400):
                def residual(q):
                    position, orientation = tool_pose(q)
                    return np.r_[position - target, 40 * (orientation[:, 2] - axis),
                                 .01 * (q - taught)]

                error = residual(angles)
                if np.linalg.norm(error[:3]) < .1:
                    break
                jacobian = np.column_stack([(residual(angles + step) - error) / .01
                                            for step in np.eye(5) * .01])
                angles = np.clip(angles - np.clip(np.linalg.lstsq(jacobian, error, rcond=None)[0], -2, 2),
                                 -limits, limits)
            position, orientation = tool_pose(angles)
            if np.linalg.norm(position - target) < .8 and np.all(np.abs(angles) <= limits):
                reached = np.degrees(np.arccos(np.clip(orientation[:, 2] @ attitude[:, 2], -1, 1)))
                print(f"The arm tilts the tool {reached:.0f} degrees to raise the {label} pose.",
                      flush=True)
                return angles
    if np.allclose(pose, taught):
        raise RuntimeError(f"The arm cannot raise the {label} pose within its joint limits.")
    short = target[2] - tool_pose(pose)[0][2]
    print(f"The joint limits stop the {label} raise {short:.0f} mm short. Using the height it reaches.",
          flush=True)
    return pose


def sort_object(args, bus, limits, observation, profile, label):
    taught_hover, taught_grasp, taught_bin, opening = profile
    close_limit = args.gripper_close_limits.get(label.casefold(), args.close_limit)
    hover = raised_pose(taught_hover, HOVER_RAISE_MM, limits, "pickup hover")
    if label.casefold() in args.exact_pose_labels:
        grasp, above_bin = taught_grasp, taught_bin
    else:
        grasp = raised_pose(taught_grasp, GRASP_RAISE_MM, limits, "grasp")
        above_bin = raised_pose(taught_bin, BIN_RAISE_MM, limits, "drop-off")
    transfer_floor = tool_pose(taught_hover)[0][2] - 5
    grasp_height = tool_pose(grasp)[0][2]
    grasp_floor = grasp_height - 3
    carry_floor = grasp_height + PICKUP_LIFT_MM - 5
    if not grasp_floor < transfer_floor or opening <= close_limit:
        raise ValueError("Teach a grasp below the pickup hover with an open gripper.")
    planned_lift = vertical_path(grasp, PICKUP_LIFT_MM)
    _, pregrasp = vertical_path(grasp, PREGRASP_MM)[-1]
    pregrasp_floor = grasp_height + PREGRASP_MM - 5
    check_segments(grasp, planned_lift, limits, grasp_floor)
    for start, end, floor in ((read_arm(bus), hover, transfer_floor), (hover, pregrasp, pregrasp_floor),
                              (pregrasp, grasp, grasp_floor),
                              (planned_lift[-1][1], above_bin, carry_floor),
                              (above_bin, observation, transfer_floor)):
        check_segments(start, [(tool_pose(end)[0], end)], limits, floor)
    set_gripper(bus, opening)
    sleep(.6)
    if abs(grip(bus)[0] - opening) > 2:
        raise RuntimeError("The gripper did not open to its taught width.")
    destination = f"{label} bin"
    args.telemetry.stage("arm", f"Picking up {label}", "Moving above the taught pickup.", destination=destination)
    print("Moving above the taught pickup point.", flush=True)
    move_fixed(bus, hover, limits, transfer_floor, 28, settle=True)
    print(f"Settling {PREGRASP_MM} mm above the raised grasp.", flush=True)
    move_fixed(bus, pregrasp, limits, pregrasp_floor, 20, settle=True, reuse_trim=True)
    print(f"Lowering to the grasp at {grasp_height:.1f} mm.", flush=True)
    move_fixed(bus, grasp, limits, grasp_floor, 12, settle=True, reuse_trim=True)
    hold = close_gripper(bus, opening, close_limit, args.grip_torque_limit,
                         args.grip_load, args.grip_load_band)
    lift_start = read_arm(bus)
    lift_floor = tool_pose(lift_start)[0][2] - 3
    lift = lift_path(lift_start, PICKUP_LIFT_MM)[0]
    check_segments(lift_start, lift, limits, lift_floor)
    check_segments(lift[-1][1], [(tool_pose(above_bin)[0], above_bin)], limits, carry_floor)
    args.telemetry.stage("arm", "Grip confirmed", f"Lifting straight up {PICKUP_LIFT_MM} mm.")
    print(f"Grip confirmed. Lifting straight up {PICKUP_LIFT_MM} mm.", flush=True)
    follow_lift(bus, PICKUP_LIFT_MM, limits, hold.update, peak_speed=12)
    args.telemetry.stage("arm", f"Arm moving to {destination}", "Carrying the part to the taught drop-off pose.")
    print("Carrying to the taught bin.", flush=True)
    # Clearance for the carry follows where the arm actually got to, never below
    # the grasp height so the part cannot be dragged along the belt.
    lifted_height = tool_pose(read_arm(bus))[0][2]
    floor = min(carry_floor, max(lifted_height - 5, grasp_height))
    move_fixed(bus, above_bin, limits, floor, args.carry_speed, hold.update, smooth=True)
    set_gripper(bus, opening)
    sleep(.6)
    if abs(grip(bus)[0] - opening) > 2:
        raise RuntimeError("Could not verify the gripper opened over the bin.")
    args.telemetry.stage("arm", f"Released into {destination}", "Returning to observation.")
    move_fixed(bus, observation, limits, transfer_floor, 32, smooth=True)


def hold_current_pose(bus):
    torque = set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values())
    if torque == {1}:
        return False
    if torque != {0}:
        raise RuntimeError("Servo torque states disagree. Check the arm before arming it.")
    positions = bus.sync_read("Present_Position", normalize=False, num_retry=3)
    sleep(.2)
    current = bus.sync_read("Present_Position", normalize=False, num_retry=3)
    if max(abs(current[joint] - positions[joint]) for joint in current) > 5:
        raise RuntimeError("The arm moved while engaging hold. Hold it steady and try again.")
    bus.sync_write("Goal_Position", current, normalize=False, num_retry=3)
    if bus.sync_read("Goal_Position", normalize=False, num_retry=3) != current:
        raise RuntimeError("Could not verify the holding targets. Torque remains off.")
    bus.sync_write("Torque_Enable", 1, normalize=False, num_retry=3)
    if set(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values()) != {1}:
        raise RuntimeError("Some servos did not confirm torque. Keep the arm supported.")
    return True


def park_arm(args, bus, limits, observation, profiles):
    if hold_current_pose(bus):
        args.telemetry.stage("ready", "Engaged servo hold", "The arm was not holding, so it now holds its pose.")
        print("The arm was not holding. Engaging servo hold at the current pose.", flush=True)
    args.telemetry.stage("ready", "Preparing the arm", "Checking the return to observation.")
    opening, load = grip(bus)
    if opening <= args.close_limit + 2 or load >= CONTACT_LOAD:
        raise RuntimeError("Start with the gripper open and empty.")
    try:
        current = read_arm(bus)
        floor = max(tool_pose(profile[0])[0][2] for profile in profiles.values()) - 5
        if np.max(abs(current - observation)) <= 4.5:
            require_hover(bus, observation)
        elif tool_pose(current)[0][2] >= floor:
            print("Moving to the saved observation pose with the belt stopped.", flush=True)
            move_fixed(bus, observation, limits, floor, 6)
        else:
            print("Raising clear of the belt, then moving to the observation pose.", flush=True)
            try:
                move_to_observation(bus, observation, limits, profiles)
            except RuntimeError:
                clearance = min(max(tool_pose(profile[0])[0][2] for profile in profiles.values()) + 10,
                                tool_pose(observation)[0][2])
                start = tool_pose(current)[0]
                lift_mm = max(0.0, clearance - start[2])
                while True:
                    lifted = raised_pose(current, lift_mm, limits, "observation")
                    if np.max(np.abs(lifted - observation)) <= 4.5:
                        require_hover(bus, observation)
                        return
                    route = [tool_pose(lifted + f * (observation - lifted))[0][2]
                             for f in np.linspace(0, 1, 40)]
                    if min(route) >= clearance - 4:
                        break
                    lift_mm += 5
                    if lift_mm > 160:
                        raise RuntimeError("Could not find a clear route back to the observation pose.")
                move_fixed(bus, lifted, limits, min(start[2], clearance) - 2, 6)
                move_fixed(bus, observation, limits, clearance - 4, 6)
        print("The arm is at the observation pose.", flush=True)
    except BaseException:
        hold_position(bus, limits)
        raise


def run_arm_test(args, robot, observation, profiles):
    label = args.arm_only.casefold()
    if label not in profiles:
        raise ValueError(f"No taught pickup and bin for {args.arm_only}.")
    with arm_bus(args.port, robot) as (bus, limits):
        park_arm(args, bus, limits, observation, profiles)
        try:
            print(f"Running one arm-only pickup for {args.arm_only}.", flush=True)
            sort_object(args, bus, limits, observation, profiles[label], args.arm_only)
        except BaseException:
            hold_position(bus, limits)
            raise
    args.telemetry.stage("complete", f"{args.arm_only} sorted", "The arm is back at observation.")
    print(f"Dropped {args.arm_only} in its bin and returned to observation.", flush=True)


def run(args, calibration, robot, observation, profiles, pose_config):
    with conveyor_session(args) as belt, arm_bus(args.port, robot) as (bus, limits):
        belt.stop_and_wait()
        park_arm(args, bus, limits, observation, profiles)
        count = 0
        while not args.cycles or count < args.cycles:
            stopped_at = seek_item(args, belt)
            if args.ultrasonic_only:
                label, display, obj = classify_object(args, calibration, profiles, stopped_at)
                title = f"Classified {display}" if obj else f"Using {display} fallback"
                args.telemetry.stage("inspect", title, "Selected the taught pickup and bin.",
                                     destination=f"{display} bin",
                                     object={"label": display, "confidence": obj["confidence"]} if obj else None)
            else:
                obj = align_item(args, belt, calibration, pose_config, profiles, stopped_at)
                label = obj["label"].casefold()
                display = obj["label"]
                if label not in profiles:
                    raise RuntimeError(f"No taught pickup and bin for {obj['label']}.")
            confidence = "" if obj is None else f" ({obj['confidence']:.0%})"
            print(f"{display}{confidence}. Using its taught pickup and bin.", flush=True)
            try:
                require_hover(bus, observation)
                sort_object(args, bus, limits, observation, profiles[label], display)
            except BaseException:
                hold_position(bus, limits)
                raise
            count += 1
            args.telemetry.stage("complete", f"{display} sorted", "The arm is back at observation.")
            print(f"Dropped {display} in its bin and returned to observation.", flush=True)
            if not args.cycles or count < args.cycles:
                args.telemetry.stage("complete", "Conveyor waiting to restart",
                                     f"Holding the belt for {args.restart_delay:g} seconds after the arm parked.")
                print(f"Belt held for {args.restart_delay:g} seconds after the arm parked.", flush=True)
                sleep(args.restart_delay)
                if not args.ultrasonic_only:
                    wait_for_clear(args, calibration, monotonic())
                status = belt.set_running(False)
                if 0 < status.distance_mm < args.stop_mm:
                    raise RuntimeError("Ultrasonic sensor still sees an item after drop-off.")
                require_hover(bus, observation)


def run_with_dashboard(args, operation, *parameters):
    with dashboard_server(args.telemetry, args.dashboard, args.qbot):
        try:
            operation(args, *parameters)
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            args.telemetry.stage("fault", "Sorting stopped", str(error))
            print(f"Sorting stopped: {error}", flush=True)
            keep_dashboard_open(args)
            raise
        keep_dashboard_open(args)


def keep_dashboard_open(args):
    if not args.telemetry.enabled:
        return
    print("The run ended. Press Ctrl+C to close the dashboard.", flush=True)
    try:
        while True:
            sleep(.25)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("station", type=Path)
    region = parser.add_mutually_exclusive_group()
    region.add_argument("--camera-calibration", type=Path, help="Use only its calibrated square for presence")
    region.add_argument("--roi", nargs=4, type=int, metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"))
    parser.add_argument("--qbot", default="http://127.0.0.1:18000")
    parser.add_argument("--adb", default="adb")
    parser.add_argument("--conveyor", help="ADB serial number of the conveyor UNO Q")
    parser.add_argument("--stop-mm", type=int, default=DEFAULT_STOP_MM)
    parser.add_argument("--max-frame-age", type=float, default=4)
    parser.add_argument("--camera-wait", type=float, default=25)
    parser.add_argument("--pickup-wait", type=float, default=0,
                        help="Seconds to wait for the ultrasonic trigger (0 waits indefinitely)")
    parser.add_argument("--pose-config", type=Path, help="Taught camera placement configuration")
    parser.add_argument("--pickup-position-px", type=float, default=10)
    parser.add_argument("--pickup-angle-deg", type=float, default=12)
    parser.add_argument("--max-jogs", type=int, default=8)
    parser.add_argument("--dashboard", nargs="?", type=int, const=8765, metavar="PORT",
                        help="Show the live station dashboard (default port 8765)")
    parser.add_argument("--belt-mm-per-rev", type=float,
                        help="Measured belt travel per encoder revolution, for displaying mm/s")
    parser.add_argument("--angle-period", type=int, choices=(0, 90, 180), default=180,
                        help="Use 90 for a square box or 0 when rotation does not matter")
    parser.add_argument("--restart-delay", type=float, default=2,
                        help="Seconds to keep the belt stopped after the arm returns to observation")
    parser.add_argument("--ultrasonic-only", action="store_true",
                        help="Stop on ultrasound and use the camera label to choose a taught route")
    parser.add_argument("--classify-wait", type=float, default=15,
                        help="Seconds to classify the object when the camera decides the bin")
    parser.add_argument("--object", metavar="LABEL",
                        help="Taught object to sort when classification does not choose one")
    parser.add_argument("--cycles", type=int, default=0,
                        help="Number of items to sort (default: 0, keep sorting until Ctrl+C)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="Enable conveyor and arm movement")
    mode.add_argument("--arm-only", metavar="LABEL", help="Test one taught pickup with conveyor power off")
    mode.add_argument("--check-conveyor", action="store_true", help="Read the ultrasonic sensor with the belt stopped")
    mode.add_argument("--teach-empty", action="store_true", help="Capture the empty belt background and start pose teaching")
    mode.add_argument("--teach-target", metavar="LABEL", help="Record the desired box center and orientation")
    mode.add_argument("--teach-upstream", metavar="LABEL", help="Record the box upstream to learn the forward direction")
    add_motion_arguments(parser, fixed_poses=True)
    parser.set_defaults(carry_speed=30)
    args = parser.parse_args()
    if args.dashboard is not None and (not 1024 <= args.dashboard <= 65535 or not (args.execute or args.arm_only)):
        parser.error("Use --dashboard with --execute or --arm-only and a port between 1024 and 65535")
    if args.belt_mm_per_rev is not None and (not math.isfinite(args.belt_mm_per_rev) or args.belt_mm_per_rev <= 0):
        parser.error("Belt travel per encoder revolution must be positive")
    args.telemetry = Dashboard(args.dashboard is not None, args.belt_mm_per_rev)
    validate_motion_arguments(parser, args)
    if (args.execute or args.check_conveyor) and not args.conveyor:
        parser.error("Specify the conveyor UNO Q's ADB serial number")
    teach = args.teach_empty or args.teach_target or args.teach_upstream
    if (teach or (args.execute and not args.ultrasonic_only)) and not args.pose_config:
        parser.error("Specify --pose-config for camera teaching or an automatic run")
    if args.teach_empty and not args.roi:
        parser.error("Specify a camera search area with --roi when teaching the empty belt")
    if not (args.arm_only or args.check_conveyor or args.ultrasonic_only or args.pose_config
            or args.camera_calibration or args.roi):
        parser.error("Specify a pickup area with --camera-calibration or --roi")
    if (args.cycles < 0 or not 10 <= args.stop_mm <= 400
            or not 0 < args.max_frame_age <= 10 or not 0 < args.camera_wait <= 120
            or not math.isfinite(args.camera_wait)):
        parser.error("Check cycle count, ultrasonic threshold, and camera timing")
    if not 0 <= args.restart_delay <= 120:
        parser.error("Use a restart delay between 0 and 120 seconds")
    if not 0 <= args.pickup_wait <= 120:
        parser.error("Use a pickup wait between 0 and 120 seconds")
    if not 0 <= args.classify_wait <= 30:
        parser.error("Use a classification wait between 0 and 30 seconds")
    if args.ultrasonic_only and not args.execute:
        parser.error("Use --ultrasonic-only with --execute")
    if not (2 <= args.pickup_position_px <= 40 and 2 <= args.pickup_angle_deg <= 30 and 0 <= args.max_jogs <= 20):
        parser.error("Use a position tolerance of 2–40 px, an angle tolerance of 2–30 degrees, and 0–20 jogs")
    if args.roi and not (0 <= args.roi[0] < args.roi[2] and 0 <= args.roi[1] < args.roi[3]):
        parser.error("Use an ROI with left < right and top < bottom")
    try:
        pose_config = None
        if args.pose_config and not args.teach_empty and not (args.check_conveyor or args.arm_only):
            pose_config = load_pose_config(args.pose_config, ready=args.execute and not args.ultrasonic_only)
            saved_roi = pose_config[0]["roi"]
            if args.roi and args.roi != saved_roi:
                raise ValueError("Use the ROI saved during camera teaching, or teach a new empty image.")
            args.roi = saved_roi
        calibration = (json.loads(args.camera_calibration.read_text(encoding="utf-8"))
                       if args.camera_calibration else None)
        if teach:
            teach_pose(args, pose_config)
            return
        robot, observation, profiles = load_station(args.station)
        station_settings = json.loads(args.station.read_text(encoding="utf-8"))
        args.exact_pose_labels = {label.casefold() for label in station_settings.get("exact_pose_labels", [])}
        if args.exact_pose_labels - profiles.keys():
            raise ValueError("Exact grasp and bin poses require a taught object profile.")
        args.gripper_close_limits = {
            label.casefold(): limit for label, limit in station_settings.get("gripper_close_limits", {}).items()
        }
        if any(label not in profiles or not isinstance(limit, (int, float)) or not 1 <= limit <= 20
               for label, limit in args.gripper_close_limits.items()):
            raise ValueError("Object-specific gripper limits need a taught label and 1–20% opening.")
        if args.ultrasonic_only and args.object is not None:
            taught_object(args, profiles)
        if args.arm_only:
            run_with_dashboard(args, run_arm_test, robot, observation, profiles)
        elif args.check_conveyor:
            with conveyor_session(args) as belt:
                belt.stop_and_wait()
                status = belt.set_running(False)
                print(f"Conveyor stopped. Ultrasonic {status.distance_mm} mm (-1 means no echo).")
        elif args.execute:
            run_with_dashboard(args, run, calibration, robot, observation, profiles, pose_config)
        else:
            frame = camera_frame(args)
            if frame is None:
                raise RuntimeError("No fresh camera frame. Start live capture on the phone.")
            objects = (objects_at_pickup(args, frame, calibration)
                       if args.roi or calibration is not None else frame["objects"])
            for obj in objects:
                label = obj["label"]
                bin_status = "taught bin" if label.casefold() in profiles else "no taught bin"
                print(f"{label} ({obj['confidence']:.0%}), {bin_status}, box {obj['box_xyxy']}")
            print("Camera preview complete.")
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        parser.exit(1, f"Sorting stopped: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Sorting interrupted.\n")


if __name__ == "__main__":
    main()
