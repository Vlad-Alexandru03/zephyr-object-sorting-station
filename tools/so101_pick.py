import argparse
import json
import math
from contextlib import contextmanager
from pathlib import Path
from time import monotonic, sleep

import numpy as np


JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
# Joint origins from TheRobotStudio's SO-101 so101_new_calib.urdf, in meters/radians.
ORIGINS = (
    ((.0388353, 0, .0624), (math.pi, 0, -math.pi)),
    ((-.0303992, -.0182778, -.0542), (-math.pi / 2, -math.pi / 2, 0)),
    ((-.11257, -.028, 0), (0, 0, math.pi / 2)),
    ((-.1349, .0052, 0), (0, 0, -math.pi / 2)),
    ((0, -.0611, .0181), (math.pi / 2, .0486795, math.pi)),
    ((-.0079, -.000218121, -.0981274), (0, math.pi, 0)),
)


def rotation(angle, axis):
    result = np.eye(3)
    i, j = (axis + 1) % 3, (axis + 2) % 3
    result[i, i] = result[j, j] = math.cos(angle)
    result[i, j], result[j, i] = -math.sin(angle), math.sin(angle)
    return result


def tool_pose(joint_degrees):
    position, orientation = np.zeros(3), np.eye(3)
    for (offset, (roll, pitch, yaw)), angle in zip(ORIGINS, np.r_[np.deg2rad(joint_degrees), 0]):
        position += orientation @ offset
        orientation = orientation @ rotation(yaw, 2) @ rotation(pitch, 1) @ rotation(roll, 0)
        orientation = orientation @ rotation(angle, 2)
    return position * 1000, orientation


def lift_path(start, height, max_tilt_deg=6, max_lateral_mm=50):
    """Plan a smooth near-vertical lift that lets the wrist settle a little.

    The tool is solved for height only. The orientation is left to drift by up to
    max_tilt_deg, which keeps the joints from fighting each other. The returned
    path is checked for height, sideways travel, and tilt before it is used.
    """
    origin, attitude = tool_pose(start)
    target = origin + (0, 0, height)

    def solve(total_height):
        point = origin + (0, 0, total_height)
        angles = start.copy()
        for _ in range(200):
            def residual(q):
                return np.r_[tool_pose(q)[0] - point, .05 * (q - start)]

            error = residual(angles)
            if np.linalg.norm(error[:3]) < .02:
                break
            jacobian = np.column_stack([(residual(angles + step) - error) / .01
                                        for step in np.eye(5) * .01])
            correction = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            angles -= np.clip(correction, -2, 2)
        position, _ = tool_pose(angles)
        return angles, position

    top, top_position = solve(height)
    if np.linalg.norm(top_position - target) > .5:
        return vertical_path(start, height), "vertical"
    steps = max(1, math.ceil(abs(height) / 5))
    path = []
    for fraction in np.linspace(1 / steps, 1, steps):
        angles = start + fraction * (top - start)
        position, orientation = tool_pose(angles)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, orientation[:, 2] @ attitude[:, 2]))))
        lateral = float(np.linalg.norm(position[:2] - origin[:2]))
        if tilt > max_tilt_deg or lateral > max_lateral_mm:
            return vertical_path(start, height), "vertical"
        path.append((position, angles))
    return path, "tilt"


def vertical_path(start, height):
    """Raise the tool straight up while holding its orientation exactly."""
    origin, attitude = tool_pose(start)
    angles = start.copy()
    path = []
    for offset in np.linspace(0, height, max(1, math.ceil(abs(height) / 5)) + 1)[1:]:
        point = origin + (0, 0, offset)

        def residual(q):
            position, orientation = tool_pose(q)
            return np.r_[position - point, 100 * (orientation - attitude).ravel()]

        for _ in range(12):
            error = residual(angles)
            if np.linalg.norm(error) < .1:
                break
            jacobian = np.column_stack([(residual(angles + step) - error) / .01 for step in np.eye(5) * .01])
            correction = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            angles -= np.clip(correction, -4, 4)
        if np.linalg.norm(residual(angles)) >= .1:
            raise RuntimeError(f'No vertical path solution at {offset:.1f} mm')
    origin, attitude = tool_pose(start)
    angles = start.copy()
    path = []
    for offset in np.linspace(0, height, max(1, math.ceil(abs(height) / 5)) + 1)[1:]:
        point = origin + (0, 0, offset)

        def residual(q):
            position, orientation = tool_pose(q)
            return np.r_[position - point, 100 * (orientation - attitude).ravel()]

        for _ in range(12):
            error = residual(angles)
            if np.linalg.norm(error) < .1:
                break
            jacobian = np.column_stack([(residual(angles + step) - error) / .01 for step in np.eye(5) * .01])
            correction = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            angles -= np.clip(correction, -4, 4)
        if np.linalg.norm(residual(angles)) >= .1:
            raise RuntimeError(f'No vertical path solution at {offset:.1f} mm')
        path.append((point, angles.copy()))
    return path


def joint_values(pose):
    return np.array([pose[joint + '.pos'] for joint in JOINTS], dtype=float)


def move_arm(bus, target, limits, monitor=lambda: None, peak_speed=12, preserve_hold=False,
             tracking_tolerance_deg=7, check_position=None, waypoints=(),
             advance_limit_deg=None, relative_to=None):
    start = read_arm(bus)
    if preserve_hold:
        goals = bus.sync_read('Goal_Position', list(JOINTS), num_retry=3)
        held = np.array([goals[joint] for joint in JOINTS])
        if not np.isfinite(held).all() or np.any(abs(held) > limits) or np.max(abs(held - start)) > 4.5:
            raise RuntimeError('The current servo goals are too far from the measured holding pose')
        start = held
    trim = np.zeros_like(start) if relative_to is None else start - relative_to
    route = np.vstack((start, *(waypoint + trim for waypoint in waypoints), target + trim))
    segments = np.diff(route, axis=0)
    if np.any(~np.isfinite(route[1:])) or np.any(abs(route[1:]) > limits):
        raise RuntimeError('A motion target exceeds the calibrated joint range')
    if (waypoints or relative_to is not None) and check_position is not None:
        for beginning, delta in zip(route, segments):
            for fraction in np.linspace(0, 1, 12):
                check_position(beginning + fraction * delta - trim)
    duration = max(.45, 1.875 * len(segments) * np.max(abs(segments)) / peak_speed)
    started = monotonic()
    last_sample = last_advance = started
    elapsed = 0.0
    command = start
    while True:
        monitor()
        actual = read_arm(bus)
        if check_position is not None:
            check_position(actual)
        errors = abs(actual - command)
        slow_joint = int(np.argmax(errors))
        lag = errors[slow_joint]
        if lag > tracking_tolerance_deg:
            raise RuntimeError(f'Arm tracking error on {JOINTS[slow_joint]}: '
                               f'{lag:.1f} degrees, limit {tracking_tolerance_deg:g}')
        now = monotonic()
        if advance_limit_deg is None:
            elapsed = now - started
        elif lag < advance_limit_deg:
            rate = max(.25, min(1, (advance_limit_deg - lag) / (advance_limit_deg - 1)))
            elapsed += rate * min(now - last_sample, .04)
            last_advance = now
        elif now - last_advance > 2:
            raise RuntimeError(f'Lift stalled waiting for {JOINTS[slow_joint]} '
                               f'({lag:.2f} degrees behind its target, '
                               f'advance limit {advance_limit_deg:g} degrees)')
        last_sample = now
        progress = min(elapsed / duration, 1)
        blend = progress**3 * (10 + progress * (-15 + 6 * progress))
        path_progress = blend * len(segments)
        segment = min(int(path_progress), len(segments) - 1)
        command = route[segment] + (path_progress - segment) * segments[segment]
        send_arm(bus, route[-1] if progress == 1 else command, limits)
        if progress == 1:
            return route[-1]
        sleep(.04)


def send_arm(bus, angles, limits):
    if np.any(abs(angles) > limits):
        raise RuntimeError('A motion target exceeds the calibrated joint range')
    bus.sync_write('Goal_Position', dict(zip(JOINTS, angles.tolist())), num_retry=3)


def read_arm(bus):
    try:
        pose = bus.sync_read('Present_Position', list(JOINTS), num_retry=3)
    except ConnectionError:
        pose = {joint: bus.read('Present_Position', joint, num_retry=3) for joint in JOINTS}
    return np.array([pose[joint] for joint in JOINTS])


def grip(bus):
    opening = bus.read('Present_Position', 'gripper', num_retry=3)
    load = abs(bus.read('Present_Load', 'gripper', normalize=False, num_retry=3))
    return opening, load


def set_gripper(bus, position):
    bus.write('Goal_Position', 'gripper', position, num_retry=3)


CONTACT_LOAD = 45


class GripMiss(RuntimeError):
    """The jaws reached the closing limit without confirming contact."""


class GripHold:
    def __init__(self, bus, goal, contact_opening, close_limit, open_position, target_load, load_band):
        self.bus = bus
        self.goal = goal
        self.contact_opening = contact_opening
        self.minimum_goal = max(close_limit, min(goal, contact_opening - 4))
        self.maximum_goal = min(open_position, contact_opening + .5)
        self.low = target_load - load_band
        self.high = target_load + load_band
        self.next_sample = 0
        self.previous_state = None
        self.consecutive = 0
        self.ready = False

    def update(self):
        now = monotonic()
        if now < self.next_sample:
            return self.ready
        self.next_sample = now + .12
        opening, load = grip(self.bus)
        if opening < self.contact_opening - 2.5 or (load < CONTACT_LOAD and opening <= self.goal + .5):
            state = 'slip'
        elif load < self.low:
            state = 'low'
        elif load > self.high:
            state = 'high'
        else:
            state = 'in_band'

        self.consecutive = self.consecutive + 1 if state == self.previous_state else 1
        self.previous_state = state
        self.ready = state == 'in_band' and self.consecutive >= 3
        if self.consecutive < 3:
            return False
        if state == 'slip':
            raise RuntimeError(f'Possible loss of grip at {opening:.1f}% opening, load {load}')
        if self.ready:
            return True

        delta = -.5 if state == 'low' else .5
        goal = max(self.minimum_goal, min(self.maximum_goal, self.goal + delta))
        if goal == self.goal:
            raise RuntimeError(f'Grip load {load} is outside {self.low}–{self.high} at the jaw adjustment limit '
                               f'(goal {self.goal:.1f}%, measured {opening:.1f}% open)')
        set_gripper(self.bus, goal)
        self.goal = goal
        self.consecutive = 0
        self.next_sample = now + .25
        return False


def close_gripper(bus, open_position, close_limit, torque_limit, target_load, load_band):
    bus.write('Torque_Limit', 'gripper', torque_limit, normalize=False, num_retry=3)
    steps = math.ceil((open_position - close_limit) / 1.5)
    for goal in np.linspace(open_position, close_limit, steps + 1)[1:]:
        goal = float(goal)
        set_gripper(bus, goal)
        sleep(.3)
        openings = []
        for _ in range(3):
            opening, load = grip(bus)
            if opening < goal + .5 or load < CONTACT_LOAD:
                break
            openings.append(opening)
            sleep(.08)
        if len(openings) == 3:
            hold = GripHold(bus, goal, min(openings), close_limit, open_position, target_load, load_band)
            deadline = monotonic() + 5
            while monotonic() < deadline:
                if hold.update():
                    return hold
                sleep(.06)
            raise RuntimeError('Grip load did not stabilize before the lift')
    raise GripMiss('Reached the jaw closing limit without confirmed contact')


def settle_tool(bus, target, point, floor, limits, monitor, validate_command=None,
                lateral_tolerance_mm=4, height_tolerance_mm=2, initial_command=None):
    command = target.copy() if initial_command is None else initial_command.copy()
    for _ in range(10):
        sleep(.2)
        monitor()
        actual = read_arm(bus)
        position = tool_pose(actual)[0]
        difference = position - point
        lateral_error = np.linalg.norm(difference[:2])
        if lateral_error <= lateral_tolerance_mm and abs(difference[2]) <= height_tolerance_mm:
            return position
        if lateral_error > 12 or position[2] < floor:
            raise RuntimeError(f'Gripper drifted outside the correction corridor '
                               f'({lateral_error:.1f} mm sideways, {difference[2]:+.1f} mm vertically)')
        command = np.clip(command + np.clip(.5 * (target - actual), -.8, .8), target - 5, target + 5)
        if validate_command is not None:
            validate_command(actual, command)
        send_arm(bus, command, limits)
    raise RuntimeError(f'Could not verify the tool position: lateral error {lateral_error:.1f} mm, '
                       f'height error {difference[2]:+.1f} mm.')


def goals_are_measured(bus, tolerance):
    held = bus.sync_read('Goal_Position', list(JOINTS), num_retry=3)
    goals = np.array([held[joint] for joint in JOINTS])
    return np.isfinite(goals).all() and np.max(abs(goals - read_arm(bus))) <= tolerance


def settle_goals_to_measured(bus, limits, tolerance=.5, wait=1.5):
    """Make the servo goals match where the arm actually is.

    After closing the jaws the goals can sit a few degrees away from the measured
    pose. Lifting from those goals would first drag the tool sideways, so the
    goals are re-zeroed here and the arm is left to settle before the lift.
    """
    if goals_are_measured(bus, tolerance):
        return read_arm(bus)
    measured = read_arm(bus)
    send_arm(bus, measured, limits)
    deadline = monotonic() + wait
    while monotonic() < deadline:
        if goals_are_measured(bus, tolerance):
            return read_arm(bus)
        sleep(.04)
    return read_arm(bus)


def follow_lift(bus, height, limits, check_grip, peak_speed=12):
    """Raise the tool by `height`. This only moves the arm; it does not grade it.

    The carry to the bin lifts further anyway, so a lift that falls short is not
    treated as a fault.
    """
    start = settle_goals_to_measured(bus, limits)
    origin = tool_pose(start)[0]
    path, mode = lift_path(start, height)
    target = path[-1][1]
    if mode == "tilt":
        print(f'Lifting {height:.0f} mm with up to 6 degrees of tool tilt.', flush=True)
    move_arm(bus, target, limits, check_grip, peak_speed=peak_speed,
             waypoints=tuple(angles for _, angles in path[:-1]),
             tracking_tolerance_deg=12)
    position = tool_pose(read_arm(bus))[0]
    print(f'Lift height now {position[2] - origin[2]:.1f} mm, '
          f'{np.linalg.norm(position[:2] - origin[:2]):.1f} mm sideways.', flush=True)
    # Accept wherever the arm actually got to, so the carry starts from reality
    # instead of from a goal the servos never reached.
    settle_goals_to_measured(bus, limits)


@contextmanager
def arm_bus(port, robot_id):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=port, id=robot_id, use_degrees=True))
    bus = robot.bus
    for attempt in range(4):
        try:
            bus.connect()
            break
        except (ConnectionError, RuntimeError):
            if attempt == 3:
                raise
            sleep(.4)
    try:
        for attempt in range(3):
            try:
                calibrated = robot.is_calibrated
                break
            except ConnectionError:
                if attempt == 2:
                    raise
                sleep(.1)
        if not calibrated:
            raise RuntimeError('Calibration mismatch')
        limits = np.array([(robot.calibration[j].range_max - robot.calibration[j].range_min) * 180 / 4095
                           for j in JOINTS])
        yield bus, limits
    finally:
        bus.disconnect(disable_torque=False)


def require_hover(bus, above_pickup):
    if np.max(abs(read_arm(bus) - above_pickup)) > 4.5:
        raise RuntimeError('Start with the arm already holding above the pickup point')
    if set(bus.sync_read('Torque_Enable', normalize=False, num_retry=3).values()) != {1}:
        raise RuntimeError('All servos must already be holding')


def hold_position(bus, limits):
    try:
        send_arm(bus, read_arm(bus), limits)
    except Exception as error:
        print(f'Could not command a hold: {error}', flush=True)


def pick_and_place(args, bus, limits, grasp_path, above_bin, open_position, approach=None,
                   carry_floor=None):
    set_gripper(bus, open_position)
    sleep(.6)
    if abs(grip(bus)[0] - open_position) > 2:
        raise RuntimeError('Gripper did not open')
    print('Descending to the grasp height', flush=True)
    if approach is None:
        for point, goal in grasp_path:
            move_arm(bus, goal, limits)
            settle_tool(bus, goal, point, point[2] - 8, limits, lambda: None)
    else:
        approach(grasp_path)

    hold = close_gripper(bus, open_position, args.close_limit, args.grip_torque_limit,
                         args.grip_load, args.grip_load_band)

    print('Grip confirmed. Lifting before the swing.', flush=True)
    follow_lift(bus, args.lift_mm, limits, hold.update)
    if carry_floor is not None:
        actual = read_arm(bus)
        for fraction in np.linspace(0, 1, 20):
            point = tool_pose(actual + fraction * (above_bin - actual))[0]
            if point[2] < carry_floor:
                raise RuntimeError('The measured lift does not clear the route to the bin')

    def monitor_carry():
        hold.update()
        if carry_floor is not None and tool_pose(read_arm(bus))[0][2] < carry_floor - 3:
            raise RuntimeError('The arm dropped below carrying clearance')

    print('Carrying to the raised bin waypoint', flush=True)
    move_arm(bus, above_bin, limits, monitor_carry, peak_speed=args.carry_speed)
    for _ in range(5):
        sleep(.12)
        monitor_carry()
    actual = read_arm(bus)
    if np.max(abs(actual - above_bin)) > 4:
        raise RuntimeError('Arm did not reach the bin waypoint. Gripper remains closed.')
    set_gripper(bus, open_position)
    sleep(.6)
    if abs(grip(bus)[0] - open_position) > 2:
        raise RuntimeError('Could not verify the gripper opened over the bin')
    print('Gripper opened over the bin. Arm is holding position.', flush=True)


def execute(args, robot_id, above_pickup, grasp, above_bin, open_position):
    with arm_bus(args.port, robot_id) as (bus, limits):
        require_hover(bus, above_pickup)
        try:
            pick_and_place(args, bus, limits, [(tool_pose(grasp)[0], grasp)], above_bin, open_position)
        except BaseException:
            hold_position(bus, limits)
            raise


def add_motion_arguments(parser, fixed_poses=False):
    parser.add_argument('--port', default='COM7')
    if fixed_poses:
        parser.set_defaults(grasp_offset_mm=0, lift_mm=30)
    else:
        parser.add_argument('--grasp-offset-mm', type=float, default=5)
        parser.add_argument('--lift-mm', type=float, default=30)
    parser.add_argument('--carry-speed', type=float, default=12, help='Peak commanded joint speed during the carry in degrees per second.')
    parser.add_argument('--close-limit', type=float, default=3, help='Minimum gripper opening in percent.')
    parser.add_argument('--grip-torque-limit', type=int, default=190, help='Feetech torque-limit units.')
    parser.add_argument('--grip-load', type=int, default=170, help='Target Present_Load in servo units.')
    parser.add_argument('--grip-load-band', type=int, default=15, help='Allowed deviation from the target load.')


def validate_motion_arguments(parser, args):
    if not 0 <= args.grasp_offset_mm <= 20 or not 10 <= args.lift_mm <= 70:
        parser.error('Use a grasp offset of 0–20 mm and a lift of 10–70 mm')
    if not 5 <= args.carry_speed <= 30:
        parser.error('Use a carry speed of 5–30 degrees per second')
    if not 3 <= args.close_limit <= 20 or not 100 <= args.grip_torque_limit <= 200:
        parser.error('Use a closing limit of 3–20% and a torque limit of 100–200')
    if not 5 <= args.grip_load_band <= 40 or not (
        CONTACT_LOAD < args.grip_load - args.grip_load_band < args.grip_load + args.grip_load_band
        <= args.grip_torque_limit
    ):
        parser.error('Use a load band of 5–40 units between the contact threshold and torque limit')


def main():
    parser = argparse.ArgumentParser(description='Plan the SO-101 pick, vertical lift, and carry.')
    parser.add_argument('waypoints', type=Path)
    add_motion_arguments(parser)
    parser.add_argument('--execute', action='store_true', help='Move an already holding arm at above_pickup.')
    args = parser.parse_args()
    validate_motion_arguments(parser, args)
    data = json.loads(args.waypoints.read_text(encoding='utf-8'))
    poses = data['waypoints']
    grasp = joint_values(poses['grasp'])
    if args.grasp_offset_mm:
        grasp = vertical_path(grasp, args.grasp_offset_mm)[-1][1]
    lift = vertical_path(grasp, args.lift_mm)
    print(f'Grasp offset: +{args.grasp_offset_mm:g} mm')
    print(f'Vertical lift: {args.lift_mm:g} mm in {len(lift)} steps, then swing to the bin')
    print(f'Grip load band: {args.grip_load - args.grip_load_band}–{args.grip_load + args.grip_load_band} servo units')
    if args.execute:
        opening = max(poses['grasp']['gripper.pos'], poses['above_pickup']['gripper.pos'])
        if opening <= args.close_limit:
            parser.error('The taught opening must be greater than the closing limit')
        execute(args, data['robot_id'], joint_values(poses['above_pickup']), grasp,
                joint_values(poses['above_bin']), opening)


if __name__ == '__main__':
    main()
