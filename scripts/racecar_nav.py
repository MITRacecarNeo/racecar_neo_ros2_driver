#!/usr/bin/env python3
"""
racecar nav: send the car to a pose on the loaded map with Nav2.

Usage:
    racecar nav goal X Y HEADING [--no-wait]   x, y in meters, heading in degrees (map frame)
    racecar nav move DIST [--no-wait]          DIST meters ahead of the car (negative: behind)
    racecar nav cancel                         stop the goal that is running
    racecar nav status                         whether Nav2 is up and has a goal
    racecar nav mission list [--map NAME]      saved Planner missions of the loaded map
    racecar nav mission run NAME               run a saved mission in the Planner
    racecar nav mission stop                   stop the Planner's mission

Nav2 runs with localization (`racecar map load NAME`). The car moves only
while RB is held, or under the 8080 safety override. A goal reports the
distance left until it ends; Ctrl+C cancels it. Missions run in the Planner
(scripts/planner_app.py), which `mission run` starts when it is not running;
they are saved from its page.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from typing import Any
import urllib.error
import urllib.request

ACTION = 'navigate_to_pose'
SERVER_WAIT_SEC = 5.0
REPORT_SEC = 2.0
PLANNER_WAIT_SEC = 20.0


def quaternion_z_w(heading_deg: float) -> tuple[float, float]:
    """Return the (z, w) of a yaw-only quaternion for a heading in degrees."""
    half = math.radians(heading_deg) / 2.0
    return math.sin(half), math.cos(half)


def parse(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog='racecar nav', description=__doc__.split('\n\n')[0])
    sub = p.add_subparsers(dest='command', required=True)
    goal = sub.add_parser('goal', help='drive to x, y (m) facing heading (deg), map frame')
    goal.add_argument('x', type=float)
    goal.add_argument('y', type=float)
    goal.add_argument('heading', type=float)
    goal.add_argument('--no-wait', action='store_true', help='send the goal and return')
    move = sub.add_parser(
        'move', help='drive DIST m ahead, keeping the heading (negative: behind)'
    )
    move.add_argument('dist', type=float)
    move.add_argument('--no-wait', action='store_true', help='send the goal and return')
    sub.add_parser('cancel', help='stop the goal that is running')
    sub.add_parser('status', help='whether Nav2 is up and has a goal')
    mission = sub.add_parser('mission', help='saved Planner missions')
    msub = mission.add_subparsers(dest='mission_command', required=True)
    listed = msub.add_parser('list', help='saved missions of the loaded map')
    listed.add_argument('--map', help='another saved map')
    run = msub.add_parser('run', help='run a saved mission in the Planner')
    run.add_argument('name')
    msub.add_parser('stop', help="stop the Planner's mission")
    return p.parse_args(argv)


# -- missions: run by the Planner (scripts/planner_app.py) --


def loaded_map() -> str:
    """Return the map localization runs on, from the autonomy mode file; '' if none."""
    from map_control import read_mode_file

    values = read_mode_file()
    return values.get('AUTONOMY_MAP', '') if values.get('AUTONOMY_MODE') == 'localization' else ''


def planner_post(path: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    """POST to the Planner on this car; (0, {}) when it does not answer."""
    from racecar_neo_ros2_driver import apps

    request = urllib.request.Request(
        f'http://127.0.0.1:{apps.port("planner")}/{path}',
        data=json.dumps(body).encode(), method='POST',
        headers={'Content-Type': 'application/json'},
    )  # fmt: skip
    try:
        with urllib.request.urlopen(request, timeout=10) as res:
            return res.status, json.loads(res.read() or b'{}')
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b'{}')
    except (urllib.error.URLError, OSError):
        return 0, {}


def planner_answers() -> bool:
    from racecar_neo_ros2_driver import apps

    try:
        url = f'http://127.0.0.1:{apps.port("planner")}/api/missions'
        with urllib.request.urlopen(url, timeout=2) as res:
            return res.status == 200
    except (urllib.error.URLError, OSError):
        return False


def start_planner() -> bool:
    """Start the Planner unit if it is not running; True once it answers."""
    from racecar_neo_ros2_driver import apps, units

    unit = units.service(apps.app('planner')['unit'])
    subprocess.run([*units.systemctl_argv(), '--no-ask-password', 'start', unit],
                   capture_output=True, check=False)  # fmt: skip
    deadline = time.monotonic() + PLANNER_WAIT_SEC
    while time.monotonic() < deadline:
        if planner_answers():
            return True
        time.sleep(0.5)
    return False


def run_mission(args: argparse.Namespace) -> int:
    from racecar_neo_ros2_driver import missions

    if args.mission_command == 'list':
        name = args.map or loaded_map()
        if not name:
            print('No map loaded. Load one (racecar map load NAME) or name one: --map NAME',
                  file=sys.stderr)  # fmt: skip
            return 2
        found = missions.listing(name)
        if not found:
            print(f'No saved missions for {name}.')
        for m in found:
            n = m['loops']
            loops = 'until Stop' if n == 0 else f'{n} loop' if n == 1 else f'{n} loops'
            count = f"{m['waypoints']} waypoint" + ('s' if m['waypoints'] != 1 else '')
            print(f"{m['name']:<24} {count:<14} {loops:<12} {m['saved']}")
        return 0
    if args.mission_command == 'stop':
        code, _ = planner_post('api/control', {'verb': 'stop'})
        print('Stopped.' if code == 200 else 'The Planner is not running.')
        return 0 if code == 200 else 1
    if not loaded_map():
        print('No map loaded. Load one first: racecar map load NAME', file=sys.stderr)
        return 2
    if not start_planner():
        print('The Planner did not start: racecar service status planner', file=sys.stderr)
        return 1
    code, reply = planner_post('api/run_saved', {'name': args.name})
    if code != 200:
        print(reply.get('message') or 'Could not start the mission.', file=sys.stderr)
        return 1
    print(f'Running {args.name}. Hold RB. Stop: racecar nav mission stop')
    return 0


def run_goal(node: Any, x: float, y: float, heading: float, wait: bool) -> int:
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    client = ActionClient(node, NavigateToPose, ACTION)
    if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
        print('Nav2 is not running. Load a map first: racecar map load NAME', file=sys.stderr)
        return 2
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quaternion_z_w(heading)
    left = {'m': None}

    def feedback(msg: Any) -> None:
        left['m'] = msg.feedback.distance_remaining

    sent = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, sent)
    handle = sent.result()
    if handle is None or not handle.accepted:
        print('Nav2 refused the goal.', file=sys.stderr)
        return 1
    print(f'Driving to x {x:.2f} m, y {y:.2f} m, heading {heading:.0f} deg. Hold RB.')
    if not wait:
        return 0
    result = handle.get_result_async()
    last = 0.0
    try:
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            if left['m'] is not None and time.monotonic() - last >= REPORT_SEC:
                print(f'  {left["m"]:.2f} m left')
                last = time.monotonic()
    except KeyboardInterrupt:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        print('Canceled.')
        return 130
    from action_msgs.msg import GoalStatus

    status = result.result().status
    if status == GoalStatus.STATUS_SUCCEEDED:
        print('Arrived.')
        return 0
    print(
        'Stopped before the goal.'
        if status == GoalStatus.STATUS_CANCELED
        else 'Could not reach the goal.'
    )
    return 1


def ahead_of(x: float, y: float, heading_deg: float, dist: float) -> tuple[float, float]:
    """Return the point `dist` meters along the heading from (x, y)."""
    rad = math.radians(heading_deg)
    return x + dist * math.cos(rad), y + dist * math.sin(rad)


def car_pose(node: Any) -> tuple[float, float, float] | None:
    """Return the car's (x, y, heading in degrees) on the map, or None."""
    import rclpy
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener

    buf = Buffer()
    TransformListener(buf, node)
    deadline = time.monotonic() + SERVER_WAIT_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=0.2))
        except Exception:  # noqa: BLE001  not there yet; try until the deadline
            continue
        t = tf.transform
        heading = math.degrees(2 * math.atan2(t.rotation.z, t.rotation.w))
        return t.translation.x, t.translation.y, heading
    return None


def run_cancel(node: Any) -> int:
    import rclpy
    from action_msgs.srv import CancelGoal

    client = node.create_client(CancelGoal, f'{ACTION}/_action/cancel_goal')
    if not client.wait_for_service(timeout_sec=SERVER_WAIT_SEC):
        print('Nav2 is not running.', file=sys.stderr)
        return 2
    # A zero goal id and stamp cancel every goal.
    future = client.call_async(CancelGoal.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    canceling = len(future.result().goals_canceling) if future.result() else 0
    print('Canceled.' if canceling else 'No goal was running.')
    return 0


def run_status(node: Any) -> int:
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile

    if not ActionClient(node, NavigateToPose, ACTION).wait_for_server(timeout_sec=SERVER_WAIT_SEC):
        print('Nav2: not running')
        return 1
    statuses: list[Any] = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(
        GoalStatusArray, f'{ACTION}/_action/status', lambda m: statuses.append(m), qos
    )
    deadline = time.monotonic() + 1.0
    while not statuses and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
    running = statuses and any(s.status in active for s in statuses[-1].status_list)
    print('Nav2: running, ' + ('driving to a goal' if running else 'no goal'))
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse(sys.argv[1:] if argv is None else argv)
    if args.command == 'mission':
        return run_mission(args)
    import rclpy

    rclpy.init()
    node = rclpy.create_node('racecar_nav')
    try:
        if args.command == 'goal':
            return run_goal(node, args.x, args.y, args.heading, not args.no_wait)
        if args.command == 'move':
            pose = car_pose(node)
            if pose is None:
                print('No car pose on the map. Load a map first: racecar map load NAME',
                      file=sys.stderr)  # fmt: skip
                return 2
            x, y = ahead_of(pose[0], pose[1], pose[2], args.dist)
            return run_goal(node, x, y, pose[2], not args.no_wait)
        if args.command == 'cancel':
            return run_cancel(node)
        return run_status(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
