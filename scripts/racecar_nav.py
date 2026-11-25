#!/usr/bin/env python3
"""
racecar nav: send the car to a pose on the loaded map with Nav2.

Usage:
    racecar nav goal X Y HEADING [--no-wait]   x, y in meters, heading in degrees (map frame)
    racecar nav move DIST [--no-wait]          DIST meters ahead of the car (negative: behind)
    racecar nav cancel                         stop the goal or mission that is running
    racecar nav status                         whether Nav2 is up and who drives
    racecar nav mission list [--map NAME]      saved Planner missions of the loaded map
    racecar nav mission run NAME               run a saved mission in the Planner
    racecar nav mission stop                   stop the mission that is running

Nav2 runs with localization (`racecar map load NAME`). The car moves only
while RB is held, or under the 8080 safety override. Goals go through the
navigator (nav_server_node.py): one is refused while something else drives. A
goal reports the distance left until it ends; Ctrl+C cancels it. Missions run
in the Planner (scripts/planner_app.py), which records them; `mission run`
starts it when it is not running. They are saved from its page.
"""

from __future__ import annotations

import argparse
import json
import math
import os
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
NAME = 'racecar nav'
NOT_RUNNING = 'Nav2 is not running. Load a map first: racecar map load NAME'
ENDINGS = {
    'failed': 'Could not reach the goal.',
    'stalled': 'Car stalled.',
    'stopped': 'Stopped before the goal.',
}
NOTES = {'relocalizing': 'Localization lost.', 'paused': 'Paused.', 'running': 'Driving.'}


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
    sub.add_parser('cancel', help='stop the goal or mission that is running')
    sub.add_parser('status', help='whether Nav2 is up and who drives')
    mission = sub.add_parser('mission', help='saved Planner missions')
    msub = mission.add_subparsers(dest='mission_command', required=True)
    listed = msub.add_parser('list', help='saved missions of the loaded map')
    listed.add_argument('--map', help='another saved map')
    run = msub.add_parser('run', help='run a saved mission in the Planner')
    run.add_argument('name')
    msub.add_parser('stop', help='stop the mission that is running')
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


def connect(node: Any) -> Any:
    """Return the link to the navigator, or None while it is not running."""
    import rclpy
    from racecar_neo_ros2_driver.nav_link import NavLink

    link = NavLink(node)
    deadline = time.monotonic() + SERVER_WAIT_SEC
    while not link.alive and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return link if link.alive else None


def send(node: Any, link: Any, verb: str, **fields: Any) -> tuple[bool, str]:
    import rclpy

    return link.command(verb, lambda sec: rclpy.spin_once(node, timeout_sec=sec), **fields)


def run_goal(node: Any, x: float, y: float, heading: float, wait: bool) -> int:
    import rclpy
    from racecar_neo_ros2_driver import missions
    from racecar_neo_ros2_driver.nav_runner import NO_MAP

    try:
        mission = missions.parse({'waypoints': [{'x': x, 'y': y, 'heading_deg': heading}]})
    except ValueError as exc:
        print(f'Not a goal: {exc}', file=sys.stderr)
        return 2
    link = connect(node)
    if link is None:
        print(NOT_RUNNING, file=sys.stderr)
        return 2
    owner = f'cli-{os.getpid()}'
    ok, message = send(node, link, 'start', owner=owner, name=NAME, mission=mission)
    if not ok:
        print(NOT_RUNNING if message == NO_MAP else message, file=sys.stderr)
        return 2 if message == NO_MAP else 1
    print(f'Driving to x {x:.2f} m, y {y:.2f} m, heading {heading:.0f} deg. Hold RB.')
    if not wait:
        return 0
    last, shown = time.monotonic(), 'running'
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            view = link.view()
            if not link.alive or view['owner'] != owner:
                print(ENDINGS['stopped'])
                return 1
            state = view['state']
            if state == 'arrived':
                print('Arrived.')
                return 0
            if state in ENDINGS:
                print(ENDINGS[state], view['message'])
                return 1
            if state != shown and state in NOTES:
                print(f'  {NOTES[state]}')
                shown = state
            if view['left'] is not None and time.monotonic() - last >= REPORT_SEC:
                print(f'  {view["left"]:.2f} m left')
                last = time.monotonic()
    except KeyboardInterrupt:
        if link.view()['owner'] == owner:
            send(node, link, 'stop')
        print('Canceled.')
        return 130


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


def run_stop(node: Any, done: str, none: str) -> int:
    """Stop what the navigator runs, and any goal sent to Nav2 past it."""
    import rclpy
    from action_msgs.srv import CancelGoal

    link = connect(node)
    running = link is not None and link.underway()
    ok = True
    if running:
        ok, message = send(node, link, 'stop')
        if not ok:
            print(message, file=sys.stderr)
    client = node.create_client(CancelGoal, f'{ACTION}/_action/cancel_goal')
    if not client.wait_for_service(timeout_sec=1.0):
        if link is None:
            print('Nav2 is not running.', file=sys.stderr)
            return 2
    else:
        # A zero goal id and stamp cancel every goal.
        future = client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if future.done() and future.result():
            running = running or bool(future.result().goals_canceling)
    if not ok:
        return 1
    print(done if running else none)
    return 0


def describe(view: dict[str, Any]) -> str:
    """Return what the navigator does, for status."""
    name = view['name']
    return {
        'running': f'{name} is driving',
        'dwelling': f'{name} is waiting at a waypoint',
        'relocalizing': f'{name} is driving, localization lost',
        'paused': f'{name} is paused',
    }.get(view['state'], 'no goal')


def run_status(node: Any) -> int:
    link = connect(node)
    if link is None or not link.view()['ready']:
        print('Nav2: not running')
        return 1
    print(f'Nav2: running, {describe(link.view())}')
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse(sys.argv[1:] if argv is None else argv)
    if args.command == 'mission' and args.mission_command != 'stop':
        return run_mission(args)
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # Ctrl+C must reach run_goal with ROS still up, to stop the car.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
            return run_stop(node, 'Canceled.', 'No goal was running.')
        if args.command == 'mission':
            return run_stop(node, 'Stopped.', 'No mission was running.')
        return run_status(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
