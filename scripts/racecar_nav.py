#!/usr/bin/env python3
"""
racecar nav: send the car to a pose on the loaded map with Nav2.

Usage:
    racecar nav goal X Y HEADING [--no-wait]   x, y in meters, heading in degrees (map frame)
    racecar nav move DIST [--no-wait]          DIST meters ahead of the car (negative: behind)
    racecar nav cancel                         stop the goal that is running
    racecar nav status                         whether Nav2 is up and has a goal

Nav2 runs with localization (`racecar map load NAME`). The car moves only
while RB is held, or under the 8080 safety override. A goal reports the
distance left until it ends; Ctrl+C cancels it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

ACTION = 'navigate_to_pose'
SERVER_WAIT_SEC = 5.0
REPORT_SEC = 2.0


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
    return p.parse_args(argv)


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
