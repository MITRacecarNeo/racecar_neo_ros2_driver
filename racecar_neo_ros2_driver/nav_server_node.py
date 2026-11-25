"""
The navigator: one mission or goal at a time, for every front end.

The Planner, `racecar nav` and student programs send commands on /nav/command
and read /nav/state (nav_link.py has the topics). nav_runner.py decides; this
node sends the waypoints to Nav2's navigate_to_pose, sets /speed_limit, and
feeds it the car's pose (/autonomy/pose), localization (/autonomy/status) and
stall protection (/pit/stall). It starts with Nav2 (launch/autonomy.launch.py)
and holds the drive lease from a mission's start to its end; nav_bridge_node
drives under it.
"""

from __future__ import annotations

from collections import OrderedDict
import json
import math
import time
from typing import Any

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from racecar_neo_ros2_driver import nav_link
from racecar_neo_ros2_driver.geometry import yaw_of
from racecar_neo_ros2_driver.nav_runner import Action, BAD_MISSION, Commander, Pose
from racecar_neo_ros2_driver.qos import LATCHED
from racecar_neo_ros2_driver.spin import run_node
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

ACTION = 'navigate_to_pose'
TICK_SEC = 0.1
STATE_EVERY = 2  # ticks
ANSWERS_KEPT = 16


class NavServerNode(Node):
    def __init__(self) -> None:
        super().__init__('nav_server_node')
        self.commander = Commander()
        self._pose: Pose | None = None
        self._localized: bool | None = None
        self._stalled = False
        self._handle: Any = None
        self._answers: OrderedDict[str, tuple[bool, str]] = OrderedDict()
        self._ticks = 0
        # A page tells missions apart by their version, also across a restart.
        self.commander.run.version = int(time.time())
        self._version = -1
        self._state_pub = self.create_publisher(String, nav_link.STATE_TOPIC, LATCHED)
        self._mission_pub = self.create_publisher(String, nav_link.MISSION_TOPIC, LATCHED)
        self._event_pub = self.create_publisher(String, nav_link.EVENT_TOPIC, nav_link.event_qos())
        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        self._action = ActionClient(self, NavigateToPose, ACTION)
        self.create_subscription(String, nav_link.COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(DiagnosticStatus, '/autonomy/status', self._on_status, LATCHED)
        self.create_subscription(DiagnosticStatus, '/pit/stall', self._on_stall, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/autonomy/pose', self._on_pose, 10)
        self.create_timer(TICK_SEC, self._tick)
        self.get_logger().info('Navigator ready')

    # -- inputs --

    def _on_status(self, msg: DiagnosticStatus) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        localizing = values.get('mode') == 'localization'
        self._localized = values.get('localized') == 'true' if localizing else None

    def _on_stall(self, msg: DiagnosticStatus) -> None:
        # pit_stall.py: 0 normal; the car drives again only from it.
        self._stalled = {kv.key: kv.value for kv in msg.values}.get('state', '0') != '0'

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_command(self, msg: String) -> None:
        raw = nav_link.decode(msg.data)
        key = str((raw or {}).get('id') or '')
        if raw is None or not key or key in self._answers:
            return
        try:
            ok, message, actions = self.commander.command(
                raw, self._pose, self._localized, self._stalled, self._action.server_is_ready()
            )
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Command refused: {exc}')
            ok, message, actions = False, BAD_MISSION, []
        self._answers[key] = (ok, message)
        while len(self._answers) > ANSWERS_KEPT:
            self._answers.popitem(last=False)
        self._apply(actions)
        self._publish()

    def _tick(self) -> None:
        now = time.monotonic()
        self._apply(self.commander.tick(now, self._pose, self._localized, self._stalled))
        self._publish_events()
        self._ticks += 1
        if self._ticks % STATE_EVERY == 0:
            self._publish()

    # -- outputs --

    def _publish_events(self) -> None:
        pose = None if self._pose is None else [round(v, 3) for v in self._pose]
        for event in self.commander.run.take_events():
            out = {'t': time.time(), 'log': self.commander.log, 'pose': pose, **event}
            self._event_pub.publish(String(data=json.dumps(out)))

    def _publish(self) -> None:
        self._publish_events()
        view = self.commander.view(time.monotonic())
        if view['version'] != self._version:
            self._version = view['version']
            mission = {'version': self._version, 'mission': self.commander.run.mission}
            self._mission_pub.publish(String(data=json.dumps(mission)))
        view['ready'] = self._action.server_is_ready()
        view['answers'] = self._answers
        self._state_pub.publish(String(data=json.dumps(view)))

    def _apply(self, actions: list[Action]) -> None:
        for action in actions:
            if action[0] == 'send':
                self._send(*action[1:])
            elif action[0] == 'cancel':
                self._cancel()
            elif action[0] == 'limit':
                msg = SpeedLimit()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.percentage = False
                msg.speed_limit = float(action[1] or 0.0)  # 0: no limit
                self._speed_pub.publish(msg)

    def _send(self, number: int, x: float, y: float, yaw: float) -> None:
        """Send a waypoint to Nav2; it preempts the goal before it."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        run = self.commander.run

        def feedback(msg: Any) -> None:
            fb = msg.feedback
            run.feedback(number, float(fb.distance_remaining), int(fb.number_of_recoveries))

        def finished(future: Any) -> None:
            result = future.result()
            status = {
                GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                GoalStatus.STATUS_CANCELED: 'canceled',
            }.get(result.status if result is not None else -1, 'failed')
            self._apply(run.result(number, status, time.monotonic(), self._pose))

        def accepted(future: Any) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                run.refused(number)
                return
            if not run.current(number):
                handle.cancel_goal_async()  # paused or stopped while Nav2 was accepting it
                return
            self._handle = handle
            handle.get_result_async().add_done_callback(finished)

        self._action.send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)

    def _cancel(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def destroy_node(self) -> None:
        # The bridge stops with its commands; the lease goes with the navigator.
        self.commander.lease.release()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    run_node(NavServerNode, args)


if __name__ == '__main__':
    main()
