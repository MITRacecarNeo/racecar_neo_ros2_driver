"""
Nav2 drive bridge: /nav/cmd_vel (speed, turn rate) to /drive on the mux's autonomy source.

RB, or the 8080 safety override, still arms the autonomy source in mux_node;
the bridge only fills it. Speed goes through the inverse of the firmware's
speed band and throttle_node's caps, steering through the car's steering
table (racecar_neo_ros2_driver/nav.py). Commands are sent only while Nav2
sends them and the car is localized; then stop_messages zeros, then nothing,
so another /drive publisher is not fought while Nav2 is idle.

Started with localization by launch/autonomy.launch.py; settings in
config/nav_bridge.yaml. It drives only while the navigator (nav_server_node)
holds the drive lease and is not paused: a goal sent to Nav2 past it does not
move the car.
"""

from __future__ import annotations

import time
from typing import Any

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from racecar_neo_ros2_driver import car_profile, nav, steering
from racecar_neo_ros2_driver.drive_lease import DriveLease
from racecar_neo_ros2_driver.nav_runner import OWNER
from racecar_neo_ros2_driver.params import declare_params
from racecar_neo_ros2_driver.paths import config_dir
from racecar_neo_ros2_driver.qos import LATCHED, SENSOR_LATEST
from racecar_neo_ros2_driver.spin import run_node
from rclpy.node import Node

DEFAULTS: dict[str, Any] = {
    'firmware_deadband': 0.01,
    'firmware_min_mps': 0.5,
    'firmware_max_mps': 1.0,
    'command_timeout_sec': 0.5,
    'stop_messages': 5,
    'rate_hz': 20.0,
}


def speed_map(params: dict[str, Any], throttle: dict[str, Any]) -> nav.SpeedMap:
    """Return the speed map from the bridge's firmware band and throttle_node's caps."""
    return nav.SpeedMap(
        deadband=float(params['firmware_deadband']),
        min_mps=float(params['firmware_min_mps']),
        max_mps=float(params['firmware_max_mps']),
        cap_forward=float(throttle.get('max_speed_forward', 1.0)),
        cap_backward=float(throttle.get('max_speed_backward', 1.0)),
    )


def localized(msg: DiagnosticStatus) -> bool:
    return any(kv.key == 'localized' and kv.value == 'true' for kv in msg.values)


class Bridge:
    """The bridge's decisions, apart from ROS: what /drive to send at each tick, if any."""

    def __init__(self, speeds: nav.SpeedMap, table: steering.SteeringTable, timeout: float,
                 stop_messages: int) -> None:  # fmt: skip
        self.speeds, self.table = speeds, table
        self.timeout, self.stop_messages = timeout, stop_messages
        self.command: tuple[float, float] | None = None
        self.stamp = -1e9
        self.localized = False
        self.zeros_left = 0

    def on_command(self, mps: float, turn_rate: float, now: float) -> None:
        self.command, self.stamp = (mps, turn_rate), now

    def tick(self, now: float, allowed: bool = True) -> tuple[float, float] | None:
        """Return (speed, steering) to publish, or None to stay quiet."""
        fresh = self.command is not None and now - self.stamp <= self.timeout
        if fresh and self.localized and allowed:
            self.zeros_left = self.stop_messages
            mps, turn_rate = self.command  # type: ignore[misc]
            return (
                self.speeds.drive_speed(mps),
                nav.drive_steering(mps, turn_rate, self.table),
            )
        if self.zeros_left > 0:
            self.zeros_left -= 1
            return (0.0, 0.0)
        return None


class NavBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('nav_bridge_node')
        params = declare_params(self, DEFAULTS)
        throttle = car_profile.node_params('throttle', 'throttle_node', config_dir())
        table = steering.load()
        if table is None:
            raise RuntimeError('No steering table on this car. Run: racecar calibrate steering')
        self.bridge = Bridge(
            speed_map(params, throttle),
            table,
            float(params['command_timeout_sec']),
            int(params['stop_messages']),
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', SENSOR_LATEST)
        self.lease = DriveLease(OWNER, 'Navigation')
        self.create_subscription(Twist, '/nav/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(DiagnosticStatus, '/autonomy/status', self._status_cb, LATCHED)
        self.create_timer(1.0 / float(params['rate_hz']), self._tick)
        s = self.bridge.speeds
        self.get_logger().info(
            f'Nav bridge ready: {s.min_mps:g} to {s.max_mps:g} m/s, caps forward '
            f'{s.cap_forward:g} backward {s.cap_backward:g}, '
            f'turning radius {nav.min_turning_radius(table):.2f} m'
        )

    def _cmd_cb(self, msg: Twist) -> None:
        self.bridge.on_command(float(msg.linear.x), float(msg.angular.z), time.monotonic())

    def _status_cb(self, msg: DiagnosticStatus) -> None:
        self.bridge.localized = localized(msg)

    def _tick(self) -> None:
        allowed = self.lease.follows()
        out = self.bridge.tick(time.monotonic(), allowed)
        # The last zeros go out after the navigator has let go, unless another drives already.
        if out is None or (not allowed and self.lease.holder() is not None):
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed, msg.drive.steering_angle = float(out[0]), float(out[1])
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    run_node(NavBridgeNode, args)


if __name__ == '__main__':
    main()
