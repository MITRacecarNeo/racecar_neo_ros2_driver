"""Joy -> /gamepad_drive passthrough. All caps live in throttle_node."""

from collections.abc import Sequence
from typing import Any

from ackermann_msgs.msg import AckermannDriveStamped
from racecar_neo_ros2_driver.params import declare_params
from racecar_neo_ros2_driver.qos import SENSOR_LATEST
from racecar_neo_ros2_driver.spin import run_node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .limits import clamp


def joy_to_drive(
    axes: Sequence[float],
    throttle_axis: int,
    steering_axis: int,
    throttle_sign: float,
    steering_sign: float,
) -> tuple[float, float] | None:
    """Return (speed, steering) in [-1, 1], or None when the frame lacks either axis."""
    if len(axes) <= max(throttle_axis, steering_axis):
        return None
    speed = float(axes[throttle_axis]) * throttle_sign
    steering = float(axes[steering_axis]) * steering_sign
    return clamp(speed), clamp(steering)


# Parameters and their defaults; values come from the launch YAML.
DEFAULTS: dict[str, Any] = {
    'throttle_axis': 1,
    'steering_axis': 3,
    'throttle_sign': 1,
    'steering_sign': 1,
}


class GamepadNode(Node):
    def __init__(self) -> None:
        super().__init__('gamepad_node')

        declare_params(self, DEFAULTS)

        self._throttle_axis = self.get_parameter('throttle_axis').value
        self._steering_axis = self.get_parameter('steering_axis').value
        self._throttle_sign = self.get_parameter('throttle_sign').value
        self._steering_sign = self.get_parameter('steering_sign').value

        qos = SENSOR_LATEST
        self._pub = self.create_publisher(AckermannDriveStamped, '/gamepad_drive', qos)
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)

        self.get_logger().info(
            f'Gamepad ready: throttle axis={self._throttle_axis} '
            f'(sign={self._throttle_sign}), '
            f'steering axis={self._steering_axis} (sign={self._steering_sign})'
        )

    def _joy_cb(self, msg: Joy) -> None:
        command = joy_to_drive(
            msg.axes,
            self._throttle_axis,
            self._steering_axis,
            self._throttle_sign,
            self._steering_sign,
        )
        if command is None:
            return
        drive = AckermannDriveStamped()
        drive.drive.speed, drive.drive.steering_angle = command
        self._pub.publish(drive)


def main(args: list[str] | None = None) -> None:
    run_node(GamepadNode, args, spinner=rclpy.spin)


if __name__ == '__main__':
    main()
