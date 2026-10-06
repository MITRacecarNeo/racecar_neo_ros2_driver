"""
Throttle: clamps /mux_out to [-1, 1] and scales by per-direction caps onto /motor.

Software caps on top of the firmware PWM limits. +speed = forward, +steering = left.
steering_trim (per car, from calibrate_steering.py) is added after the cap so
every source drives straight at steering 0.
"""

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .limits import clamp


def scale_speed(speed: float, max_fwd: float, max_back: float) -> float:
    speed = clamp(speed)
    return speed * (max_fwd if speed >= 0 else max_back)


def scale_steering(angle: float, max_steer: float, trim: float = 0.0) -> float:
    return clamp(clamp(angle) * max_steer + trim)


class ThrottleNode(Node):
    def __init__(self) -> None:
        super().__init__('throttle_node')

        # Conservative fallbacks for a missing YAML; throttle.yaml ships 1.0.
        self.declare_parameter('max_speed_forward', 0.17)
        self.declare_parameter('max_speed_backward', 0.24)
        self.declare_parameter('max_steering', 0.625)
        self.declare_parameter('steering_trim', 0.0)

        self._max_fwd = self.get_parameter('max_speed_forward').value
        self._max_back = self.get_parameter('max_speed_backward').value
        self._max_steer = self.get_parameter('max_steering').value
        self._trim = float(self.get_parameter('steering_trim').value)

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/motor', qos)
        self.create_subscription(AckermannDriveStamped, '/mux_out', self._drive_cb, qos)

        self.get_logger().info(
            f'Throttle ready: max_fwd={self._max_fwd}, max_back={self._max_back}, '
            f'max_steer={self._max_steer}, steering_trim={self._trim:+g} '
            '(fractions of the firmware PWM range)'
        )

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        out = AckermannDriveStamped()
        out.header = msg.header
        out.drive.speed = scale_speed(msg.drive.speed, self._max_fwd, self._max_back)
        out.drive.steering_angle = scale_steering(
            msg.drive.steering_angle, self._max_steer, self._trim
        )
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThrottleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
