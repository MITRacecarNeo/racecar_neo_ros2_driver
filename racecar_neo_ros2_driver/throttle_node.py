"""
Throttle: clamps /mux_out to [-1, 1] and scales by per-direction caps onto /motor.

Software caps on top of the firmware PWM limits. +speed = forward, +steering = left.
steering_trim (per car, from calibrate_steering.py) is added after the cap so
every source drives straight at steering 0. While /autonomy/status reports
mode 'mapping', speed is also clipped to mapping_speed_cap, which the firmware
drives at its floor, and steering to mapping_steering_cap (before the trim;
1.0, off, by default). The caps lift when the status reports another mode.
speed_cap clips speed for every source at all times (1.0, off, by default);
the lowest active cap wins.
"""

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .limits import clamp


def scale_speed(speed: float, max_fwd: float, max_back: float) -> float:
    speed = clamp(speed)
    return speed * (max_fwd if speed >= 0 else max_back)


def cap_speed(speed: float, cap: float) -> float:
    """Clip speed to [-cap, cap]; a command inside the firmware deadband stays as is."""
    return max(-cap, min(cap, speed))


def status_mode(msg: DiagnosticStatus) -> str:
    return next((kv.value for kv in msg.values if kv.key == 'mode'), '')


def scale_steering(angle: float, max_steer: float, trim: float = 0.0, cap: float = 1.0) -> float:
    steer = clamp(angle) * max_steer
    return clamp(max(-cap, min(cap, steer)) + trim)


class ThrottleNode(Node):
    def __init__(self) -> None:
        super().__init__('throttle_node')

        # Conservative fallbacks for a missing YAML; throttle.yaml ships 1.0.
        self.declare_parameter('max_speed_forward', 0.17)
        self.declare_parameter('max_speed_backward', 0.24)
        self.declare_parameter('max_steering', 0.625)
        self.declare_parameter('steering_trim', 0.0)
        self.declare_parameter('mapping_speed_cap', 0.02)
        self.declare_parameter('mapping_steering_cap', 1.0)
        self.declare_parameter('speed_cap', 1.0)

        self._max_fwd = self.get_parameter('max_speed_forward').value
        self._max_back = self.get_parameter('max_speed_backward').value
        self._max_steer = self.get_parameter('max_steering').value
        self._trim = float(self.get_parameter('steering_trim').value)
        self._mapping_cap = float(self.get_parameter('mapping_speed_cap').value)
        self._mapping_steer = float(self.get_parameter('mapping_steering_cap').value)
        self._speed_cap = float(self.get_parameter('speed_cap').value)
        self._mapping = False

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/motor', qos)
        self.create_subscription(AckermannDriveStamped, '/mux_out', self._drive_cb, qos)
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(DiagnosticStatus, '/autonomy/status', self._status_cb, latched)

        self.get_logger().info(
            f'Throttle ready: max_fwd={self._max_fwd}, max_back={self._max_back}, '
            f'max_steer={self._max_steer}, steering_trim={self._trim:+g}, '
            f'speed_cap={self._speed_cap:g} (fractions of the firmware PWM range)'
        )

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        out = AckermannDriveStamped()
        out.header = msg.header
        out.drive.speed = cap_speed(
            scale_speed(msg.drive.speed, self._max_fwd, self._max_back), self._speed_cap
        )
        if self._mapping:
            out.drive.speed = cap_speed(out.drive.speed, self._mapping_cap)
        out.drive.steering_angle = scale_steering(
            msg.drive.steering_angle,
            self._max_steer,
            self._trim,
            self._mapping_steer if self._mapping else 1.0,
        )
        self._pub.publish(out)

    def _status_cb(self, msg: DiagnosticStatus) -> None:
        mapping = status_mode(msg) == 'mapping'
        if mapping != self._mapping:
            self.get_logger().info(
                f'Mapping caps (speed {self._mapping_cap:g}, steering {self._mapping_steer:g}) '
                + ('on' if mapping else 'off')
            )
        self._mapping = mapping


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
