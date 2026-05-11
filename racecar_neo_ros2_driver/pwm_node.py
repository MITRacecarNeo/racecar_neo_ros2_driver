"""
PWM driver: maps /motor (AckermannDriveStamped in [-1, 1]) to Maestro pulses.

Per axis: PWM = center + cmd * magnitude. Convention: +speed = forward,
+steering = left (matches ackermann_msgs).
"""

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from . import maestro


def command_to_pwm(cmd: float, center: int, magnitude: int, sign: int = 1) -> int:
    """
    Map a normalized command in [-1, 1] to a Maestro PWM target.

    sign=+1: cmd=+1 → center + magnitude (motor / forward = positive)
    sign=-1: cmd=+1 → center - magnitude (steering / +angle = left)
    """
    cmd = max(-1.0, min(1.0, cmd))
    return int(center + sign * cmd * magnitude)


class PwmNode(Node):
    def __init__(self):
        super().__init__('pwm_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('device_id', 12)

        self.declare_parameter('motor_channel', 0)
        self.declare_parameter('motor_center_pwm', 6000)
        self.declare_parameter('motor_magnitude_pwm', 3000)

        self.declare_parameter('steering_channel', 1)
        self.declare_parameter('steering_center_pwm', 6000)
        self.declare_parameter('steering_magnitude_pwm', 2000)

        self._serial_port = self.get_parameter('serial_port').value
        self._device_id = self.get_parameter('device_id').value

        self._motor_ch = self.get_parameter('motor_channel').value
        self._motor_center = self.get_parameter('motor_center_pwm').value
        self._motor_mag = self.get_parameter('motor_magnitude_pwm').value

        self._steer_ch = self.get_parameter('steering_channel').value
        self._steer_center = self.get_parameter('steering_center_pwm').value
        self._steer_mag = self.get_parameter('steering_magnitude_pwm').value

        self._controller = maestro.Controller(
            ttyStr=self._serial_port, device=self._device_id
        )
        self._controller.setSpeed(self._motor_ch, 0)
        self._controller.setAccel(self._motor_ch, 0)
        self._controller.setTarget(self._motor_ch, self._motor_center)
        self._controller.setSpeed(self._steer_ch, 0)
        self._controller.setAccel(self._steer_ch, 0)
        self._controller.setTarget(self._steer_ch, self._steer_center)

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            AckermannDriveStamped, '/motor', self._motor_cb, qos
        )

        self.get_logger().info(
            f'PWM ready: motor ch={self._motor_ch} '
            f'(center={self._motor_center}, mag={self._motor_mag}); '
            f'steering ch={self._steer_ch} '
            f'(center={self._steer_center}, mag={self._steer_mag})'
        )

    def _motor_cb(self, msg: AckermannDriveStamped):
        motor_target = command_to_pwm(
            msg.drive.speed, self._motor_center, self._motor_mag, sign=+1
        )
        steer_target = command_to_pwm(
            msg.drive.steering_angle, self._steer_center, self._steer_mag, sign=-1
        )
        self._controller.setTarget(self._motor_ch, motor_target)
        self._controller.setTarget(self._steer_ch, steer_target)

    def shutdown(self):
        try:
            self._controller.setTarget(self._motor_ch, self._motor_center)
            self._controller.setTarget(self._steer_ch, self._steer_center)
        except Exception:
            pass
        self._controller.close()


def main(args=None):
    rclpy.init(args=args)
    node = PwmNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
