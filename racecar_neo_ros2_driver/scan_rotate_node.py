"""
Republish a LaserScan rotated about the vertical axis.

Corrects a lidar whose mount is yawed relative to the chassis, once in the
driver rather than in every consumer. The RPLIDAR on this car is mounted 180
degrees out: an object off the car's right nose reads on the left rear of the
raw scan. Verified on hardware, by placing an object to the right (reported at
-90 deg in the dashboards' convention) and in front (reported at 180 deg). Only
a 180 degree yaw explains both; a mirrored scan would have put the front object
at 0.

The rotation moves angle_min and angle_max, not the ranges array. Consumers
that derive an index from angle_min follow the correction; consumers that index
the array directly are unaffected, which is what keeps racecar-neo-library
working (real/lidar_real.py indexes raw positions after np.flip). It is also
exact, where rolling the array quantizes to whole samples, and O(1) rather than
a copy of 1080 floats per scan.

The cost is that angle_min leaves [-pi, pi]: a 180 degree correction gives a
scan running 0 to 2pi. That is legal, and sensor_msgs/LaserScan imposes no such
bound, but a consumer that assumes it will need to normalize.

Off by default. `scan_rotate:=true` on the lidar launch puts sllidar on
/scan_raw and this node on /scan; without it sllidar owns /scan directly and
the node graph is unchanged.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def rotate_scan(msg: LaserScan, rotate_deg: float) -> LaserScan:
    """
    Copy msg with the angular window shifted by rotate_deg.

    The ranges array is passed through untouched; only the header angles move,
    so an index derived from angle_min lands on a different ray while a raw
    index still lands on the same one.
    """
    offset = math.radians(rotate_deg)
    out = LaserScan()
    out.header = msg.header
    out.angle_min = msg.angle_min + offset
    out.angle_max = msg.angle_max + offset
    out.angle_increment = msg.angle_increment
    out.time_increment = msg.time_increment
    out.scan_time = msg.scan_time
    out.range_min = msg.range_min
    out.range_max = msg.range_max
    out.ranges = msg.ranges
    out.intensities = msg.intensities
    return out


class ScanRotateNode(Node):
    def __init__(self):
        super().__init__('scan_rotate_node')

        self.declare_parameter('rotate_deg', 180.0)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        self._rotate_deg = float(self.get_parameter('rotate_deg').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, input_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'Rotating {input_topic} by {self._rotate_deg} deg onto {output_topic}'
        )

    def _scan_cb(self, msg: LaserScan):
        self._pub.publish(rotate_scan(msg, self._rotate_deg))


def main(args=None):
    rclpy.init(args=args)
    node = ScanRotateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
