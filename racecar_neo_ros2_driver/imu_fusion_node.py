"""
IMU fusion: merge the RealSense and Teensy LSM9DS1 IMUs into /imu/fused.

Subscribes /imu/realsense and /imu/lsm9ds1 (sensor_msgs/Imu) and republishes on
/imu/fused at a fixed rate. With both sources fresh it averages linear
acceleration and angular velocity; with one, it passes that source through as
the source of truth; with neither, it stays silent. Orientation is not fused
(both sensors are 6-DoF with no absolute heading).

Sources must share one frame; config/pit.yaml maps the LSM9DS1 axes into the
RealSense IMU frame.
"""

from collections.abc import Callable, Mapping, Sequence
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu

Vec3 = tuple[float, float, float]


def fresh_sources(
    stamps: Mapping[str, float], sources: Sequence[str], now: float, timeout: float
) -> list[str]:
    """Return the sources, in configured order, whose last arrival is within timeout."""
    return [t for t in sources if t in stamps and (now - stamps[t]) <= timeout]


def fuse(samples: Sequence[tuple[Vec3, Vec3]]) -> tuple[Vec3, Vec3]:
    """Average (accel, gyro) pairs component-wise; one sample passes through unchanged."""
    accel = np.mean([a for a, _ in samples], axis=0)
    gyro = np.mean([g for _, g in samples], axis=0)
    return (
        (float(accel[0]), float(accel[1]), float(accel[2])),
        (float(gyro[0]), float(gyro[1]), float(gyro[2])),
    )


class ImuFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_fusion_node')

        self.declare_parameter('sources', ['/imu/realsense', '/imu/lsm9ds1'])
        self.declare_parameter('output_topic', '/imu/fused')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('source_timeout_sec', 0.25)
        self.declare_parameter('frame_id', 'imu_link')

        self.declare_parameter('realsense_topic', '/imu/realsense')
        self.declare_parameter('realsense_accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('realsense_gyro_bias', [0.0, 0.0, 0.0])
        self._rs_accel_bias = np.array(self.get_parameter('realsense_accel_bias').value, float)
        self._rs_gyro_bias = np.array(self.get_parameter('realsense_gyro_bias').value, float)

        self._rs_topic = self.get_parameter('realsense_topic').value
        self._sources = list(self.get_parameter('sources').value)
        output_topic = self.get_parameter('output_topic').value
        self._timeout = float(self.get_parameter('source_timeout_sec').value)
        self._frame = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._latest: dict[str, tuple[Imu, float]] = {}
        for topic in self._sources:
            self.create_subscription(Imu, topic, self._make_cb(topic), qos)
        self._pub = self.create_publisher(Imu, output_topic, qos)
        self._last_active: list[str] | None = None
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'IMU fusion: {self._sources} -> {output_topic} @ {rate}Hz')

    def _make_cb(self, topic: str) -> Callable[[Imu], None]:
        def cb(msg: Imu) -> None:
            # Bias applied on arrival; _publish re-reads the cached message each tick.
            if topic == self._rs_topic:
                msg.linear_acceleration.x -= float(self._rs_accel_bias[0])
                msg.linear_acceleration.y -= float(self._rs_accel_bias[1])
                msg.linear_acceleration.z -= float(self._rs_accel_bias[2])
                msg.angular_velocity.x -= float(self._rs_gyro_bias[0])
                msg.angular_velocity.y -= float(self._rs_gyro_bias[1])
                msg.angular_velocity.z -= float(self._rs_gyro_bias[2])
            self._latest[topic] = (msg, time.monotonic())

        return cb

    def _publish(self) -> None:
        now = time.monotonic()
        active = fresh_sources(
            {t: stamp for t, (_msg, stamp) in self._latest.items()},
            self._sources,
            now,
            self._timeout,
        )
        if not active:
            return
        if active != self._last_active:
            self.get_logger().info(f'Fused source(s): {active}')
            self._last_active = active

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame

        samples = []
        for topic in active:
            m = self._latest[topic][0]
            a, g = m.linear_acceleration, m.angular_velocity
            samples.append(((a.x, a.y, a.z), (g.x, g.y, g.z)))
        accel, gyro = fuse(samples)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = accel
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro

        # No absolute orientation from a 6-DoF IMU (ROS convention).
        out.orientation_covariance[0] = -1.0
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
