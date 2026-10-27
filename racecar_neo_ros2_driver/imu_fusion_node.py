"""
IMU fusion: merge the RealSense and Teensy LSM9DS1 IMUs into /imu/fused.

Subscribes /imu/realsense and /imu/lsm9ds1 (sensor_msgs/Imu) and republishes on
/imu/fused at a fixed rate. With both sources fresh it averages linear
acceleration and angular velocity; with one, it passes that source through as
the source of truth; with neither, it stays silent. Orientation is not fused
(both sensors are 6-DoF with no absolute heading).

Sources must share one frame, REP-103 body axes (x forward, y left, z up):
config/pit.yaml remaps the LSM9DS1 in pit_node, and this node remaps the
RealSense IMU (realsense_axis_order, realsense_axis_sign) after its calibration.

Both sources arrive at 200 Hz and only the newest of each is used, so each
message is kept as it arrived (a raw subscription) and only the newest is
decoded and calibrated, at the publish rate.
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
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

from .spin import spin

Vec3 = tuple[float, float, float]


def fresh_sources(
    stamps: Mapping[str, float], sources: Sequence[str], now: float, timeout: float
) -> list[str]:
    """Return the sources, in configured order, whose last arrival is within timeout."""
    return [t for t in sources if t in stamps and (now - stamps[t]) <= timeout]


def remap(vec: Vec3, order: Sequence[int], sign: Sequence[float]) -> Vec3:
    """Reorder and sign-flip a 3-vector: out[i] = sign[i] * vec[order[i]]."""
    return (
        sign[0] * vec[order[0]],
        sign[1] * vec[order[1]],
        sign[2] * vec[order[2]],
    )


def fuse(samples: Sequence[tuple[Vec3, Vec3]]) -> tuple[Vec3, Vec3]:
    """Average (accel, gyro) pairs component-wise; one sample passes through unchanged."""
    n = len(samples)

    def mean(vecs: Sequence[Vec3]) -> Vec3:
        return (
            sum(v[0] for v in vecs) / n,
            sum(v[1] for v in vecs) / n,
            sum(v[2] for v in vecs) / n,
        )

    return mean([a for a, _ in samples]), mean([g for _, g in samples])


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
        self.declare_parameter('realsense_accel_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('realsense_axis_order', [0, 1, 2])
        self.declare_parameter('realsense_axis_sign', [1.0, 1.0, 1.0])
        self._rs_order = [int(i) for i in self.get_parameter('realsense_axis_order').value]
        self._rs_sign = [float(s) for s in self.get_parameter('realsense_axis_sign').value]
        self._rs_accel_bias = np.array(self.get_parameter('realsense_accel_bias').value, float)
        self._rs_accel_scale = np.array(self.get_parameter('realsense_accel_scale').value, float)
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
        self._latest: dict[str, tuple[bytes, float]] = {}
        for topic in self._sources:
            self.create_subscription(Imu, topic, self._make_cb(topic), qos, raw=True)
        self._pub = self.create_publisher(Imu, output_topic, qos)
        self._last_active: list[str] | None = None
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'IMU fusion: {self._sources} -> {output_topic} @ {rate}Hz')

    def _make_cb(self, topic: str) -> Callable[[bytes], None]:
        def cb(raw: bytes) -> None:
            self._latest[topic] = (raw, time.monotonic())

        return cb

    def _sample(self, topic: str, raw: bytes) -> tuple[Vec3, Vec3]:
        """Decode a newest message; RealSense accel is (raw - bias) * scale, then remapped."""
        msg = deserialize_message(raw, Imu)
        a, g = msg.linear_acceleration, msg.angular_velocity
        accel: Vec3 = (a.x, a.y, a.z)
        gyro: Vec3 = (g.x, g.y, g.z)
        if topic == self._rs_topic:
            bias, scale, gbias = self._rs_accel_bias, self._rs_accel_scale, self._rs_gyro_bias
            accel = (
                (accel[0] - float(bias[0])) * float(scale[0]),
                (accel[1] - float(bias[1])) * float(scale[1]),
                (accel[2] - float(bias[2])) * float(scale[2]),
            )
            gyro = (
                gyro[0] - float(gbias[0]),
                gyro[1] - float(gbias[1]),
                gyro[2] - float(gbias[2]),
            )
            accel = remap(accel, self._rs_order, self._rs_sign)
            gyro = remap(gyro, self._rs_order, self._rs_sign)
        return accel, gyro

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

        accel, gyro = fuse([self._sample(topic, self._latest[topic][0]) for topic in active])
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = accel
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro

        # No absolute orientation from a 6-DoF IMU (ROS convention).
        out.orientation_covariance[0] = -1.0
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuFusionNode()
    try:
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
