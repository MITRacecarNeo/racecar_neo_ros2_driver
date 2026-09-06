"""
IMU fusion: merge the RealSense and Teensy LSM9DS1 IMUs into /imu/fused.

Subscribes /imu/realsense and /imu/lsm9ds1 (sensor_msgs/Imu) and republishes on
/imu/fused at a fixed rate. With both sources fresh it averages linear
acceleration and angular velocity; with one, it passes that source through as
the source of truth; with neither, it stays silent. Orientation is not fused
(both sensors are 6-DoF with no absolute heading).

The average is only meaningful when both inputs are expressed in the common
body frame; align each source's axes upstream (the RealSense optical frame and
the LSM9DS1 mounting both to the body frame) before trusting the fused output
while both publish. Today only /imu/realsense publishes, so /imu/fused passes
it through.
"""

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


class ImuFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')

        self.declare_parameter('sources', ['/imu/realsense', '/imu/lsm9ds1'])
        self.declare_parameter('output_topic', '/imu/fused')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('source_timeout_sec', 0.25)
        self.declare_parameter('frame_id', 'imu_link')

        self.declare_parameter('realsense_accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('realsense_gyro_bias', [0.0, 0.0, 0.0])
        self._rs_accel_bias = np.array(self.get_parameter('realsense_accel_bias').value, float)
        self._rs_gyro_bias = np.array(self.get_parameter('realsense_gyro_bias').value, float)

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
        self._latest = {}  # topic -> (Imu, monotonic stamp)
        for topic in self._sources:
            self.create_subscription(Imu, topic, self._make_cb(topic), qos)
        self._pub = self.create_publisher(Imu, output_topic, qos)
        self._last_active = None
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'IMU fusion: {self._sources} -> {output_topic} @ {rate}Hz'
        )

    def _make_cb(self, topic: str):
        def cb(msg: Imu):
            # Correct on arrival, not on publish: _publish re-reads the cached
            # message every timer tick, so subtracting there compounds the bias
            # once per tick whenever the source is slower than the timer.
            if topic == '/imu/realsense':
                msg.linear_acceleration.x -= float(self._rs_accel_bias[0])
                msg.linear_acceleration.y -= float(self._rs_accel_bias[1])
                msg.linear_acceleration.z -= float(self._rs_accel_bias[2])
                msg.angular_velocity.x -= float(self._rs_gyro_bias[0])
                msg.angular_velocity.y -= float(self._rs_gyro_bias[1])
                msg.angular_velocity.z -= float(self._rs_gyro_bias[2])
            self._latest[topic] = (msg, time.monotonic())
        return cb

    def _publish(self):
        now = time.monotonic()
        fresh = []
        active = []
        for topic in self._sources:
            item = self._latest.get(topic)
            if item is not None and (now - item[1]) <= self._timeout:
                fresh.append(item[0])
                active.append(topic)

        if not fresh:
            return
        if active != self._last_active:
            self.get_logger().info(f'Fused source(s): {active or "none"}')
            self._last_active = active

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame

        if len(fresh) == 1:
            src = fresh[0]
            out.linear_acceleration = src.linear_acceleration
            out.angular_velocity = src.angular_velocity
        else:
            accel = np.mean([[m.linear_acceleration.x,
                              m.linear_acceleration.y,
                              m.linear_acceleration.z] for m in fresh], axis=0)
            gyro = np.mean([[m.angular_velocity.x,
                             m.angular_velocity.y,
                             m.angular_velocity.z] for m in fresh], axis=0)
            out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (
                float(accel[0]), float(accel[1]), float(accel[2])
            )
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = (
                float(gyro[0]), float(gyro[1]), float(gyro[2])
            )

        # No absolute orientation from a 6-DoF IMU (ROS convention).
        out.orientation_covariance[0] = -1.0
        self._pub.publish(out)


def main(args=None):
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
