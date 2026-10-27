"""Unit tests for imu_fusion_node: source freshness and the fusion step."""

import numpy as np
import pytest
from racecar_neo_ros2_driver.imu_fusion_node import fresh_sources, fuse, ImuFusionNode
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu

SOURCES = ['/imu/realsense', '/imu/lsm9ds1']


class TestFreshSources:
    def test_both_fresh(self):
        stamps = {'/imu/realsense': 9.9, '/imu/lsm9ds1': 9.95}
        assert fresh_sources(stamps, SOURCES, 10.0, 0.25) == SOURCES

    def test_stale_source_is_dropped(self):
        stamps = {'/imu/realsense': 9.5, '/imu/lsm9ds1': 9.95}
        assert fresh_sources(stamps, SOURCES, 10.0, 0.25) == ['/imu/lsm9ds1']

    def test_exactly_at_the_timeout_is_fresh(self):
        assert fresh_sources({'/imu/lsm9ds1': 9.75}, SOURCES, 10.0, 0.25) == ['/imu/lsm9ds1']

    def test_never_received_is_not_fresh(self):
        assert fresh_sources({}, SOURCES, 10.0, 0.25) == []

    def test_configured_order_is_kept(self):
        stamps = {'/imu/lsm9ds1': 10.0, '/imu/realsense': 10.0}
        assert fresh_sources(stamps, SOURCES, 10.0, 0.25) == SOURCES

    def test_unconfigured_topics_are_ignored(self):
        assert fresh_sources({'/imu/other': 10.0}, SOURCES, 10.0, 0.25) == []


class TestFuse:
    def test_one_source_passes_through(self):
        sample = ((0.1, -0.2, 9.81), (0.01, 0.02, -0.03))
        assert fuse([sample]) == sample

    def test_two_sources_are_averaged_per_component(self):
        accel, gyro = fuse(
            [
                ((0.0, 0.2, 9.8), (0.1, 0.0, -0.2)),
                ((0.2, 0.0, 9.6), (0.3, 0.2, 0.0)),
            ]
        )
        assert accel == pytest.approx((0.1, 0.1, 9.7))
        assert gyro == pytest.approx((0.2, 0.1, -0.1))

    def test_returns_plain_floats(self):
        accel, gyro = fuse([((1, 2, 3), (4, 5, 6))])
        assert all(type(v) is float for v in accel + gyro)


class _Fusion:
    """ImuFusionNode._sample over plain attributes, without rclpy."""

    _sample = ImuFusionNode._sample

    def __init__(self):
        self._rs_topic = '/imu/realsense'
        self._rs_accel_bias = np.array([0.1, 0.0, -0.2])
        self._rs_accel_scale = np.array([1.0, 2.0, 1.0])
        self._rs_gyro_bias = np.array([0.01, 0.02, 0.03])
        self._rs_order = [1, 0, 2]
        self._rs_sign = [1.0, -1.0, 1.0]


def _raw(accel, gyro):
    msg = Imu()
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
    return serialize_message(msg)


class TestSample:
    def test_realsense_is_calibrated_then_remapped(self):
        raw = _raw((1.1, 2.0, 9.8), (0.11, 0.22, 0.33))
        accel, gyro = _Fusion()._sample('/imu/realsense', raw)
        # (raw - bias) * scale = (1.0, 4.0, 10.0); order [1, 0, 2], sign [+, -, +].
        assert accel == pytest.approx((4.0, -1.0, 10.0))
        assert gyro == pytest.approx((0.2, -0.1, 0.3))

    def test_other_source_passes_through(self):
        accel, gyro = _Fusion()._sample('/imu/lsm9ds1', _raw((1.0, 2.0, 3.0), (0.1, 0.2, 0.3)))
        assert accel == pytest.approx((1.0, 2.0, 3.0)) and gyro == pytest.approx((0.1, 0.2, 0.3))
