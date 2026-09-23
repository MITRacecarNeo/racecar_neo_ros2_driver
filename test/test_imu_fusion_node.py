"""Unit tests for imu_fusion_node: source freshness and the fusion step."""

import pytest
from racecar_neo_ros2_driver.imu_fusion_node import fresh_sources, fuse

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
