"""Unit tests for pit_node's pure IMU-transform helpers."""

import numpy as np
import pytest

from racecar_neo_ros2_driver.mux_node import MuxMode
from racecar_neo_ros2_driver.pit_node import (
    _MODE_BITS,
    build_odom,
    clamp,
    POSE_UNESTIMATED,
    remap_axes,
    SPEED_VARIANCE,
    SYS_DOTMATRIX_ACTIVE,
    SYS_DRIVER_STARTING,
    SYS_LED_ACTIVE,
    SYS_MODE_AUTO,
    SYS_MODE_IDLE,
    SYS_MODE_MANUAL,
    SYS_MODE_MASK,
    transform_accel,
    transform_gyro,
    TWIST_COMPONENT_UNKNOWN,
)


class TestSystemStateBits:
    """The SystemState byte layout must match firmware cfg::SYS_STATE."""

    def test_bit_layout(self):
        assert SYS_MODE_MASK == 0x03
        assert SYS_DOTMATRIX_ACTIVE == 0x04
        assert SYS_LED_ACTIVE == 0x08
        assert SYS_DRIVER_STARTING == 0x10

    def test_mode_values(self):
        assert [SYS_MODE_IDLE, SYS_MODE_MANUAL, SYS_MODE_AUTO] == [0, 1, 2]

    def test_mode_mapping(self):
        assert _MODE_BITS[MuxMode.IDLE] == SYS_MODE_IDLE
        assert _MODE_BITS[MuxMode.GAMEPAD] == SYS_MODE_MANUAL
        assert _MODE_BITS[MuxMode.AUTONOMY] == SYS_MODE_AUTO

    def test_flags_disjoint_from_mode(self):
        flags = SYS_DOTMATRIX_ACTIVE | SYS_LED_ACTIVE | SYS_DRIVER_STARTING
        assert flags & SYS_MODE_MASK == 0


class TestClamp:
    def test_within(self):
        assert clamp(0.3) == 0.3

    def test_saturates(self):
        assert clamp(2.0) == 1.0
        assert clamp(-2.0) == -1.0


class TestRemap:
    def test_identity(self):
        out = remap_axes([1.0, 2.0, 3.0], [0, 1, 2], [1.0, 1.0, 1.0])
        assert np.allclose(out, [1.0, 2.0, 3.0])

    def test_reorder_and_flip(self):
        # Map raw (x=front, y=right, z=up) into a z=front frame with a sign flip.
        out = remap_axes([1.0, 2.0, 3.0], [2, 1, 0], [1.0, 1.0, -1.0])
        assert np.allclose(out, [3.0, 2.0, -1.0])


class TestTransformAccel:
    def test_bias_and_scale(self):
        out = transform_accel(
            [9.81, 0.0, 0.0], [0, 1, 2], [1.0, 1.0, 1.0], 1.0, [0.81, 0.0, 0.0]
        )
        assert np.allclose(out, [9.0, 0.0, 0.0])


class TestTransformGyro:
    def test_deg_to_rad_scale(self):
        # gyro_scale converts deg/s -> rad/s; 180 deg/s -> pi rad/s.
        out = transform_gyro(
            [180.0, 0.0, 0.0], [0, 1, 2], [1.0, 1.0, 1.0], np.pi / 180.0, [0.0, 0.0, 0.0]
        )
        assert np.allclose(out, [np.pi, 0.0, 0.0])


class TestBuildOdom:
    """Encoder speed wrapped as Odometry; forward twist only."""

    def test_speed_lands_in_forward_twist(self):
        odom = build_odom(2.5, 'odom', 'base_link')
        assert odom.twist.twist.linear.x == pytest.approx(2.5)

    def test_frames_are_set(self):
        odom = build_odom(0.0, 'odom', 'base_link')
        assert odom.header.frame_id == 'odom'
        assert odom.child_frame_id == 'base_link'

    def test_reverse_speed_is_signed(self):
        odom = build_odom(-1.25, 'odom', 'base_link')
        assert odom.twist.twist.linear.x == pytest.approx(-1.25)

    def test_pose_is_marked_unestimated(self):
        # -1 in the first element is the ROS signal for "this block is not a
        # measurement". Zeros would read as a car parked at the origin.
        odom = build_odom(1.0, 'odom', 'base_link')
        assert odom.pose.covariance[0] == POSE_UNESTIMATED

    def test_unestimated_twist_components_are_flagged(self):
        odom = build_odom(1.0, 'odom', 'base_link')
        assert odom.twist.covariance[0] == pytest.approx(SPEED_VARIANCE)
        for i in (7, 14, 21, 28, 35):
            assert odom.twist.covariance[i] == pytest.approx(TWIST_COMPONENT_UNKNOWN)

    def test_heading_is_not_derived(self):
        # No wheelbase or steering calibration exists on this platform, and
        # /motor carries steering normalized to [-1, 1] rather than radians.
        odom = build_odom(3.0, 'odom', 'base_link')
        assert odom.twist.twist.angular.z == 0.0
