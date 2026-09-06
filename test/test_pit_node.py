"""Unit tests for pit_node's pure IMU-transform helpers."""

import numpy as np

from racecar_neo_ros2_driver.mux_node import MuxMode
from racecar_neo_ros2_driver.pit_node import (
    _MODE_BITS,
    clamp,
    remap_axes,
    SYS_DOTMATRIX_ACTIVE,
    SYS_DRIVER_STARTING,
    SYS_LED_ACTIVE,
    SYS_MODE_AUTO,
    SYS_MODE_IDLE,
    SYS_MODE_MANUAL,
    SYS_MODE_MASK,
    transform_accel,
    transform_gyro,
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
