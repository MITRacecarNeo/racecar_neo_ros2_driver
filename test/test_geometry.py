"""racecar_neo_ros2_driver.geometry: quaternion yaw and stamp seconds."""

import math
from types import SimpleNamespace

import pytest
from racecar_neo_ros2_driver.geometry import stamp_sec, yaw_of
from racecar_neo_ros2_driver.odometry import yaw_quaternion


@pytest.mark.parametrize('theta', [0.0, 0.5, -1.2, math.pi - 1e-6, -math.pi + 1e-6])
def test_yaw_round_trip(theta):
    x, y, z, w = yaw_quaternion(theta)
    assert yaw_of(SimpleNamespace(x=x, y=y, z=z, w=w)) == pytest.approx(theta)


def test_planar_shortcut_agrees_modulo_two_pi():
    x, y, z, w = yaw_quaternion(3.0)
    q = SimpleNamespace(x=x, y=y, z=-z, w=-w)  # same rotation, sign flipped
    diff = yaw_of(q) - 2 * math.atan2(q.z, q.w)
    assert math.isclose(math.remainder(diff, 2 * math.pi), 0.0, abs_tol=1e-9)


def test_stamp_sec():
    assert stamp_sec(SimpleNamespace(sec=12, nanosec=500_000_000)) == 12.5
