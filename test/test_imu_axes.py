"""REP-103 IMU axes: the shipped remaps applied to at-rest readings from the reference car."""

from pathlib import Path

import numpy as np
import pytest
from racecar_neo_ros2_driver.imu_fusion_node import remap
from racecar_neo_ros2_driver.pit_node import remap_axes
import yaml

CONFIG = Path(__file__).resolve().parent.parent / 'config'


def _params(name: str, node: str) -> dict:
    return yaml.safe_load((CONFIG / name).read_text())[node]['ros__parameters']


PIT = _params('pit.yaml', 'pit_node')
FUSION = _params('imu_fusion.yaml', 'imu_fusion_node')
# Car upright and still, 2026-10-08. LSM9DS1 chip axes, from the optical-axis
# reading (-0.575, -9.576, -0.261) through the earlier mapping.
LSM_ACCEL = [-0.261, 0.575, -9.576]
LSM_MAG = [2.91e-5, 1.065e-4, 1.571e-4]  # chip magnetometer axes, T
RS_ACCEL = [0.0098, -8.993, -0.402]  # /imu/realsense, optical axes


def _det(order, sign) -> float:
    m = np.zeros((3, 3))
    for i, (j, s) in enumerate(zip(order, sign, strict=True)):
        m[i, j] = s
    return float(np.linalg.det(m))


def test_lsm9ds1_gravity_is_up():
    a = remap_axes(LSM_ACCEL, PIT['imu.accel_gyro_axis_order'], PIT['imu.accel_gyro_axis_sign'])
    assert a[2] == pytest.approx(9.576) and abs(a[0]) < 1 and abs(a[1]) < 1


def test_realsense_gravity_is_up():
    a = remap(RS_ACCEL, FUSION['realsense_axis_order'], FUSION['realsense_axis_sign'])
    assert a[2] == pytest.approx(8.993) and abs(a[0]) < 1 and abs(a[1]) < 1


def test_the_two_imus_agree_on_body_axes():
    lsm = remap_axes(LSM_ACCEL, PIT['imu.accel_gyro_axis_order'], PIT['imu.accel_gyro_axis_sign'])
    rs = remap(RS_ACCEL, FUSION['realsense_axis_order'], FUSION['realsense_axis_sign'])
    cos = np.dot(lsm, rs) / np.linalg.norm(lsm) / np.linalg.norm(rs)
    # About 3.4 deg apart at rest on the reference car: separate mounts, uncalibrated.
    assert cos > np.cos(np.radians(5))


def test_magnetic_field_points_down():
    # Earth's field dips about 66 deg below horizontal in Massachusetts.
    m = remap_axes(LSM_MAG, PIT['imu.mag_axis_order'], PIT['imu.mag_axis_sign'])
    assert m[2] < -0.7 * np.linalg.norm(m)


def test_magnetometer_and_realsense_remaps_are_rotations():
    # Both sensors are right-handed; only the LSM9DS1 accel/gyro axes are not.
    assert _det(PIT['imu.mag_axis_order'], PIT['imu.mag_axis_sign']) == pytest.approx(1.0)
    assert _det(FUSION['realsense_axis_order'], FUSION['realsense_axis_sign']) == pytest.approx(
        1.0
    )
    assert _det(
        PIT['imu.accel_gyro_axis_order'], PIT['imu.accel_gyro_axis_sign']
    ) == pytest.approx(-1.0)


def test_calibration_poses_follow_each_topics_axes():
    from conftest import load_script

    common = load_script('calibrate_common')
    lsm = load_script('calibrate_imu').TARGET
    rs = load_script('calibrate_realsense_imu').TARGET
    assert dict(lsm.positions)['Z+'] == 'Set the car upright on its wheels.'
    assert dict(lsm.positions)['X+'].startswith('Stand the car on its rear end')
    assert rs.positions is common.OPTICAL_POSITIONS
    assert dict(rs.positions)['Y-'] == 'Set the car upright on its wheels.'
