"""Tests for racecar_neo_ros2_driver/check_words.py, check rows worded for the pages."""

import pytest
from racecar_neo_ros2_driver import check_words as cw


@pytest.mark.parametrize(
    'repair, shown',
    [
        ('Calibrate imu', 'Calibrate the IMU'),
        ('Calibrate mag again', 'Recalibrate the magnetometer'),
        ('Start teleop: racecar service start teleop', 'Start teleop'),
        (
            'Keep the car still, then calibrate imu and odom',
            'Keep the car still. Then calibrate the IMU and odometry.',
        ),
        ('Restart teleop', 'Restart teleop'),
        ('', ''),
    ],
)
def test_repairs(repair, shown):
    assert cw.check_repair(repair) == shown


@pytest.mark.parametrize(
    'detail, shown',
    [
        ('no data', 'No reading'),
        ('no reading', 'No reading'),
        ('not calibrated: imu, mag', 'Not calibrated: IMU, magnetometer'),
        ('last check failed: steering', 'Last check failed: steering'),
        ('every step calibrated', 'Every step calibrated'),
        ('9.810 m/s^2 (+0.0%)', '9.810 m/s² (+0.0%)'),
    ],
)
def test_details(detail, shown):
    assert cw.check_detail(detail) == shown


def test_row_names_and_links():
    row = cw.check_row({'name': 'health link', 'detail': '8 checks', 'link': '/handbook/health'})
    assert (row['name'], row['link']) == ('Board link', 'handbook/health')
    assert cw.check_row({'name': 'camera color rate'})['name'] == 'Camera color rate'
    assert cw.checks_last(None) is None
