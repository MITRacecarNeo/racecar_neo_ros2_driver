"""Standalone imu_node launch (watchdog restart target)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    config_dir = os.path.join(pkg_dir, 'config')

    cal_arg = DeclareLaunchArgument(
        'imu_cal',
        default_value=os.path.join(config_dir, 'lsm9ds1_cal.yaml'),
        description='Accel/gyro calibration YAML',
    )
    mag_cal_arg = DeclareLaunchArgument(
        'imu_mag_cal',
        default_value=os.path.join(config_dir, 'lsm9ds1_mag_cal.yaml'),
        description='Magnetometer calibration YAML',
    )

    imu = Node(
        package='racecar_neo_ros2_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[
            LaunchConfiguration('imu_cal'),
            LaunchConfiguration('imu_mag_cal'),
        ],
    )

    return LaunchDescription([cal_arg, mag_cal_arg, imu])
