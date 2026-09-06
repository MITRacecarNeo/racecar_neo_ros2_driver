"""Standalone imu_fusion_node launch (watchdog restart target)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')

    fusion_cfg = DeclareLaunchArgument(
        'imu_fusion_config',
        default_value=os.path.join(pkg_dir, 'config', 'imu_fusion.yaml'),
        description='Path to imu_fusion_node config YAML',
    )
    cal_cfg = DeclareLaunchArgument(
        'realsense_cal_config',
        default_value=os.path.join(pkg_dir, 'config', 'realsense_cal.yaml'),
        description='Path to the RealSense IMU bias calibration YAML',
    )

    node = Node(
        package='racecar_neo_ros2_driver',
        executable='imu_fusion_node',
        name='imu_fusion_node',
        output='screen',
        parameters=[
            LaunchConfiguration('imu_fusion_config'),
            LaunchConfiguration('realsense_cal_config'),
        ],
    )

    return LaunchDescription([fusion_cfg, cal_cfg, node])
