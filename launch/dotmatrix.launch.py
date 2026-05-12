"""Standalone dotmatrix_node launch (watchdog restart target)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    config_dir = os.path.join(pkg_dir, 'config')

    cfg_arg = DeclareLaunchArgument(
        'dotmatrix_config',
        default_value=os.path.join(config_dir, 'dotmatrix.yaml'),
        description='MAX7219 dot matrix parameters',
    )

    dotmatrix = Node(
        package='racecar_neo_ros2_driver',
        executable='dotmatrix_node',
        name='dotmatrix_node',
        output='screen',
        parameters=[LaunchConfiguration('dotmatrix_config')],
    )

    return LaunchDescription([cfg_arg, dotmatrix])
