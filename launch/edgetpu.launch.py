"""Standalone edgetpu_node launch (watchdog restart target)."""

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
        'edgetpu_config',
        default_value=os.path.join(config_dir, 'edgetpu.yaml'),
        description='EdgeTPU node parameters YAML',
    )

    edgetpu = Node(
        package='racecar_neo_ros2_driver',
        executable='edgetpu_node',
        name='edgetpu_node',
        output='screen',
        parameters=[LaunchConfiguration('edgetpu_config')],
    )

    return LaunchDescription([cfg_arg, edgetpu])
