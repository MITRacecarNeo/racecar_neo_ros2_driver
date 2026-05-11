"""
Standalone pwm_node launch (watchdog restart target).

Owns /dev/ttyACM0; watchdog should TERM→KILL before restart to release it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    default_cfg = os.path.join(pkg_dir, 'config', 'pwm.yaml')

    cfg_arg = DeclareLaunchArgument(
        'pwm_config',
        default_value=default_cfg,
        description='Path to pwm_node config YAML',
    )

    pwm = Node(
        package='racecar_neo_ros2_driver',
        executable='pwm_node',
        name='pwm_node',
        output='screen',
        parameters=[LaunchConfiguration('pwm_config')],
    )

    return LaunchDescription([cfg_arg, pwm])
