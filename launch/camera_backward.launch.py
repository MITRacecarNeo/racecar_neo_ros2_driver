"""Standalone backward camera launch (Arducam B0578 via gscam)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    default_cfg = os.path.join(pkg_dir, 'config', 'camera_backward.yaml')

    cfg_arg = DeclareLaunchArgument(
        'camera_backward_config',
        default_value=default_cfg,
        description='Path to backward-camera config YAML',
    )

    cam = Node(
        package='gscam',
        executable='gscam_node',
        name='camera_backward',
        output='screen',
        parameters=[LaunchConfiguration('camera_backward_config')],
        remappings=[
            ('camera/image_raw', '/camera/backward'),
            ('camera/camera_info', '/camera/backward/camera_info'),
        ],
    )

    return LaunchDescription([cfg_arg, cam])
