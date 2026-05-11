"""Standalone forward camera launch (Logitech BRIO via gscam)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    default_cfg = os.path.join(pkg_dir, 'config', 'camera_forward.yaml')

    cfg_arg = DeclareLaunchArgument(
        'camera_forward_config',
        default_value=default_cfg,
        description='Path to forward-camera config YAML',
    )

    cam = Node(
        package='gscam',
        executable='gscam_node',
        name='camera_forward',
        output='screen',
        parameters=[LaunchConfiguration('camera_forward_config')],
        remappings=[
            ('camera/image_raw', '/camera/forward'),
            ('camera/camera_info', '/camera/forward/camera_info'),
        ],
    )

    return LaunchDescription([cfg_arg, cam])
