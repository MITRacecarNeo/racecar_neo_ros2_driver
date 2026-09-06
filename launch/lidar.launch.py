"""
sllidar_node launch, optionally through the scan rotator.

With scan_rotate:=false (the default) sllidar owns /scan and the node graph is
exactly what it was. With scan_rotate:=true sllidar publishes /scan_raw and
scan_rotate_node republishes the corrected /scan, which is how a yawed lidar
mount is corrected once in the driver rather than in every consumer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PKG = 'racecar_neo_ros2_driver'


def generate_launch_description():
    pkg_dir = get_package_share_directory(_PKG)
    default_cfg = os.path.join(pkg_dir, 'config', 'lidar.yaml')
    local_cfg = os.path.join(pkg_dir, 'config', 'lidar.local.yaml')

    cfg_arg = DeclareLaunchArgument(
        'lidar_config',
        default_value=default_cfg,
        description='Path to sllidar_node config YAML',
    )
    rotate_arg = DeclareLaunchArgument(
        'scan_rotate',
        default_value='false',
        description='Publish /scan through scan_rotate_node (yawed-mount correction)',
    )

    params = [LaunchConfiguration('lidar_config')]
    if os.path.exists(local_cfg):
        params.append(local_cfg)

    rotate = LaunchConfiguration('scan_rotate')

    # Rotation off: sllidar owns /scan, unchanged from before this option existed.
    lidar_direct = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=params,
        condition=UnlessCondition(rotate),
    )

    # Rotation on: sllidar steps aside onto /scan_raw.
    lidar_raw = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=params,
        remappings=[('scan', 'scan_raw')],
        condition=IfCondition(rotate),
    )

    rotator = Node(
        package=_PKG,
        executable='scan_rotate_node',
        name='scan_rotate_node',
        output='screen',
        parameters=params,
        condition=IfCondition(rotate),
    )

    return LaunchDescription([cfg_arg, rotate_arg, lidar_direct, lidar_raw, rotator])
