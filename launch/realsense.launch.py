"""Intel RealSense D435i as the RACECAR Neo v2 camera (color + depth + IMU)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    realsense_dir = get_package_share_directory('realsense2_camera')

    pointcloud_arg = DeclareLaunchArgument(
        'pointcloud_enable',
        default_value='false',
        description='Enable point cloud generation (CPU intensive on Pi 5)'
    )

    align_depth_arg = DeclareLaunchArgument(
        'align_depth_enable',
        default_value='false',
        description='Align depth frames to color camera'
    )

    depth_profile_arg = DeclareLaunchArgument(
        'depth_profile',
        default_value='640x480x30',
        description='Depth and infrared stream profile (widthxheightxfps)'
    )

    color_profile_arg = DeclareLaunchArgument(
        'color_profile',
        default_value='640x480x60',
        description='Color stream profile (widthxheightxfps)'
    )

    # D435i IMU IIO/HID-sensor sysfs attributes default to root-only on the
    # Pi 5. Permissions are fixed at the root level by setup_realsense.sh's
    # udev rule (99-realsense-imu.rules, RUN+= on iio device-add) and the
    # realsense-imu-permissions.service boot unit; the launch does not shell
    # out to sudo (no TTY under systemd, so it only ever failed with exit 1).

    # Include the stock realsense launch with UAV Neo defaults; a short delay
    # lets the camera USB enumerate before rs_launch opens it.
    realsense_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_dir, 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'camera_namespace': '/',
                    'camera_name': 'camera',
                    # Streams
                    'enable_depth': 'true',
                    'enable_color': 'true',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    # Profiles
                    'depth_module.depth_profile':
                        LaunchConfiguration('depth_profile'),
                    'rgb_camera.color_profile':
                        LaunchConfiguration('color_profile'),
                    'depth_module.infra_profile':
                        LaunchConfiguration('depth_profile'),
                    'gyro_fps': '200',
                    'accel_fps': '63',
                    'unite_imu_method': '2',
                    # Sync and alignment
                    'enable_sync': 'true',
                    'align_depth.enable':
                        LaunchConfiguration('align_depth_enable'),
                    # Filters. Decimation off so depth stays 640x480 (matches
                    # the color frame and the library's depth API shape).
                    'decimation_filter.enable': 'false',
                    'spatial_filter.enable': 'false',
                    'temporal_filter.enable': 'false',
                    # Point cloud
                    'pointcloud.enable':
                        LaunchConfiguration('pointcloud_enable'),
                    # TF
                    'publish_tf': 'true',
                    'tf_publish_rate': '0.0',
                    # Diagnostics
                    'diagnostics_period': '1.0',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        pointcloud_arg,
        align_depth_arg,
        depth_profile_arg,
        color_profile_arg,
        # Republish the RealSense streams onto the RACECAR topic names the
        # student library and edgetpu_node read: color -> /camera/color,
        # depth -> /camera/depth, and the combined IMU -> /imu/realsense
        # (imu_fusion_node merges it with the Teensy LSM9DS1 into /imu/fused).
        # SetRemap applies to the node inside the included rs_launch.
        SetRemap(src='/camera/color/image_raw', dst='/camera/color'),
        SetRemap(src='/camera/depth/image_rect_raw', dst='/camera/depth'),
        SetRemap(src='/camera/imu', dst='/imu/realsense'),
        realsense_launch,
    ])
