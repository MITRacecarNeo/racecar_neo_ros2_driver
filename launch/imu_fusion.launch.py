"""Standalone imu_fusion_node launch (watchdog restart target)."""

from launch import LaunchDescription
from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description() -> LaunchDescription:
    return single_node_launch(
        arg_name='imu_fusion_config',
        default_yaml='imu_fusion.yaml',
        package='racecar_neo_ros2_driver',
        executable='imu_fusion_node',
        extra_yamls=(('realsense_cal_config', 'realsense_cal.yaml'),),
    )
