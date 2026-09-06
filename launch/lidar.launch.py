"""Standalone sllidar_node launch (watchdog restart target)."""

from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='lidar_config',
        default_yaml='lidar.yaml',
        package='sllidar_ros2',
        executable='sllidar_node',
    )
