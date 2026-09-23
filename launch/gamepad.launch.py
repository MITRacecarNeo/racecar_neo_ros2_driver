"""Standalone gamepad_node launch (watchdog restart target)."""

from launch import LaunchDescription
from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description() -> LaunchDescription:
    return single_node_launch(
        arg_name='gamepad_config',
        default_yaml='gamepad.yaml',
        package='racecar_neo_ros2_driver',
        executable='gamepad_node',
    )
