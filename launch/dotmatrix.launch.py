"""Standalone dotmatrix_node launch."""

from launch import LaunchDescription
from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description() -> LaunchDescription:
    return single_node_launch(
        arg_name='dotmatrix_config',
        default_yaml='dotmatrix.yaml',
        package='racecar_neo_ros2_driver',
        executable='dotmatrix_node',
        description='Dot-matrix rasterizer parameters',
    )
