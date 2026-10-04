"""
Static frame tree: robot_state_publisher on urdf/racecar_neo.urdf.xacro.

Offsets come from config/frames.yaml, with config/frames.local.yaml on top
(racecar_neo_ros2_driver.frames). The watchdog does not supervise this node:
it publishes /tf_static once, latched, which the watchdog's rate check would
read as dead. Launch respawns it instead, with a retry cap so a node that dies
on start cannot loop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from racecar_neo_ros2_driver.frames import load_frames, xacro_mappings
import xacro

RESPAWN_DELAY_SEC = 2.0
RESPAWN_MAX_RETRIES = 5


def robot_description(pkg_dir: str) -> str:
    values = load_frames(os.path.join(pkg_dir, 'config'))
    urdf = os.path.join(pkg_dir, 'urdf', 'racecar_neo.urdf.xacro')
    return xacro.process_file(urdf, mappings=xacro_mappings(values)).toxml()


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description(pkg_dir)}],
                respawn=True,
                respawn_delay=RESPAWN_DELAY_SEC,
                respawn_max_retries=RESPAWN_MAX_RETRIES,
            )
        ]
    )
