"""Standalone pit_node launch — owns /dev/neo-pit-pcb (Teensy UART); watchdog restart target."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    
    return LaunchDescription([
        Node(
            package='racecar_neo_ros2_driver',
            executable='pit_node',
            name='pit_node',
            parameters=[
                os.path.join(pkg_dir, 'config', 'pit.yaml'),
                os.path.join(pkg_dir, 'config', 'lsm9ds1_cal.yaml'),
                os.path.join(pkg_dir, 'config', 'lsm9ds1_mag_cal.yaml')
            ]
        )
    ])