"""imu_fusion_node launch (watchdog restart target)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    
    base_config = os.path.join(pkg_dir, 'config', 'imu_fusion.yaml')
    cal_config = os.path.join(pkg_dir, 'config', 'realsense_cal.yaml')
    
    node = Node(
        package='racecar_neo_ros2_driver',
        executable='imu_fusion_node',
        name='imu_fusion_node',
        output='screen',
        parameters=[
            base_config,
            cal_config
        ]
    )
    
    return LaunchDescription([node])