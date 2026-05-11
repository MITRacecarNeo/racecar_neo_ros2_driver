"""
Top-level teleop launch: joy + gamepad + mux + throttle + pwm.

Each node is included via its standalone launch file so the watchdog can
restart any one in isolation. Sensors/ML added in later phases.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')

    joy_device_arg = DeclareLaunchArgument(
        'joy_device_id',
        default_value='0',
        description='Joystick device index (matches /dev/input/jsN)',
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.05',
        description='Joy node deadzone (axes below this are reported as 0)',
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        'joy_autorepeat_rate',
        default_value='20.0',
        description='Hz at which joy_node republishes axes when no event arrives',
    )

    gamepad_cfg_arg = DeclareLaunchArgument(
        'gamepad_config',
        default_value=os.path.join(config_dir, 'gamepad.yaml'),
    )
    mux_cfg_arg = DeclareLaunchArgument(
        'mux_config',
        default_value=os.path.join(config_dir, 'mux.yaml'),
    )
    throttle_cfg_arg = DeclareLaunchArgument(
        'throttle_config',
        default_value=os.path.join(config_dir, 'throttle.yaml'),
    )
    pwm_cfg_arg = DeclareLaunchArgument(
        'pwm_config',
        default_value=os.path.join(config_dir, 'pwm.yaml'),
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device_id'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }],
    )

    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gamepad.launch.py')),
        launch_arguments={'gamepad_config': LaunchConfiguration('gamepad_config')}.items(),
    )
    mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mux.launch.py')),
        launch_arguments={'mux_config': LaunchConfiguration('mux_config')}.items(),
    )
    throttle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'throttle.launch.py')),
        launch_arguments={'throttle_config': LaunchConfiguration('throttle_config')}.items(),
    )
    pwm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pwm.launch.py')),
        launch_arguments={'pwm_config': LaunchConfiguration('pwm_config')}.items(),
    )

    return LaunchDescription([
        joy_device_arg,
        joy_deadzone_arg,
        joy_autorepeat_arg,
        gamepad_cfg_arg,
        mux_cfg_arg,
        throttle_cfg_arg,
        pwm_cfg_arg,
        joy,
        gamepad,
        mux,
        throttle,
        pwm,
    ])
