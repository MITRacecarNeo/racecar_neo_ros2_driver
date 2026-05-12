"""
Full-stack teleop launch: control + sensors + ML + display.

Control pipeline (gamepad/mux/throttle/pwm + joy_node) is always brought up.
Each sensor/ML/display subsystem can be disabled via a name_enable arg
(default 'true') -- for instance, edgetpu_enable:=false skips the Coral.
EdgeTPU is delayed 10s so Coral USB firmware enumerates before
make_interpreter runs; the backward camera is delayed 5s to stagger USB
bus contention.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


_SUBSYSTEMS = (
    'imu', 'lidar', 'camera_forward', 'camera_backward', 'edgetpu', 'dotmatrix',
)


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')

    # Joy node + control-pipeline configs (existing args from v0.0.1)
    joy_device_arg = DeclareLaunchArgument(
        'joy_device_id', default_value='0',
        description='Joystick device index (matches /dev/input/jsN)',
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone', default_value='0.05',
        description='Joy node deadzone (axes below this are reported as 0)',
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        'joy_autorepeat_rate', default_value='20.0',
        description='Hz at which joy_node republishes axes when no event arrives',
    )
    gamepad_cfg_arg = DeclareLaunchArgument(
        'gamepad_config', default_value=os.path.join(config_dir, 'gamepad.yaml'))
    mux_cfg_arg = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(config_dir, 'mux.yaml'))
    throttle_cfg_arg = DeclareLaunchArgument(
        'throttle_config', default_value=os.path.join(config_dir, 'throttle.yaml'))
    pwm_cfg_arg = DeclareLaunchArgument(
        'pwm_config', default_value=os.path.join(config_dir, 'pwm.yaml'))

    # Per-subsystem enable flags
    enable_args = [
        DeclareLaunchArgument(
            f'{name}_enable', default_value='true',
            description=f'Bring up {name} subsystem ("true"/"false")',
        )
        for name in _SUBSYSTEMS
    ]

    # Control pipeline — always on
    joy = Node(
        package='joy', executable='joy_node', name='joy_node', output='log',
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

    def _gated_include(name: str, delay: float = 0.0):
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, f'{name}.launch.py')),
        )
        if delay > 0:
            return TimerAction(
                period=delay,
                actions=[include],
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration(f'{name}_enable'), 'true'
                    )
                ),
            )
        # Wrap a 0-delay TimerAction so the condition still gates correctly.
        return TimerAction(
            period=0.0,
            actions=[include],
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration(f'{name}_enable'), 'true')
            ),
        )

    imu_launch = _gated_include('imu')
    lidar_launch = _gated_include('lidar')
    camera_forward_launch = _gated_include('camera_forward')
    # Backward camera delayed 5s — gives the forward camera time to grab its
    # USB bandwidth share before the Arducam starts negotiating.
    camera_backward_launch = _gated_include('camera_backward', delay=5.0)
    # EdgeTPU delayed 10s — Coral USB firmware enumeration (1a6e:089a →
    # 18d1:9302) needs to complete before make_interpreter runs.
    edgetpu_launch = _gated_include('edgetpu', delay=10.0)
    dotmatrix_launch = _gated_include('dotmatrix')

    return LaunchDescription([
        joy_device_arg, joy_deadzone_arg, joy_autorepeat_arg,
        gamepad_cfg_arg, mux_cfg_arg, throttle_cfg_arg, pwm_cfg_arg,
        *enable_args,
        joy, gamepad, mux, throttle, pwm,
        imu_launch, lidar_launch,
        camera_forward_launch, camera_backward_launch,
        edgetpu_launch, dotmatrix_launch,
    ])
