"""
Full-stack teleop launch: control + sensors + ML + display.

Control pipeline (gamepad/mux/throttle/pit + joy_node) is always brought up.
pit_node owns the Teensy UART and drives the servo/ESC. imu_fusion merges the
RealSense and Teensy LSM9DS1 IMUs into /imu/fused; the RealSense D435i is the
camera.
Each sensor/ML/display subsystem can be disabled via a name_enable arg
(default 'true') -- for instance, edgetpu_enable:=false skips the Coral.
EdgeTPU is staggered 3s so the camera topics are up before it subscribes
(the M.2 Apex is bound at boot, so no USB-firmware enumeration wait).
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
    'imu_fusion',
    'lidar',
    'realsense',
    'edgetpu',
    'dotmatrix',
)


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')

    # Joy node + control-pipeline configs (existing args from v0.0.1)
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
        'gamepad_config', default_value=os.path.join(config_dir, 'gamepad.yaml')
    )
    mux_cfg_arg = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(config_dir, 'mux.yaml')
    )
    throttle_cfg_arg = DeclareLaunchArgument(
        'throttle_config', default_value=os.path.join(config_dir, 'throttle.yaml')
    )
    pit_cfg_arg = DeclareLaunchArgument(
        'pit_config', default_value=os.path.join(config_dir, 'pit.yaml')
    )

    # Per-subsystem enable flags
    enable_args = [
        DeclareLaunchArgument(
            f'{name}_enable',
            default_value='true',
            description=f'Bring up {name} subsystem ("true"/"false")',
        )
        for name in _SUBSYSTEMS
    ]

    # Control pipeline — always on
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log',
        parameters=[
            {
                'device_id': LaunchConfiguration('joy_device_id'),
                'deadzone': LaunchConfiguration('joy_deadzone'),
                'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
            }
        ],
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
    pit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pit.launch.py')),
        launch_arguments={'pit_config': LaunchConfiguration('pit_config')}.items(),
    )

    def _gated_include(name: str, delay: float = 0.0):
        # TimerAction with period=0 still fires correctly and lets the condition gate.
        return TimerAction(
            period=delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, f'{name}.launch.py'))
                )
            ],
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration(f'{name}_enable'), 'true')
            ),
        )

    imu_fusion_launch = _gated_include('imu_fusion')
    lidar_launch = _gated_include('lidar')
    # RealSense D435i is the camera; realsense.launch.py remaps its color stream
    # to /camera/color, depth to /camera/depth, and the camera IMU to
    # /imu/realsense (imu_fusion_node merges it into /imu/fused).
    realsense_launch = _gated_include('realsense')
    # EdgeTPU short stagger so the camera topics are up before edgetpu_node
    # subscribes. The M.2 Apex is bound at boot, so the old 10s USB-firmware
    # enumeration wait is no longer needed.
    edgetpu_launch = _gated_include('edgetpu', delay=3.0)
    dotmatrix_launch = _gated_include('dotmatrix')

    return LaunchDescription(
        [
            joy_device_arg,
            joy_deadzone_arg,
            joy_autorepeat_arg,
            gamepad_cfg_arg,
            mux_cfg_arg,
            throttle_cfg_arg,
            pit_cfg_arg,
            *enable_args,
            joy,
            gamepad,
            mux,
            throttle,
            pit,
            imu_fusion_launch,
            lidar_launch,
            realsense_launch,
            edgetpu_launch,
            dotmatrix_launch,
        ]
    )
