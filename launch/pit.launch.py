"""Standalone pit_node launch; owns /dev/neo-pit-pcb (Teensy UART), watchdog restart target."""

from launch import LaunchDescription
from racecar_neo_ros2_driver.launch_common import single_node_launch


def generate_launch_description() -> LaunchDescription:
    return single_node_launch(
        arg_name='pit_config',
        default_yaml='pit.yaml',
        package='racecar_neo_ros2_driver',
        executable='pit_node',
        extra_yamls=(
            ('lsm9ds1_cal_config', 'lsm9ds1_cal.yaml'),
            ('lsm9ds1_mag_cal_config', 'lsm9ds1_mag_cal.yaml'),
        ),
    )
