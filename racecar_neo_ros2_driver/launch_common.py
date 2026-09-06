"""Shared helpers for the per-node launch files (watchdog restart targets)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def single_node_launch(
    arg_name: str,
    default_yaml: str,
    package: str,
    executable: str,
    node_name: str = None,
    remappings=None,
    description: str = None,
):
    """
    Build a 1-node LaunchDescription whose only config is a YAML param file.

    arg_name: launch arg the YAML path is exposed as (e.g. 'throttle_config').
    default_yaml: filename inside this package's share/config (e.g. 'throttle.yaml').
    """
    pkg_dir = get_package_share_directory('racecar_neo_ros2_driver')
    default_cfg = os.path.join(pkg_dir, 'config', default_yaml)
    local_cfg = os.path.join(
        pkg_dir, 'config', default_yaml.replace('.yaml', '.local.yaml')
    )

    cfg_arg = DeclareLaunchArgument(
        arg_name,
        default_value=default_cfg,
        description=description or f'Path to {executable} config YAML',
    )

    # A per-car override, applied after the shipped file so its keys win.
    # .gitignore already reserves config/*.local.yaml; this is what reads it.
    # Settings that differ per car (an enabled RC gate, say) go here instead of
    # in the committed YAML, so a car never carries a dirty working tree.
    parameters = [LaunchConfiguration(arg_name)]
    if os.path.exists(local_cfg):
        parameters.append(local_cfg)

    node_kwargs = {
        'package': package,
        'executable': executable,
        'name': node_name or executable,
        'output': 'screen',
        'parameters': parameters,
    }
    if remappings:
        node_kwargs['remappings'] = remappings
    node = Node(**node_kwargs)

    return LaunchDescription([cfg_arg, node])
