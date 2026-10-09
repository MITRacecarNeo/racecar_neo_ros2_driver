"""
SLAM on top of a running teleop stack: mapping, or localization on a saved map.

    racecar launch autonomy map:=room                         # mapping, saved as ~/data/maps/room
    racecar launch autonomy mode:=localization map:=room      # AMCL on ~/data/maps/room
    racecar launch autonomy mode:=localization map:=room localizer:=slam_toolbox pose:=1.0,0.5,1.57
    racecar launch autonomy mode:=localization map:=room pose:=global    # AMCL searches the map

Teleop provides /scan and the odom -> base_link -> laser transforms. Mapping
runs slam_toolbox; localization runs AMCL with map_server (default) or the
slam_toolbox localizer. autonomy_node runs in every mode and publishes
/autonomy/*. `map` is a name under the map root (racecar_neo_ros2_driver/
maps.py) or a path stem; mapping saves under it. `pose` (x,y,yaw in the map
frame) is the start pose; without one, both localizers start at the map
origin, which is the odometry origin of the mapping run (where pit_node
started); `global` (AMCL only) searches the whole map. `resume` continues a
mapping run from an autosaved pose graph at `pose`; `foxglove:=true` adds
foxglove_bridge. Each node's config loads its `.local.yaml` override last. A
node that exits ends the launch.
"""

from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racecar_neo_ros2_driver.launch_common import local_overrides
from racecar_neo_ros2_driver.maps import map_stem, parse_pose

MODES = ('mapping', 'localization')
LOCALIZERS = ('amcl', 'slam_toolbox')


def _files(config_dir: str, yaml: str) -> list[str]:
    return [os.path.join(config_dir, yaml), *local_overrides(config_dir, [yaml])]


def _lifecycle_manager(name: str, files: list[str]) -> dict[str, Any]:
    """Return a lifecycle manager that configures and activates its node_names."""
    return {
        'package': 'nav2_lifecycle_manager',
        'executable': 'lifecycle_manager',
        'name': name,
        'parameters': files,
    }


def autonomy_spec(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    session: str,
    config_dir: str,
    pose_source: str = '',
    resumed: bool = False,
) -> dict[str, Any]:
    if pose_source:
        source = pose_source
    elif resumed:
        source = 'autosave'
    elif mode == 'mapping':
        source = ''
    else:
        source = 'global' if pose_text == 'global' else 'pose' if pose_text else 'origin'
    return {
        'package': 'racecar_neo_ros2_driver',
        'executable': 'autonomy_node',
        'name': 'autonomy_node',
        'parameters': [
            *_files(config_dir, 'autonomy.yaml'),
            {
                'mode': mode,
                'map': map_name,
                'localizer': localizer,
                'session': session,
                'pose': pose_text,
                'pose_source': source,
                'resumed_from_autosave': resumed,
            },
        ],
    }


def foxglove_spec(config_dir: str) -> dict[str, Any]:
    return {
        'package': 'foxglove_bridge',
        'executable': 'foxglove_bridge',
        'name': 'foxglove_bridge',
        'parameters': _files(config_dir, 'foxglove.yaml'),
    }


def node_specs(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    config_dir: str,
    session: str = '',
    resume: str = '',
    foxglove: bool = False,
    pose_source: str = '',
) -> list[dict[str, Any]]:
    """Return (package, executable, name, parameters) per node; ValueError on a bad argument."""
    specs = [
        *slam_specs(mode, localizer, map_name, pose_text, config_dir, resume),
        autonomy_spec(
            mode,
            localizer,
            map_name,
            pose_text,
            session,
            config_dir,
            pose_source,
            bool(resume),
        ),
    ]
    return [*specs, foxglove_spec(config_dir)] if foxglove else specs


def slam_specs(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    config_dir: str,
    resume: str = '',
) -> list[dict[str, Any]]:
    if mode not in MODES:
        raise ValueError(f'mode must be one of {", ".join(MODES)}: {mode!r}')
    if localizer not in LOCALIZERS:
        raise ValueError(f'localizer must be one of {", ".join(LOCALIZERS)}: {localizer!r}')
    search = pose_text.strip() == 'global'
    if search and (mode != 'localization' or localizer != 'amcl'):
        raise ValueError('pose:=global needs mode:=localization with localizer:=amcl')
    pose = None if search else parse_pose(pose_text)
    if resume and mode != 'mapping':
        raise ValueError('resume:= is for mode:=mapping')
    if mode == 'mapping':
        files = _files(config_dir, 'slam_mapping.yaml')
        # Resuming: slam_toolbox loads the autosaved pose graph and continues
        # mapping from the start pose, which the scan match then refines.
        resumed = (
            [{'map_file_name': resume, 'map_start_pose': list(pose or (0.0, 0.0, 0.0))}]
            if resume
            else []
        )
        return [
            {
                'package': 'slam_toolbox',
                'executable': 'async_slam_toolbox_node',
                'name': 'slam_toolbox',
                'parameters': [*files, *resumed],
            },
            _lifecycle_manager('lifecycle_manager_slam', files),
        ]
    if not map_name:
        raise ValueError('mode:=localization needs map:=NAME')
    stem = str(map_stem(map_name))
    if localizer == 'slam_toolbox':
        start = list(pose) if pose else [0.0, 0.0, 0.0]
        files = _files(config_dir, 'slam_localization.yaml')
        return [
            {
                'package': 'slam_toolbox',
                'executable': 'localization_slam_toolbox_node',
                'name': 'slam_toolbox',
                'parameters': [*files, {'map_file_name': stem, 'map_start_pose': start}],
            },
            _lifecycle_manager('lifecycle_manager_slam', files),
        ]
    amcl_files = _files(config_dir, 'amcl.yaml')
    x, y, yaw = pose or (0.0, 0.0, 0.0)
    # A global search starts without a pose; autonomy_node asks AMCL to spread
    # its particles over the map once AMCL is up.
    amcl_params = [
        *amcl_files,
        {
            'set_initial_pose': not search,
            'initial_pose.x': x,
            'initial_pose.y': y,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': yaw,
        },
    ]
    return [
        {
            'package': 'nav2_map_server',
            'executable': 'map_server',
            'name': 'map_server',
            'parameters': [*amcl_files, {'yaml_filename': stem + '.yaml'}],
        },
        {
            'package': 'nav2_amcl',
            'executable': 'amcl',
            'name': 'amcl',
            'parameters': amcl_params,
        },
        _lifecycle_manager('lifecycle_manager_localization', amcl_files),
    ]


def _launch_setup(context: LaunchContext) -> list[Node]:
    config_dir = os.path.join(get_package_share_directory('racecar_neo_ros2_driver'), 'config')
    specs = node_specs(
        LaunchConfiguration('mode').perform(context),
        LaunchConfiguration('localizer').perform(context),
        LaunchConfiguration('map').perform(context),
        LaunchConfiguration('pose').perform(context),
        config_dir,
        LaunchConfiguration('session').perform(context),
        LaunchConfiguration('resume').perform(context),
        LaunchConfiguration('foxglove').perform(context).lower() in ('1', 'true'),
        LaunchConfiguration('pose_source').perform(context),
    )
    # 'both': node output also goes to the launch's log directory.
    return [Node(**spec, output='both', on_exit=Shutdown()) for spec in specs]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode', default_value='mapping', description='mapping or localization'
            ),
            DeclareLaunchArgument(
                'localizer', default_value='amcl', description='localization: amcl or slam_toolbox'
            ),
            DeclareLaunchArgument(
                'map',
                default_value='',
                description='map name or path stem: saved to (mapping) or loaded (localization)',
            ),
            DeclareLaunchArgument(
                'pose',
                default_value='',
                description='localization: start pose x,y,yaw (map frame), or global (AMCL)',
            ),
            DeclareLaunchArgument(
                'session', default_value='', description='run id recorded with saves'
            ),
            DeclareLaunchArgument(
                'resume',
                default_value='',
                description='mapping: autosaved pose graph stem to continue from, at pose',
            ),
            DeclareLaunchArgument(
                'foxglove',
                default_value='false',
                description='also start foxglove_bridge (port 8765)',
            ),
            DeclareLaunchArgument(
                'pose_source', default_value='', description='reported in /autonomy/status'
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
