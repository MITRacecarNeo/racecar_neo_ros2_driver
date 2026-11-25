"""
SLAM on top of a running teleop stack: mapping, or localization on a saved map.

    racecar launch autonomy map:=room                         # mapping, saved in <data root>/maps
    racecar launch autonomy mode:=localization map:=room      # AMCL on <data root>/maps/room
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
mapping run from an autosaved pose graph at `pose`; `extend` starts a mapping
run from a saved pose graph, scan-matched from the rough `pose`, and saves
under `map`. `foxglove:=true` adds foxglove_bridge. Each node's config loads
its car profile override last. A node that exits ends the launch.

Localization also starts Nav2 (config/nav2.yaml, config/nav2_bt.xml) and the
drive bridge (nav_bridge_node: /nav/cmd_vel to /drive), unless `nav:=false`
or the car has no steering table. The Nav2 parameters are written to one
file at launch with this car's footprint (frames.yaml) and turning radius.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racecar_neo_ros2_driver.launch_common import local_overrides
from racecar_neo_ros2_driver.maps import map_stem, parse_pose
import yaml

MODES = ('mapping', 'localization')
LOCALIZERS = ('amcl', 'slam_toolbox')


def _files(config_dir: str, yaml: str) -> list[str]:
    return [os.path.join(config_dir, yaml), *local_overrides([yaml])]


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
    source_stem: str = '',
    extended_from: str = '',
) -> dict[str, Any]:
    if pose_source:
        source = pose_source
    elif resumed:
        source = 'autosave'
    elif mode == 'mapping':
        source = 'pose' if source_stem else ''
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
                'source': source_stem,
                'extended_from': extended_from,
            },
        ],
    }


# Nav2 server: package.
NAV2_NODES = {
    'controller_server': 'nav2_controller',
    'planner_server': 'nav2_planner',
    'behavior_server': 'nav2_behaviors',
    'bt_navigator': 'nav2_bt_navigator',
}


def nav2_param_file(config_dir: str, out_dir: str | None = None) -> str | None:
    """
    Write Nav2's parameters for this car and return the file, or None without a steering table.

    config/nav2.yaml, its car profile override, then the footprint, the
    turning radius and the behavior tree.
    """
    from racecar_neo_ros2_driver import frames, nav, steering

    table = steering.load()
    if table is None:
        return None
    tree: dict[str, Any] = {}
    for path in _files(config_dir, 'nav2.yaml'):
        tree = nav.merge(tree, yaml.safe_load(Path(path).read_text()) or {})
    tree = nav.nav2_params(tree, table, frames.load_frames(config_dir))
    tree['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
        config_dir, 'nav2_bt.xml'
    )
    folder = Path(out_dir or tempfile.gettempdir())
    path = folder / f'racecar_nav2_{os.getpid()}.yaml'
    path.write_text(yaml.safe_dump(tree, sort_keys=False))
    return str(path)


def nav_specs(config_dir: str, params_file: str) -> list[dict[str, Any]]:
    """Return Nav2's servers, their lifecycle manager, the drive bridge and the navigator."""
    specs: list[dict[str, Any]] = []
    for name, package in NAV2_NODES.items():
        spec = {'package': package, 'executable': name, 'name': name, 'parameters': [params_file]}
        if name == 'controller_server':
            # The bridge reads its own topic, apart from any other cmd_vel publisher.
            spec['remappings'] = [('cmd_vel', '/nav/cmd_vel')]
        specs.append(spec)
    specs.append(_lifecycle_manager('lifecycle_manager_navigation', [params_file]))
    specs.append(
        {
            'package': 'racecar_neo_ros2_driver',
            'executable': 'nav_bridge_node',
            'name': 'nav_bridge_node',
            'parameters': _files(config_dir, 'nav_bridge.yaml'),
        }
    )
    specs.append(
        {
            'package': 'racecar_neo_ros2_driver',
            'executable': 'nav_server_node',
            'name': 'nav_server_node',
            'parameters': [],
        }
    )
    return specs


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
    extend: str = '',
    nav_params: str | None = None,
) -> list[dict[str, Any]]:
    """
    Return (package, executable, name, parameters) per node; ValueError on a bad argument.

    `nav_params`, the Nav2 parameter file, adds Nav2 and the drive bridge to
    localization.
    """
    specs = [
        *slam_specs(mode, localizer, map_name, pose_text, config_dir, resume, extend),
        autonomy_spec(
            mode,
            localizer,
            map_name,
            pose_text,
            session,
            config_dir,
            pose_source,
            bool(resume),
            resume or extend,
            # The stem's last part is the source map's name, saved or autosaved.
            os.path.basename(extend),
        ),
    ]
    if mode == 'localization' and nav_params:
        specs += nav_specs(config_dir, nav_params)
    return [*specs, foxglove_spec(config_dir)] if foxglove else specs


def slam_specs(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    config_dir: str,
    resume: str = '',
    extend: str = '',
) -> list[dict[str, Any]]:
    if mode not in MODES:
        raise ValueError(f'mode must be one of {", ".join(MODES)}: {mode!r}')
    if localizer not in LOCALIZERS:
        raise ValueError(f'localizer must be one of {", ".join(LOCALIZERS)}: {localizer!r}')
    search = pose_text.strip() == 'global'
    if search and (mode != 'localization' or localizer != 'amcl'):
        raise ValueError('pose:=global needs mode:=localization with localizer:=amcl')
    pose = None if search else parse_pose(pose_text)
    if (resume or extend) and mode != 'mapping':
        raise ValueError('resume:= and extend:= are for mode:=mapping')
    if resume and extend:
        raise ValueError('resume:= and extend:= exclude each other')
    if mode == 'mapping':
        files = _files(config_dir, 'slam_mapping.yaml')
        # A pose graph given at startup is loaded at map_start_pose, and the
        # first scan is matched against the saved scans near it; mapping then
        # continues. (LOCALIZE_AT_POSE, the other match type, stops adding to
        # the map.)
        loaded = resume or extend
        resumed = (
            [{'map_file_name': loaded, 'map_start_pose': list(pose or (0.0, 0.0, 0.0))}]
            if loaded
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
    mode = LaunchConfiguration('mode').perform(context)
    want_nav = LaunchConfiguration('nav').perform(context).lower() in ('1', 'true')
    nav_params = nav2_param_file(config_dir) if mode == 'localization' and want_nav else None
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
        LaunchConfiguration('extend').perform(context),
        nav_params,
    )
    # 'both': node output also goes to the launch's log directory.
    return [
        Node(**spec, output='both', on_exit=Shutdown(), **_stop_timeouts(spec)) for spec in specs
    ]


def _stop_timeouts(spec: dict[str, Any]) -> dict[str, Any]:
    """Return short stop timeouts for foxglove_bridge; it can ignore SIGINT with a client on."""
    if spec['name'] != 'foxglove_bridge':
        return {}
    return {'sigterm_timeout': '2', 'sigkill_timeout': '2'}


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
                'extend',
                default_value='',
                description='mapping: saved pose graph stem to extend, near pose',
            ),
            DeclareLaunchArgument(
                'foxglove',
                default_value='false',
                description='also start foxglove_bridge (port 8765)',
            ),
            DeclareLaunchArgument(
                'pose_source', default_value='', description='reported in /autonomy/status'
            ),
            DeclareLaunchArgument(
                'nav',
                default_value='true',
                description='localization: also start Nav2 and the drive bridge',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
