"""Unit tests for launch/autonomy.launch.py, maps.py, and the SLAM configs."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import pytest
from racecar_neo_ros2_driver import maps
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'


@pytest.fixture(scope='module')
def autonomy():
    spec = importlib.util.spec_from_file_location(
        'autonomy_launch', ROOT / 'launch' / 'autonomy.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _dicts(spec):
    out = {}
    for p in spec['parameters']:
        if isinstance(p, dict):
            out.update(p)
    return out


class TestMaps:
    def test_name_resolves_under_the_root(self, tmp_path):
        assert maps.map_stem('room', tmp_path) == tmp_path / 'room' / 'room'

    def test_root_from_the_environment(self, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        assert maps.map_stem('room') == tmp_path / 'room' / 'room'

    def test_default_root_is_data_maps(self, monkeypatch):
        monkeypatch.delenv(maps.ROOT_ENV, raising=False)
        assert maps.map_root() == Path.home() / 'data' / 'maps'

    def test_path_drops_a_map_suffix(self):
        assert maps.map_stem('/tmp/a/room.yaml') == Path('/tmp/a/room')
        assert maps.map_stem('/tmp/a/room') == Path('/tmp/a/room')

    @pytest.mark.parametrize('name', ['room', 'hall_2', 'a-b'])
    def test_valid_names(self, name):
        assert maps.valid_name(name)

    @pytest.mark.parametrize('name', ['', 'a b', '../x', 'a/b'])
    def test_invalid_names(self, name):
        assert not maps.valid_name(name)

    def test_pose(self):
        assert maps.parse_pose('1, -2.5, 1.57') == (1.0, -2.5, 1.57)
        assert maps.parse_pose('') is None

    @pytest.mark.parametrize('text', ['1,2', '1,2,3,4', 'a,b,c', 'nan,0,0'])
    def test_bad_pose(self, text):
        with pytest.raises(ValueError):
            maps.parse_pose(text)


class TestLaunch:
    def test_mapping_runs_slam_toolbox_async(self, autonomy):
        node, manager = autonomy.slam_specs('mapping', 'amcl', '', '', str(CONFIG))
        assert node['executable'] == 'async_slam_toolbox_node'
        assert node['parameters'] == [str(CONFIG / 'slam_mapping.yaml')]
        assert manager['name'] == 'lifecycle_manager_slam'

    def test_amcl_localization(self, autonomy, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', '', str(CONFIG))
        assert [n['name'] for n in nodes] == [
            'map_server',
            'amcl',
            'lifecycle_manager_localization',
        ]
        assert _dicts(nodes[0])['yaml_filename'] == str(tmp_path / 'room' / 'room.yaml')
        p = _dicts(nodes[1])
        assert p['set_initial_pose'] is True
        assert (p['initial_pose.x'], p['initial_pose.y'], p['initial_pose.yaw']) == (0, 0, 0)

    def test_amcl_start_pose(self, autonomy):
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', '1,2,0.5', str(CONFIG))
        p = _dicts(nodes[1])
        assert p['set_initial_pose'] is True
        assert (p['initial_pose.x'], p['initial_pose.y'], p['initial_pose.yaw']) == (1.0, 2.0, 0.5)

    def test_slam_toolbox_localization(self, autonomy, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        node, _ = autonomy.slam_specs('localization', 'slam_toolbox', 'room', '', str(CONFIG))
        assert node['executable'] == 'localization_slam_toolbox_node'
        p = _dicts(node)
        assert p['map_file_name'] == str(tmp_path / 'room' / 'room')
        assert p['map_start_pose'] == [0.0, 0.0, 0.0]

    @pytest.mark.parametrize(
        'args',
        [
            ('drive', 'amcl', '', ''),
            ('localization', 'gmapping', 'room', ''),
            ('localization', 'amcl', '', ''),
            ('localization', 'amcl', 'room', '1,2'),
        ],
    )
    def test_bad_arguments(self, autonomy, args):
        with pytest.raises(ValueError):
            autonomy.slam_specs(*args, str(CONFIG))

    def test_local_override_loads_last(self, autonomy, tmp_path):
        for name in ('slam_mapping.yaml', 'slam_mapping.local.yaml'):
            (tmp_path / name).write_text('slam_toolbox: {ros__parameters: {}}\n')
        node, _ = autonomy.slam_specs('mapping', 'amcl', '', '', str(tmp_path))
        assert [Path(p).name for p in node['parameters']] == [
            'slam_mapping.yaml',
            'slam_mapping.local.yaml',
        ]

    def test_launch_description_declares_the_arguments(self, autonomy):
        ld = autonomy.generate_launch_description()
        names = [e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)]
        assert names == [
            'mode',
            'localizer',
            'map',
            'pose',
            'session',
            'resume',
            'extend',
            'foxglove',
            'pose_source',
        ]


class TestAutonomyNodeSpec:
    def test_every_mode_adds_autonomy_node(self, autonomy):
        for args in (('mapping', 'amcl', 'room', ''), ('localization', 'amcl', 'room', '')):
            assert autonomy.node_specs(*args, str(CONFIG))[-1]['name'] == 'autonomy_node'

    def test_autonomy_node_parameters(self, autonomy):
        spec = autonomy.node_specs(
            'localization', 'amcl', 'room', '1,2,0', str(CONFIG), session='s1'
        )[-1]
        assert spec['parameters'][0] == str(CONFIG / 'autonomy.yaml')
        p = _dicts(spec)
        assert (p['mode'], p['map'], p['session'], p['pose_source']) == (
            'localization',
            'room',
            's1',
            'pose',
        )

    @pytest.mark.parametrize(
        'pose,source', [('', 'origin'), ('global', 'global'), ('1,2,0', 'pose')]
    )
    def test_pose_source(self, autonomy, pose, source):
        spec = autonomy.node_specs('localization', 'amcl', 'room', pose, str(CONFIG))[-1]
        assert _dicts(spec)['pose_source'] == source

    def test_global_starts_amcl_without_a_pose(self, autonomy):
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', 'global', str(CONFIG))
        assert _dicts(nodes[1])['set_initial_pose'] is False

    @pytest.mark.parametrize(
        'mode,localizer', [('mapping', 'amcl'), ('localization', 'slam_toolbox')]
    )
    def test_global_needs_amcl_localization(self, autonomy, mode, localizer):
        with pytest.raises(ValueError):
            autonomy.slam_specs(mode, localizer, 'room', 'global', str(CONFIG))


class TestResumeAndFoxglove:
    def test_resume_loads_the_autosave_at_the_pose(self, autonomy):
        specs = autonomy.node_specs(
            'mapping',
            'amcl',
            'room',
            '1,2,0.5',
            str(CONFIG),
            session='s',
            resume='/m/room/autosave/room',
        )
        slam = _dicts(specs[0])
        assert slam == {
            'map_file_name': '/m/room/autosave/room',
            'map_start_pose': [1.0, 2.0, 0.5],
        }
        node = _dicts(specs[-1])
        assert node['resumed_from_autosave'] is True and node['pose_source'] == 'autosave'

    def test_fresh_mapping_loads_no_graph(self, autonomy):
        specs = autonomy.node_specs('mapping', 'amcl', 'room', '', str(CONFIG))
        assert _dicts(specs[0]) == {} and _dicts(specs[-1])['resumed_from_autosave'] is False

    def test_resume_is_for_mapping(self, autonomy):
        with pytest.raises(ValueError):
            autonomy.slam_specs('localization', 'amcl', 'room', '', str(CONFIG), resume='/x')

    def test_foxglove_on_demand(self, autonomy):
        assert 'foxglove_bridge' not in [
            s['name'] for s in autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG))
        ]
        specs = autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG), foxglove=True)
        assert specs[-1]['parameters'][0] == str(CONFIG / 'foxglove.yaml')

    def test_pose_source_given(self, autonomy):
        spec = autonomy.node_specs(
            'localization', 'amcl', 'r', '1,2,0', str(CONFIG), pose_source='last_pose'
        )[-1]
        assert _dicts(spec)['pose_source'] == 'last_pose'


class TestConfigs:
    @pytest.mark.parametrize('name', ['slam_mapping.yaml', 'slam_localization.yaml'])
    def test_slam_toolbox_frames_and_ranges(self, name):
        p = yaml.safe_load((CONFIG / name).read_text())['slam_toolbox']['ros__parameters']
        assert (p['base_frame'], p['odom_frame'], p['map_frame']) == ('base_link', 'odom', 'map')
        assert p['min_laser_range'] == 0.3 and p['max_laser_range'] == 12.0
        assert p['mode'] == name.split('_')[1].removesuffix('.yaml')

    @pytest.mark.parametrize('name', ['slam_mapping.yaml', 'slam_localization.yaml'])
    def test_slam_toolbox_runs_under_the_lifecycle_manager(self, name):
        data = yaml.safe_load((CONFIG / name).read_text())
        assert data['slam_toolbox']['ros__parameters']['use_lifecycle_manager'] is True
        manager = data['lifecycle_manager_slam']['ros__parameters']
        assert manager == {'autostart': True, 'node_names': ['slam_toolbox']}

    @pytest.mark.parametrize('name', ['slam_mapping.yaml', 'slam_localization.yaml'])
    def test_scan_matching_weights_the_odometry(self, name):
        p = yaml.safe_load((CONFIG / name).read_text())['slam_toolbox']['ros__parameters']
        assert p['angle_variance_penalty'] <= 0.05 and p['minimum_angle_penalty'] <= 0.5
        assert p['distance_variance_penalty'] <= 0.1

    def test_amcl(self):
        data = yaml.safe_load((CONFIG / 'amcl.yaml').read_text())
        p = data['amcl']['ros__parameters']
        assert p['base_frame_id'] == 'base_link' and p['laser_min_range'] == 0.3
        assert p['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'
        assert all(p[f'alpha{i}'] <= 0.05 for i in range(1, 5))
        assert p['sigma_hit'] <= 0.1 and p['z_hit'] >= 0.9 and p['z_rand'] <= 0.1
        manager = data['lifecycle_manager_localization']['ros__parameters']
        assert manager['node_names'] == ['map_server', 'amcl']


class TestExtend:
    def test_extend_loads_the_saved_graph_near_the_pose(self, autonomy):
        specs = autonomy.node_specs(
            'mapping', 'amcl', 'hall', '1,2,0.5', str(CONFIG), session='s', extend='/m/room/room'
        )
        assert _dicts(specs[0]) == {
            'map_file_name': '/m/room/room',
            'map_start_pose': [1.0, 2.0, 0.5],
        }
        node = _dicts(specs[-1])
        assert node['resumed_from_autosave'] is False and node['pose_source'] == 'pose'
        assert node['source'] == '/m/room/room' and node['extended_from'] == 'room'

    def test_extend_an_autosave_names_the_map(self, autonomy):
        spec = autonomy.node_specs(
            'mapping', 'amcl', 'hall', '0,0,0', str(CONFIG), extend='/m/room/autosave/room'
        )[-1]
        assert _dicts(spec)['extended_from'] == 'room'

    def test_resume_carries_from_the_autosave(self, autonomy):
        spec = autonomy.node_specs(
            'mapping', 'amcl', 'room', '0,0,0', str(CONFIG), resume='/m/room/autosave/room'
        )[-1]
        assert _dicts(spec)['source'] == '/m/room/autosave/room'
        assert _dicts(spec)['extended_from'] == ''

    @pytest.mark.parametrize(
        'mode,resume', [('localization', ''), ('mapping', '/m/room/autosave/room')]
    )
    def test_bad_extend(self, autonomy, mode, resume):
        with pytest.raises(ValueError):
            autonomy.slam_specs(mode, 'amcl', 'room', '', str(CONFIG), resume, '/m/room/room')


def test_foxglove_stops_quickly(autonomy):
    specs = autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG), foxglove=True)
    assert autonomy._stop_timeouts(specs[-1]) == {'sigterm_timeout': '2', 'sigkill_timeout': '2'}
    assert autonomy._stop_timeouts(specs[0]) == {}
