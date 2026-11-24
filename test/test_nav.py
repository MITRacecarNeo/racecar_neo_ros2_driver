"""Tests for Nav2 bringup: nav.py, nav_bridge_node.py, launch, racecar nav, configs."""

from pathlib import Path
import xml.etree.ElementTree as ET

from conftest import load_launch, load_script
import pytest
from racecar_neo_ros2_driver import nav
from racecar_neo_ros2_driver.steering import SteeringTable
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'
TABLE = SteeringTable(
    command=(-1.0, -0.5, 0.0, 0.5, 1.0),
    curvature=(-2.0, -1.0, 0.0, 1.0, 2.0),
    curvature_reverse=(-1.6, -0.8, 0.0, 0.8, 1.6),
)
FRAMES = {
    'footprint_front_x_mm': 287.5,
    'footprint_rear_x_mm': -59.7,
    'footprint_left_y_mm': 104.5,
    'footprint_right_y_mm': -104.5,
}


class TestSpeedMap:
    @pytest.mark.parametrize('mps', [0.0, 0.2, 0.49, -0.49])
    def test_under_the_floor_is_a_stop(self, mps):
        assert nav.SpeedMap().drive_speed(mps) == 0.0

    def test_floor_engages_past_the_deadband(self):
        m = nav.SpeedMap()
        assert m.drive_speed(0.5) > m.deadband and m.drive_speed(-0.5) < -m.deadband

    @pytest.mark.parametrize('mps', [0.6, 0.75, 0.9, 1.0, -0.6, -0.8])
    def test_round_trip_through_the_firmware_band(self, mps):
        m = nav.SpeedMap()
        assert m.speed_of(m.drive_speed(mps)) == pytest.approx(mps, abs=1e-6)

    def test_top_speed_clamps(self):
        assert nav.SpeedMap().drive_speed(3.0) == 1.0

    def test_throttle_caps_are_inverted(self):
        m = nav.SpeedMap(cap_forward=0.5, cap_backward=0.8)
        d = m.drive_speed(0.6)
        assert m.speed_of(d) == pytest.approx(0.6) and d > nav.SpeedMap().drive_speed(0.6)
        # A speed the cap cannot reach is the most /drive can ask.
        assert m.drive_speed(1.0) == 1.0


class TestSteering:
    def test_curvature_through_the_table(self):
        # 0.8 m/s at 0.8 rad/s is 1/m: command 0.5 forward.
        assert nav.drive_steering(0.8, 0.8, TABLE) == pytest.approx(0.5)

    def test_reverse_uses_the_reverse_table(self):
        # Curvature 0.8/m in reverse: command 0.5 on the reverse table.
        assert nav.drive_steering(-0.6, -0.48, TABLE) == pytest.approx(0.5)

    def test_standing_still_is_straight(self):
        assert nav.drive_steering(0.0, 1.0, TABLE) == 0.0

    def test_past_full_lock_clamps(self):
        assert nav.drive_steering(0.5, 5.0, TABLE) == 1.0

    def test_turning_radius_holds_in_every_direction(self):
        # Reverse full lock (1.6/m) is the widest of the four: the planner uses it.
        assert nav.min_turning_radius(TABLE) == pytest.approx(0.625)


class TestNav2Params:
    def test_footprint_and_radius_set_base_untouched(self):
        base = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
        tree = nav.nav2_params(base, TABLE, FRAMES)
        local = tree['local_costmap']['local_costmap']['ros__parameters']['footprint']
        assert (
            local == '[[0.2875, 0.1045], [0.2875, -0.1045], [-0.0597, -0.1045], [-0.0597, 0.1045]]'
        )
        assert tree['global_costmap']['global_costmap']['ros__parameters']['footprint'] == local
        assert (
            tree['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
            == 0.625
        )
        assert 'footprint' not in base['local_costmap']['local_costmap']['ros__parameters']

    def test_merge_is_deep(self):
        out = nav.merge({'a': {'b': 1, 'c': 2}}, {'a': {'c': 3}, 'd': 4})
        assert out == {'a': {'b': 1, 'c': 3}, 'd': 4}


class TestConfigs:
    def test_planner_and_controller(self):
        cfg = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
        planner = cfg['planner_server']['ros__parameters']['GridBased']
        assert planner['plugin'] == 'nav2_smac_planner::SmacPlannerHybrid'
        assert planner['motion_model_for_search'] == 'REEDS_SHEPP'
        rpp = cfg['controller_server']['ros__parameters']['FollowPath']
        assert rpp['allow_reversing'] and not rpp['use_rotate_to_heading']
        assert rpp['desired_linear_vel'] == 1.0
        # The slowest moving speeds sit at the firmware floor.
        assert rpp['min_approach_linear_velocity'] == 0.5
        assert rpp['regulated_linear_scaling_min_speed'] == 0.5
        assert (
            cfg['local_costmap']['local_costmap']['ros__parameters']['footprint_padding'] == 0.05
        )

    def test_no_spin_or_backup(self):
        cfg = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
        assert cfg['behavior_server']['ros__parameters']['behavior_plugins'] == ['wait']
        tree = ET.parse(CONFIG / 'nav2_bt.xml')
        tags = {el.tag for el in tree.iter()}
        assert {'ComputePathToPose', 'FollowPath', 'Wait'} <= tags
        assert not tags & {'Spin', 'BackUp', 'DriveOnHeading'}

    def test_lifecycle_nodes_match_the_launch(self):
        cfg = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
        names = cfg['lifecycle_manager_navigation']['ros__parameters']['node_names']
        autonomy = load_launch('autonomy')
        assert names == list(autonomy.NAV2_NODES)

    def test_setup_installs_nav2(self):
        text = (ROOT / 'scripts' / 'setup_ros2.sh').read_text()
        for pkg in ('planner', 'controller', 'bt-navigator', 'behaviors', 'costmap-2d',
                    'smac-planner', 'regulated-pure-pursuit-controller',
                    'simple-commander'):  # fmt: skip
            assert f'ros-jazzy-nav2-{pkg} \\' in text, pkg


class TestBridge:
    @pytest.fixture
    def bridge(self):
        from racecar_neo_ros2_driver import nav_bridge_node

        return nav_bridge_node.Bridge(nav.SpeedMap(), TABLE, timeout=0.5, stop_messages=3)

    def test_quiet_until_nav2_commands(self, bridge):
        assert bridge.tick(0.0) is None

    def test_drives_only_while_localized(self, bridge):
        bridge.on_command(0.8, 0.8, 0.0)
        assert bridge.tick(0.1) is None
        bridge.localized = True
        speed, steer = bridge.tick(0.1)
        assert speed == pytest.approx(nav.SpeedMap().drive_speed(0.8)) and steer == pytest.approx(
            0.5
        )

    def test_stale_command_stops_then_goes_quiet(self, bridge):
        bridge.localized = True
        bridge.on_command(0.8, 0.0, 0.0)
        bridge.tick(0.1)
        assert [bridge.tick(1.0 + i * 0.05) for i in range(4)] == [(0.0, 0.0)] * 3 + [None]

    def test_lost_localization_stops(self, bridge):
        bridge.localized = True
        bridge.on_command(0.8, 0.0, 0.0)
        bridge.tick(0.1)
        bridge.localized = False
        assert bridge.tick(0.2) == (0.0, 0.0)

    def test_speed_map_from_throttle(self):
        from racecar_neo_ros2_driver import nav_bridge_node

        params = {'firmware_deadband': 0.01, 'firmware_min_mps': 0.5, 'firmware_max_mps': 1.0}
        m = nav_bridge_node.speed_map(params, {'max_speed_forward': 0.5})
        assert (m.cap_forward, m.cap_backward) == (0.5, 1.0)

    def test_bridge_config(self):
        cfg = yaml.safe_load((CONFIG / 'nav_bridge.yaml').read_text())['nav_bridge_node']
        p = cfg['ros__parameters']
        assert (p['firmware_min_mps'], p['firmware_max_mps'], p['firmware_deadband']) == (
            0.5,
            1.0,
            0.01,
        )


class TestLaunch:
    @pytest.fixture(scope='class')
    def autonomy(self):
        return load_launch('autonomy')

    def test_localization_adds_nav2_and_the_bridge(self, autonomy):
        specs = autonomy.node_specs(
            'localization', 'amcl', 'room', '', str(CONFIG), nav_params='p.yaml'
        )
        names = [s['name'] for s in specs]
        assert names[-6:] == ['controller_server', 'planner_server', 'behavior_server',
                              'bt_navigator',
                              'lifecycle_manager_navigation', 'nav_bridge_node']  # fmt: skip
        controller = next(s for s in specs if s['name'] == 'controller_server')
        assert controller['remappings'] == [('cmd_vel', '/nav/cmd_vel')]
        assert controller['parameters'] == ['p.yaml']

    def test_mapping_and_no_params_leave_nav2_out(self, autonomy):
        for mode, params in (('mapping', 'p.yaml'), ('localization', None)):
            specs = autonomy.node_specs(mode, 'amcl', 'room', '', str(CONFIG), nav_params=params)
            assert 'nav_bridge_node' not in [s['name'] for s in specs]

    def test_param_file_needs_a_steering_table(self, autonomy, tmp_path, monkeypatch):
        from racecar_neo_ros2_driver import steering

        monkeypatch.setattr(steering, 'load', lambda: None)
        assert autonomy.nav2_param_file(str(CONFIG), str(tmp_path)) is None
        monkeypatch.setattr(steering, 'load', lambda: TABLE)
        path = autonomy.nav2_param_file(str(CONFIG), str(tmp_path))
        tree = yaml.safe_load(Path(path).read_text())
        assert tree['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'].endswith(
            'nav2_bt.xml'
        )
        assert (
            tree['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
            == 0.625
        )


class TestCli:
    @pytest.fixture(scope='class')
    def cli(self):
        return load_script('racecar_nav')

    @pytest.mark.parametrize('deg, z, w', [(0, 0.0, 1.0), (90, 0.7071, 0.7071), (180, 1.0, 0.0)])
    def test_heading_to_quaternion(self, cli, deg, z, w):
        qz, qw = cli.quaternion_z_w(deg)
        assert (qz, qw) == (pytest.approx(z, abs=1e-4), pytest.approx(w, abs=1e-4))

    def test_parse(self, cli):
        args = cli.parse(['goal', '1.5', '-2', '90', '--no-wait'])
        assert (args.command, args.x, args.y, args.heading, args.no_wait) == (
            'goal',
            1.5,
            -2.0,
            90.0,
            True,
        )
        assert cli.parse(['cancel']).command == 'cancel'
        with pytest.raises(SystemExit):
            cli.parse(['goal', '1'])

    def test_move_is_along_the_heading(self, cli):
        assert cli.ahead_of(1.0, 2.0, 90.0, 3.0) == (pytest.approx(1.0), pytest.approx(5.0))
        assert cli.ahead_of(0.0, 0.0, 0.0, -2.0) == (pytest.approx(-2.0), pytest.approx(0.0))
        args = cli.parse(['move', '-1.5'])
        assert (args.command, args.dist) == ('move', -1.5)

    def test_in_the_racecar_tool(self):
        text = (ROOT / 'scripts' / 'racecar-tool.sh').read_text()
        assert 'scripts/racecar_nav.py' in text and 'nav <action>' in text


def test_behavior_tree_is_installed():
    text = (ROOT / 'setup.py').read_text()
    assert "glob.glob('config/*.xml')" in text


def test_inflation_covers_the_padded_footprint():
    import math

    cfg = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
    frames = yaml.safe_load((CONFIG / 'frames.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        p = cfg[costmap][costmap]['ros__parameters']
        radius = max(math.hypot(x, y) for x, y in nav.footprint(frames)) + p['footprint_padding']
        assert p['inflation_layer']['inflation_radius'] >= radius, costmap
