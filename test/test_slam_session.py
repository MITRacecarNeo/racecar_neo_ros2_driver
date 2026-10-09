"""Unit tests for scripts/validation/slam_session.py (no ROS graph, no processes)."""

from datetime import datetime
import math

from conftest import load_script
import pytest
import yaml


@pytest.fixture(scope='module')
def ss():
    return load_script('slam_session')


class TestCommands:
    def test_teleop_turns_edgetpu_off(self, ss):
        argv = ss.teleop_argv(camera=True)
        assert argv[-1] == 'edgetpu_enable:=false' and argv[1].endswith('launch_teleop.sh')

    def test_teleop_without_camera(self, ss):
        assert ss.teleop_argv(camera=False)[-1] == 'realsense_enable:=false'

    def test_mapping_caps_the_speed(self, ss):
        argv = ss.teleop_argv(camera=True, throttle_config=ss.MAPPING_THROTTLE)
        assert argv[-1] == f'throttle_config:={ss.MAPPING_THROTTLE}'
        p = yaml.safe_load(ss.MAPPING_THROTTLE.read_text())['throttle_node']['ros__parameters']
        assert p['max_speed_forward'] == p['max_speed_backward'] == 0.02
        assert p['max_steering'] == 1.0

    def test_autonomy_mapping(self, ss):
        assert ss.autonomy_argv('mapping', 'room', 'amcl', '')[-1] == 'mode:=mapping'

    def test_autonomy_localization(self, ss):
        argv = ss.autonomy_argv('localization', 'room', 'slam_toolbox', '1,2,0')
        assert argv[-3:] == ['map:=room', 'localizer:=slam_toolbox', 'pose:=1,2,0']

    def test_bag_records_the_slam_topics(self, ss, tmp_path):
        argv = ss.bag_argv(tmp_path / 'bag')
        assert argv[:3] == ['ros2', 'bag', 'record'] and '/tf_static' in argv and '/map' in argv
        assert not any(t.startswith('/camera') for t in argv)

    def test_foxglove_whitelist_has_no_camera(self, ss):
        params = yaml.safe_load(ss.FOXGLOVE_PARAMS.read_text())['foxglove_bridge']
        topics = params['ros__parameters']['topic_whitelist']
        assert '^/map$' in topics and not any('camera' in t for t in topics)

    def test_session_name(self, ss):
        now = datetime(2026, 10, 8, 14, 5, 9)
        assert ss.session_dir_name('map', 'room', now) == '20261008-140509-map-room'


class TestDisk:
    GB = 1_000_000_000

    def test_ok(self, ss):
        assert ss.disk_status(58 * self.GB, 23 * self.GB, 33 * self.GB)[0] == 'ok'

    def test_warns_when_mostly_used(self, ss):
        level, text = ss.disk_status(58 * self.GB, 47 * self.GB, 11 * self.GB)
        assert level == 'warn' and '81% used' in text

    def test_warns_below_ten_gb(self, ss):
        assert ss.disk_status(200 * self.GB, 100 * self.GB, 9 * self.GB)[0] == 'warn'

    def test_refuses_below_three_gb(self, ss):
        assert ss.disk_status(58 * self.GB, 56 * self.GB, 2 * self.GB)[0] == 'refuse'

    def test_largest_entries(self, ss, tmp_path):
        (tmp_path / 'big').mkdir()
        (tmp_path / 'big' / 'a').write_bytes(b'x' * 300)
        (tmp_path / 'small.log').write_bytes(b'x' * 10)
        assert [p.name for _, p in ss.largest_entries(tmp_path)] == ['big', 'small.log']


class TestAnalysis:
    def test_max_gap_only_counts_the_save_window(self, ss):
        times = [0.0, 0.1, 0.2, 2.0, 2.1, 5.0, 5.1]
        assert ss.max_gap(times, 0.15, 2.05) == pytest.approx(1.8)
        assert ss.max_gap([1.0], 0, 2) == 0.0

    def test_drift(self, ss):
        map_odom = [(0.0, 0, 0, 0), (1.0, 0.01, 0, 0), (2.0, 0.11, 0, math.radians(2))]
        odom = [(0.0, 0, 0), (1.0, 3, 0), (2.0, 3, 4)]
        d = ss.drift_summary(map_odom, odom)
        assert d.path_m == pytest.approx(7.0)
        assert d.largest_jump_m == pytest.approx(0.1) and d.largest_jump_t == pytest.approx(2.0)
        assert d.largest_jump_deg == pytest.approx(2.0)
        assert d.final[0] == pytest.approx(0.11)

    def test_no_drift_without_transforms(self, ss):
        assert ss.drift_summary([], [(0, 0, 0)]) is None

    def test_loop_lines(self, ss):
        assert ss.loop_lines('a\n[slam] Loop closure found\nb') == ['[slam] Loop closure found']

    def test_load_csv_round_trip(self, ss, tmp_path):
        ev = load_script('slam_eval')
        rec = object.__new__(ev.LoadRecorder)
        rec.samples = [ev.Sample(1.0, 40.0, 60.5, 0, 2400, {'slam_toolbox': 5.0})]
        rec.write_csv(tmp_path / 'load.csv')
        (s,) = ss.read_load_csv(tmp_path / 'load.csv')
        assert (s.t, s.cpu_busy, s.temp_c, s.throttled, s.arm_mhz) == (1.0, 40.0, 60.5, 0, 2400)
        assert s.per_process == {'slam_toolbox': 5.0}

    def test_report_for_a_mapping_session(self, ss, tmp_path, monkeypatch):
        from PIL import Image

        maps = tmp_path / 'maps' / 'room'
        maps.mkdir(parents=True)
        Image.new('L', (4, 2), 254).save(maps / 'room.pgm')
        (maps / 'room.yaml').write_text('image: room.pgm\nresolution: 0.05\n')
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path / 'maps'))
        session = tmp_path / 'session'
        (session / 'logs').mkdir(parents=True)
        (session / 'logs' / 'autonomy.log').write_text('x\nLoop closure ok\n')
        (session / 'session.yaml').write_text(
            yaml.safe_dump({'mode': 'mapping', 'map': 'room', 'camera': True, 'git': {}})
        )
        (session / 'stalls.csv').write_text('n,call_s\n1,0.80\n')
        report = ss.analyze(session).read_text()
        assert '## Load' in report and 'No load record.' in report
        assert 'free 8 (0.0 m^2)' in report and (session / 'map.png').exists()
        assert '1 log lines mention a loop.' in report
        assert '1,0.80' in report


class TestReplay:
    def test_overrides_are_yaml_values(self, ss):
        got = ss.parse_overrides(['angle_variance_penalty=0.03', 'do_loop_closing=false'])
        assert got == {'angle_variance_penalty': 0.03, 'do_loop_closing': False}

    @pytest.mark.parametrize('item', ['novalue', '=1'])
    def test_bad_override(self, ss, item):
        with pytest.raises(ValueError):
            ss.parse_overrides([item])

    def test_commands(self, ss, tmp_path):
        slam, manager, play = ss.replay_argv(
            tmp_path / 'c.yaml', tmp_path / 'o.yaml', tmp_path / 'bag'
        )
        assert slam[-1] == str(tmp_path / 'o.yaml') and '__node:=slam_toolbox' in slam
        assert '__node:=lifecycle_manager_slam' in manager
        assert play[3] == str(tmp_path / 'bag') and play[-2:] == ['--remap', '/tf:=/tf_bag']
        assert '/map' not in play  # slam_toolbox rebuilds the map

    def test_drift_spread(self, ss):
        d = ss.drift_summary(
            [(0.0, 0, 0, 0), (1.0, 0.3, 0.4, math.radians(8)), (2.0, 0, 0, 0)], [(0, 0, 0)]
        )
        assert d.max_offset_m == pytest.approx(0.5) and d.max_yaw_deg == pytest.approx(8)
        assert d.yaw_sd_deg > 3
