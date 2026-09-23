"""Unit tests for scripts/diagnose.py (the `racecar status` diagnostic)."""

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

from conftest import load_script
import pytest

SCRIPT = Path(__file__).resolve().parent.parent / 'scripts' / 'diagnose.py'


@pytest.fixture(scope='module')
def diag():
    return load_script('diagnose')


def _run(*args, timeout=60):
    return subprocess.run(
        ['python3', str(SCRIPT), *args],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


class TestTopicSpecs:
    def test_floor_is_a_fraction_of_nominal(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        assert spec.floor == pytest.approx(80.0)

    def test_sensors_default_to_eighty_percent(self, diag):
        pit = {
            '/imu/lsm9ds1',
            '/mag',
            '/encoder/speed',
            '/battery/voltage',
            '/battery/current',
            '/rc/channels',
        }
        for spec in diag.SENSOR_TOPICS:
            if spec.topic in pit:
                continue
            assert spec.floor_frac == 0.8, f'{spec.topic} should use the 80% floor'

    def test_pit_topics_carry_the_wider_floor(self, diag):
        # The six topics off the Teensy telemetry frame slow under graph load,
        # so their floor leaves room for it. A halved frame rate (68 Hz) must
        # still fail, or the check stops meaning anything.
        pit = [s for s in diag.SENSOR_TOPICS if s.nominal == 136.0 and 'Teensy' in s.note]
        assert len(pit) == 6
        for spec in pit:
            assert spec.floor_frac == diag.PIT_FLOOR_FRAC
            assert spec.floor < 90.0, 'floor must pass a car delivering 90 Hz'
            assert spec.floor > 68.0, 'floor must still catch a halved frame rate'

    def test_lidar_nominal_matches_the_delivered_rate(self, diag):
        scan = next(s for s in diag.SENSOR_TOPICS if s.topic == '/scan')
        assert scan.nominal == pytest.approx(7.2)
        assert scan.floor < 6.2, 'an ordinary dip must not fail a healthy lidar'
        assert scan.floor > 2.0, 'a lidar desynced to 2 Hz must still fail'

    def test_camera_nominal_is_the_configured_rate(self, diag):
        # The camera node reports 59.0 and 29.6 against these on /diagnostics.
        by_topic = {s.topic: s for s in diag.SENSOR_TOPICS}
        assert by_topic['/camera/color'].nominal == 60.0
        assert by_topic['/camera/depth'].nominal == 30.0

    def test_realsense_rates_come_from_diagnostics(self, diag):
        # Subscribing to the image streams would starve the PIT telemetry rate.
        assert diag.DIAGNOSTIC_SOURCED == {'/camera/color', '/camera/depth', '/imu/realsense'}
        for topic in diag.DIAGNOSTIC_SOURCED:
            spec = next(s for s in diag.SENSOR_TOPICS if s.topic == topic)
            assert 'diagnostics' in spec.note

    def test_every_notebook_topic_is_covered(self, diag):
        covered = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        for topic in (
            '/camera/color',
            '/camera/depth',
            '/scan',
            '/imu/lsm9ds1',
            '/mag',
            '/imu/fused',
            '/encoder/speed',
            '/battery/voltage',
            '/battery/current',
            '/rc/channels',
            '/edgetpu/inference',
            '/joy',
            '/motor',
            '/mux_out',
        ):
            assert topic in covered, f'{topic} lost from the check set'

    def test_value_topics_are_a_subset_of_sampled_topics(self, diag):
        sampled = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        assert diag.VALUE_TOPICS <= sampled


class TestRateChecks:
    def _ros(self, diag, counts, elapsed=2.0):
        return diag.RosResult(available=True, counts=counts, elapsed=elapsed, present=set(counts))

    def test_rate_above_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 200})  # 100 Hz over 2 s
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_rate_below_floor_fails(self, diag):
        # A desynced lidar still advertises its topic; only the rate floor catches it.
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 20})  # 10 Hz
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.FAIL
        assert '10.0 Hz' in check.detail

    def test_rate_exactly_at_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 160})  # 80 Hz
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_absent_topic_fails(self, diag):
        spec = diag.TopicSpec('/missing', 'M', 10.0)
        ros = diag.RosResult(available=True, counts={}, elapsed=2.0, present=set())
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.FAIL
        assert 'not published' in check.detail

    def test_no_ros_graph_skips_rather_than_fails(self, diag):
        spec = diag.TopicSpec('/t', 'T', 10.0)
        ros = diag.RosResult(available=False, reason='no ROS graph visible')
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.SKIP
        assert 'no ROS graph' in check.detail

    def test_note_is_carried_into_the_detail(self, diag):
        spec = diag.TopicSpec('/t', 'T', 10.0, 0.5, 'configured 60, gap tracked')
        ros = self._ros(diag, {'/t': 40})
        assert 'gap tracked' in diag.rate_checks('sensors', [spec], ros)[0].detail

    def test_diagnostic_sourced_topic_uses_the_reported_rate(self, diag):
        # /camera/color is never counted here, so a zero count must not read
        # as a dead stream.
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/color')
        ros = diag.RosResult(
            available=True,
            counts={},
            elapsed=5.0,
            present={'/camera/color'},
            reported={'/camera/color': 59.0},
        )
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.OK
        assert '59.0 Hz' in check.detail

    def test_diagnostic_sourced_topic_below_floor_fails(self, diag):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/depth')
        ros = diag.RosResult(
            available=True,
            counts={},
            elapsed=5.0,
            present={'/camera/depth'},
            reported={'/camera/depth': 12.0},
        )
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_missing_diagnostic_rate_warns_rather_than_reading_zero(self, diag):
        # A DiagnosticArray that never arrived says nothing about the camera.
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/color')
        ros = diag.RosResult(
            available=True, counts={}, elapsed=5.0, present={'/camera/color'}, reported={}
        )
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.WARN
        assert '/diagnostics' in check.detail


class TestSampleWindow:
    def test_window_is_long_enough_for_the_pit_stream(self, diag):
        # Shorter windows measure the ~136 Hz PIT stream too noisily to judge.
        assert diag.DEFAULT_WINDOW >= 5.0

    def test_diagnostic_names_cover_every_diagnostic_sourced_topic(self, diag):
        assert set(diag.REALSENSE_DIAGNOSTIC_NAMES.values()) == diag.DIAGNOSTIC_SOURCED


class TestValueChecks:
    def _ros_with(self, diag, values):
        return diag.RosResult(available=True, elapsed=2.0, values=values)

    def _imu(self, x, y, z):
        return SimpleNamespace(linear_acceleration=SimpleNamespace(x=x, y=y, z=z))

    def test_gravity_magnitude_at_rest_passes(self, diag):
        ros = self._ros_with(diag, {'/imu/lsm9ds1': self._imu(0.0, 0.0, 9.81)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'IMU magnitude')
        assert check.status == diag.OK

    def test_implausible_gravity_fails(self, diag):
        ros = self._ros_with(diag, {'/imu/lsm9ds1': self._imu(0.0, 0.0, 0.2)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'IMU magnitude')
        assert check.status == diag.FAIL

    def test_pack_voltage_in_range_passes(self, diag):
        ros = self._ros_with(
            diag,
            {
                '/battery/voltage': SimpleNamespace(data=11.4),
                '/battery/current': SimpleNamespace(data=2.0),
            },
        )
        check = next(c for c in diag.value_checks(ros) if c.name == 'Pack voltage range')
        assert check.status == diag.OK

    def test_pack_voltage_out_of_range_fails(self, diag):
        ros = self._ros_with(
            diag,
            {
                '/battery/voltage': SimpleNamespace(data=2.0),
                '/battery/current': SimpleNamespace(data=1.0),
            },
        )
        check = next(c for c in diag.value_checks(ros) if c.name == 'Pack voltage range')
        assert check.status == diag.FAIL

    def test_lidar_sample_count(self, diag):
        ros = self._ros_with(diag, {'/scan': SimpleNamespace(ranges=[0.0] * 1080)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'LIDAR samples')
        assert check.status == diag.OK

    def test_unexpected_lidar_sample_count_warns(self, diag):
        # The sim publishes 720: worth surfacing, not a hardware fault.
        ros = self._ros_with(diag, {'/scan': SimpleNamespace(ranges=[0.0] * 720)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'LIDAR samples')
        assert check.status == diag.WARN

    def test_rc_channel_count(self, diag):
        ros = self._ros_with(diag, {'/rc/channels': SimpleNamespace(data=[0.0] * 8)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'RC channels')
        assert check.status == diag.OK

    def test_missing_samples_skip(self, diag):
        checks = diag.value_checks(diag.RosResult(available=True, elapsed=2.0, values={}))
        assert all(c.status == diag.SKIP for c in checks)

    def test_no_ros_skips_every_value_check(self, diag):
        checks = diag.value_checks(diag.RosResult(available=False, reason='none'))
        assert checks and all(c.status == diag.SKIP for c in checks)


class TestRender:
    def test_groups_are_labelled_and_ordered(self, diag):
        checks = [
            diag.Check('network', 'eth0', diag.FAIL, 'two addresses'),
            diag.Check('devices', 'lidar', diag.OK, '/dev/ttyUSB0'),
        ]
        out = diag.render(checks, 1.0, 1)
        assert out.index('DEVICES') < out.index('NETWORK'), 'section order is fixed'

    def test_tally_and_exit_code_reported(self, diag):
        checks = [
            diag.Check('devices', 'a', diag.OK),
            diag.Check('devices', 'b', diag.WARN),
            diag.Check('devices', 'c', diag.FAIL),
            diag.Check('devices', 'd', diag.SKIP),
        ]
        out = diag.render(checks, 3.4, 1)
        assert '1 ok' in out and '1 warn' in out and '1 fail' in out and '1 skipped' in out
        assert 'exit 1' in out

    def test_strict_note_only_on_failure(self, diag):
        passing = diag.render([diag.Check('devices', 'a', diag.OK)], 1.0, 0)
        assert 'Strict' not in passing
        failing = diag.render([diag.Check('devices', 'a', diag.WARN)], 1.0, 1)
        assert 'Strict' in failing


class TestCommandLine:
    def test_unknown_section_errors(self):
        result = _run('--section', 'nonsense')
        assert result.returncode == 2
        assert 'unknown section' in result.stderr

    def test_quick_skips_the_ros_phase(self):
        # Host-only, so it must not wait on discovery.
        result = _run('--quick', timeout=30)
        assert result.returncode in (0, 1)
        assert 'SENSORS' not in result.stdout
        assert 'DEVICES' in result.stdout

    def test_quick_does_not_report_skipped_sensors(self):
        # Deselecting is not the same as failing to run: a section that was
        # never requested must not drag the exit code down.
        result = _run('--quick', '--section', 'devices', timeout=30)
        assert '0 skipped' in result.stdout

    def test_section_narrows_output(self):
        result = _run('--quick', '--section', 'system', timeout=30)
        assert 'SYSTEM' in result.stdout
        assert 'DEVICES' not in result.stdout

    def test_json_is_valid_and_carries_the_exit_code(self):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        assert 'checks' in payload
        assert payload['exit_code'] == result.returncode
        for check in payload['checks']:
            assert set(check) == {'group', 'name', 'status', 'detail'}
            assert check['status'] in ('OK', 'WARN', 'FAIL', 'SKIP')

    def test_json_groups_are_known_sections(self, diag):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        groups = {c['group'] for c in payload['checks']}
        assert groups <= set(diag.SECTIONS)

    def test_help_documents_strictness(self):
        result = _run('--help')
        assert result.returncode == 0
        assert 'Exits 0 only when every requested check passes' in result.stdout


class TestStrictness:
    def test_all_ok_passes(self, diag):
        assert diag.exit_code_for([diag.Check('devices', 'a', diag.OK)]) == 0

    def test_nothing_checked_passes(self, diag):
        assert diag.exit_code_for([]) == 0

    @pytest.mark.parametrize('status', ['WARN', 'FAIL', 'SKIP'])
    def test_non_ok_status_is_not_a_pass(self, diag, status):
        # A skipped sensor check on a car with teleop stopped is not a healthy car.
        checks = [
            diag.Check('devices', 'a', diag.OK),
            diag.Check('sensors', 'b', getattr(diag, status)),
        ]
        assert diag.exit_code_for(checks) == 1


def _eth(*addrs):
    return ''.join(f'2: eth0    inet {a} brd 192.168.52.255 scope global eth0\n' for a in addrs)


V6_DEFAULT = 'default via fe80::1 dev eth0 proto ra metric 100\n'


@pytest.fixture
def network(diag, monkeypatch, tmp_path):
    """Run check_network against canned command output; returns {check name: Check}."""
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('RACECAR_ETH_MODE', raising=False)

    def run(eth='', wlan0='', v6='', nmcli='', target=''):
        table = {
            'ip -4 -o addr show eth0': eth,
            'ip -4 -o addr show wlan0': wlan0,
            'ip -6 route show default dev eth0': v6,
            'nmcli': nmcli,
            'systemctl get-default': target,
        }

        def fake(cmd, timeout=5.0):
            key = ' '.join(cmd)
            return next((out for prefix, out in table.items() if key.startswith(prefix)), '')

        monkeypatch.setattr(diag, '_run', fake)
        return {c.name: c for c in diag.check_network()}

    return run


class TestCheckNetwork:
    def test_one_address_is_ok(self, network, diag):
        check = network(eth=_eth('192.168.52.200/24'))['eth0 address']
        assert (check.status, check.detail) == (diag.OK, '192.168.52.200/24')

    def test_two_addresses_fail_and_name_the_fix(self, network, diag):
        check = network(eth=_eth('192.168.52.200/24', '10.0.0.7/24'))['eth0 address']
        assert check.status == diag.FAIL
        assert '2 IPv4 addresses (192.168.52.200/24, 10.0.0.7/24)' in check.detail
        assert 'racecar eth static' in check.detail

    def test_no_address_warns(self, network, diag):
        assert network()['eth0 address'].status == diag.WARN

    def test_v6_default_fails_in_static_mode(self, network, diag):
        # No persisted config under HOME, so the mode reads as static.
        assert network(v6=V6_DEFAULT)['eth0 v6 default'].status == diag.FAIL

    def test_no_v6_default_in_static_mode_is_ok(self, network, diag):
        assert network()['eth0 v6 default'].status == diag.OK

    def test_v6_default_allowed_in_dynamic_mode(self, network, diag, tmp_path):
        cfg = tmp_path / '.config' / 'racecar'
        cfg.mkdir(parents=True)
        (cfg / 'networking.env').write_text('RACECAR_ETH_MODE="dynamic"\n')
        check = network(v6=V6_DEFAULT)['eth0 v6 default']
        assert check.status == diag.OK
        assert 'dynamic' in check.detail

    def test_environment_mode_wins(self, network, diag, monkeypatch):
        monkeypatch.setenv('RACECAR_ETH_MODE', 'dynamic')
        assert network(v6=V6_DEFAULT)['eth0 v6 default'].status == diag.OK

    def test_wlan0_is_informational(self, network, diag):
        assert network()['wlan0 client'].status == diag.OK
        assert network(wlan0=_eth('10.1.2.3/16'))['wlan0 client'].detail == '10.1.2.3/16'

    def test_ap_states(self, network, diag):
        assert network()['wlan1 AP'].status == diag.SKIP
        assert network(nmcli='racecar-neo-ap:wlan1\nHomeNet:wlan0\n')['wlan1 AP'].status == diag.OK
        assert network(nmcli='HomeNet:wlan0\n')['wlan1 AP'].status == diag.WARN

    def test_desktop_target(self, network, diag):
        assert network(target='graphical.target\n')['desktop'].detail == 'enabled'
        headless = network(target='multi-user.target\n')['desktop'].detail
        assert headless == 'headless (multi-user.target)'
        assert network()['desktop'].status == diag.SKIP


class TestActuatorChecks:
    def test_display_rows_follow_the_running_nodes(self, diag, monkeypatch):
        monkeypatch.setattr(
            diag, '_run', lambda cmd, timeout=5.0: '123\n' if cmd[-1] == 'pit_node' else ''
        )
        checks = {c.name: c for c in diag.actuator_checks(diag.RosResult())}
        assert checks['Dot matrix'].status == diag.WARN
        assert checks['LED strip'].status == diag.OK
