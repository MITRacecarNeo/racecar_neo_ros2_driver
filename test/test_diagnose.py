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
        for spec in diag.SENSOR_TOPICS:
            if spec.topic in diag.PIT_TOPICS:
                continue
            assert spec.floor_frac == 0.8, f'{spec.topic} should use the 80% floor'

    def test_pit_topics_carry_the_wider_floor(self, diag):
        # The six topics off the Teensy telemetry frame slow under graph load,
        # so their floor leaves room for it. A halved frame rate (68 Hz) must
        # still warn, or the check stops meaning anything.
        pit = [s for s in diag.SENSOR_TOPICS if s.topic in diag.PIT_TOPICS]
        assert len(pit) == 6
        for spec in pit:
            assert spec.nominal == 136.0
            assert spec.floor_frac == diag.PIT_FLOOR_FRAC
            assert spec.floor < 90.0, 'floor must pass a car delivering 90 Hz'
            assert spec.floor > 68.0, 'floor must still flag a halved frame rate'

    def test_lidar_nominal_matches_the_delivered_rate(self, diag):
        scan = next(s for s in diag.SENSOR_TOPICS if s.topic == '/scan')
        assert scan.nominal == pytest.approx(7.2)
        assert scan.floor < 6.2, 'an ordinary dip must not fail a healthy lidar'
        assert scan.floor > 2.0, 'a lidar desynced to 2 Hz must still warn'

    def test_camera_nominal_is_the_configured_rate(self, diag):
        # The camera node reports 59.0 and 29.6 against these on /diagnostics.
        by_topic = {s.topic: s for s in diag.SENSOR_TOPICS}
        assert by_topic['/camera/color'].nominal == 60.0
        assert by_topic['/camera/depth'].nominal == 30.0

    def test_realsense_rates_come_from_diagnostics(self, diag):
        # Subscribing to the image streams would starve the PIT telemetry rate.
        assert diag.DIAGNOSTIC_SOURCED == {'/camera/color', '/camera/depth', '/imu/realsense'}

    def test_every_nominal_sits_well_above_the_stall_line(self, diag):
        # A nominal near STALL_HZ would turn an ordinary dip into a FAIL.
        for spec in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS:
            assert spec.floor > 2 * diag.STALL_HZ, spec.topic

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

    def test_every_sampled_topic_has_a_reader(self, diag):
        sampled = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        assert set(diag.SAMPLE_READERS) == sampled


class TestRateChecks:
    def _ros(self, diag, counts, elapsed=2.0):
        return diag.RosResult(available=True, counts=counts, elapsed=elapsed, present=set(counts))

    def test_rate_above_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 200})  # 100 Hz over 2 s
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_rate_below_floor_warns(self, diag):
        # Degraded but delivering: the car is still usable.
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 20})  # 10 Hz
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.WARN
        assert check.detail == '10.0/100 Hz'

    def test_slow_coral_warns(self, diag):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/edgetpu/inference')
        ros = self._ros(diag, {spec.topic: 43}, elapsed=5.0)  # 8.6 Hz
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.WARN

    @pytest.mark.parametrize('count', [0, 1, 9])
    def test_stalled_rate_fails(self, diag, count):
        # 0, 0.2 and 1.8 Hz over 5 s: stopped, not degraded.
        spec = diag.TopicSpec('/t', 'T', 136.0, 0.65)
        ros = self._ros(diag, {'/t': count}, elapsed=5.0)
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_stall_line_is_inclusive_of_two_hz(self, diag):
        spec = diag.TopicSpec('/t', 'T', 7.2)
        assert diag.rate_status(2.0, spec) == diag.WARN
        assert diag.rate_status(1.99, spec) == diag.FAIL

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

    def test_detail_carries_no_annotation(self, diag):
        for spec in diag.SENSOR_TOPICS:
            ros = self._ros(diag, {spec.topic: 1000}, elapsed=5.0)
            ros.reported = {spec.topic: 200.0}
            detail = diag.rate_checks('sensors', [spec], ros)[0].detail
            assert ';' not in detail and detail.endswith(' Hz'), spec.topic

    def test_sample_is_carried_in_the_data_column(self, diag):
        spec = diag.TopicSpec('/encoder/speed', 'Encoder', 100.0)
        ros = self._ros(diag, {'/encoder/speed': 200})
        ros.values = {'/encoder/speed': SimpleNamespace(data=1.5)}
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert (check.status, check.data) == (diag.OK, '+1.50 m/s')

    def test_bad_sample_fails_a_healthy_rate(self, diag):
        spec = diag.TopicSpec('/battery/voltage', 'Pack voltage', 100.0)
        ros = self._ros(diag, {'/battery/voltage': 200})
        ros.values = {'/battery/voltage': SimpleNamespace(data=2.0)}
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_missing_sample_does_not_change_the_status(self, diag):
        spec = diag.TopicSpec('/encoder/speed', 'Encoder', 100.0)
        check = diag.rate_checks('sensors', [spec], self._ros(diag, {'/encoder/speed': 200}))[0]
        assert (check.status, check.data) == (diag.OK, 'no sample captured')

    def test_stopped_stack_fails_every_row(self, diag):
        # rclpy works and the graph is visible, but none of the car's topics.
        ros = diag.RosResult(available=True, present=set())
        checks = diag.rate_checks('sensors', diag.SENSOR_TOPICS, ros)
        assert all(c.status == diag.FAIL for c in checks)

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
        assert check.detail == '59.0/60 Hz'

    @pytest.mark.parametrize('hz, status', [(12.0, 'WARN'), (0.5, 'FAIL')])
    def test_diagnostic_sourced_topic_below_floor(self, diag, hz, status):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/depth')
        ros = diag.RosResult(
            available=True,
            counts={},
            elapsed=5.0,
            present={'/camera/depth'},
            reported={'/camera/depth': hz},
        )
        assert diag.rate_checks('sensors', [spec], ros)[0].status == status

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


def _vec(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


class TestSampleReaders:
    def test_gravity_magnitude_at_rest_passes(self, diag):
        msg = SimpleNamespace(linear_acceleration=_vec(0.0, 0.0, 9.81))
        status, data = diag.read_pit_imu(msg)
        assert status == diag.OK
        assert data == 'accel +0.00 +0.00 +9.81 m/s^2, |a| 9.81'

    def test_implausible_gravity_fails(self, diag):
        msg = SimpleNamespace(linear_acceleration=_vec(0.0, 0.0, 0.2))
        status, data = diag.read_pit_imu(msg)
        assert status == diag.FAIL
        assert 'expect 8 to 12' in data

    def test_pack_voltage(self, diag):
        assert diag.read_pack_voltage(SimpleNamespace(data=11.4)) == (diag.OK, '11.40 V')
        assert diag.read_pack_voltage(SimpleNamespace(data=2.0))[0] == diag.FAIL

    def test_negative_pack_current_fails(self, diag):
        assert diag.read_pack_current(SimpleNamespace(data=2.6)) == (diag.OK, '2.60 A')
        assert diag.read_pack_current(SimpleNamespace(data=-0.5))[0] == diag.FAIL

    def _scan(self, ranges):
        return SimpleNamespace(ranges=ranges, range_min=0.15, range_max=12.0)

    def test_lidar_sample_count(self, diag):
        ranges = [1.0] * 540 + [float('inf')] * 540
        status, data = diag.read_scan(self._scan(ranges))
        assert status == diag.OK
        assert data == '1080 ranges, 540 returns, median 1.00 m'

    def test_unexpected_lidar_sample_count_warns(self, diag):
        # The sim publishes 720: worth surfacing, not a hardware fault.
        status, data = diag.read_scan(self._scan([2.0] * 720))
        assert status == diag.WARN
        assert 'expect 1080' in data

    def test_rc_channel_count(self, diag):
        status, data = diag.read_rc(SimpleNamespace(data=[0.0, 1.0, -1.0, 0.5] + [0.0] * 4))
        assert status == diag.OK
        assert data == '8 channels: +0.00 +1.00 -1.00 +0.50 ...'
        assert diag.read_rc(SimpleNamespace(data=[0.0] * 6))[0] == diag.FAIL

    def test_magnetometer_reads_in_microtesla(self, diag):
        msg = SimpleNamespace(magnetic_field=_vec(2e-5, 0.0, -4.5e-5))
        assert diag.read_mag(msg) == (diag.OK, 'field +20.00 +0.00 -45.00 uT')

    def test_fused_yaw(self, diag):
        # Quarter turn about z.
        q = SimpleNamespace(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        msg = SimpleNamespace(orientation=q, angular_velocity=_vec(0.0, 0.0, 0.1))
        assert diag.read_fused(msg) == (diag.OK, 'yaw +90.0 deg, gyro z +0.10 rad/s')

    def test_detections_report_the_top_score(self, diag):
        def det(*hyps):
            return SimpleNamespace(
                results=[
                    SimpleNamespace(hypothesis=SimpleNamespace(class_id=c, score=p))
                    for c, p in hyps
                ]
            )

        msg = SimpleNamespace(detections=[det(('cup', 0.41)), det(('person', 0.87))])
        assert diag.read_detections(msg) == (diag.OK, '2 detections, top person 0.87')
        assert diag.read_detections(SimpleNamespace(detections=[])) == (diag.OK, '0 detections')

    def test_joy(self, diag):
        msg = SimpleNamespace(axes=[0.0] * 8, buttons=[0, 1] + [0] * 9)
        assert diag.read_joy(msg) == (diag.OK, '8 axes, 11 buttons, 1 pressed')

    def test_depth_center_distance(self, diag):
        w, h = 4, 2
        data = bytearray(w * h * 2)
        lo = (h // 2) * w * 2 + (w // 2) * 2
        hi = lo + 2
        data[lo:hi] = (1234).to_bytes(2, 'little')
        msg = SimpleNamespace(
            width=w, height=h, step=w * 2, encoding='16UC1', is_bigendian=0, data=data
        )
        assert diag.read_depth(msg) == (diag.OK, '4x2 16UC1, center 1.23 m')

    def test_depth_without_a_return(self, diag):
        msg = SimpleNamespace(
            width=2, height=2, step=4, encoding='16UC1', is_bigendian=0, data=bytes(8)
        )
        assert diag.read_depth(msg)[1].endswith('center no return')

    def test_drive_command(self, diag):
        msg = SimpleNamespace(drive=SimpleNamespace(speed=0.5, steering_angle=-0.1))
        assert diag.read_drive(msg) == (diag.OK, 'speed +0.50 m/s, steer -0.10 rad')

    def test_unexpected_payload_warns(self, diag):
        # A sim publishing another type under the same name must not crash the run.
        status, data = diag.read_sample('/imu/lsm9ds1', SimpleNamespace(data=1.0))
        assert status == diag.WARN
        assert 'unreadable sample' in data


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
        assert 'RESULT FAIL' in out
        assert 'exit 1' in out

    def test_strict_note_only_in_strict_mode_on_failure(self, diag):
        warn = [diag.Check('devices', 'a', diag.WARN)]
        assert 'Strict' not in diag.render(warn, 1.0, 0)
        assert 'Strict' in diag.render(warn, 1.0, 1, strict=True)
        ok = [diag.Check('devices', 'a', diag.OK)]
        assert 'Strict' not in diag.render(ok, 1.0, 0, strict=True)

    def test_sample_column_is_tab_separated_and_aligned(self, diag):
        checks = [
            diag.Check('sensors', 'RPLIDAR', diag.OK, '7.2/7.2 Hz', '1080 ranges'),
            diag.Check('sensors', 'RealSense IMU', diag.OK, '200.0/200 Hz', 'accel'),
            diag.Check('system', 'throttling', diag.OK, 'none'),
        ]
        rows = [ln for ln in diag.render(checks, 1.0, 0).splitlines() if 'Hz' in ln]
        assert all(ln.count('\t') == 1 for ln in rows)
        assert len({ln.index('\t') for ln in rows}) == 1
        assert [ln.split('\t')[1] for ln in rows] == ['1080 ranges', 'accel']
        # The widest rate still has a space before the tab.
        assert all(ln.split('\t')[0].endswith(' ') for ln in rows)

    def test_rows_without_a_sample_have_no_tab(self, diag):
        out = diag.render([diag.Check('system', 'disk', diag.OK, '32G free')], 1.0, 0)
        assert '\t' not in out

    def test_plain_output_has_no_escape_codes(self, diag):
        checks = [diag.Check('devices', s, s) for s in (diag.OK, diag.WARN, diag.FAIL)]
        assert '\033[' not in diag.render(checks, 1.0, 1)

    def test_color_marks_each_status(self, diag):
        checks = [diag.Check('devices', s, s) for s in (diag.OK, diag.WARN, diag.FAIL)]
        out = diag.render(checks, 1.0, 1, color=True)
        assert '\033[32m[ OK ]\033[0m' in out
        assert '\033[33m[WARN]\033[0m' in out
        assert '\033[31m[FAIL]\033[0m' in out
        assert 'RESULT \033[1;31mFAIL\033[0m' in out

    @pytest.mark.parametrize(
        'statuses, result',
        [
            (['OK'], 'OK'),
            (['OK', 'SKIP'], 'WARN'),
            (['OK', 'WARN'], 'WARN'),
            (['WARN', 'FAIL'], 'FAIL'),
        ],
    )
    def test_overall_result(self, diag, statuses, result):
        assert diag.overall([diag.Check('devices', s, s) for s in statuses]) == result

    def test_no_color_when_not_a_terminal(self, diag, monkeypatch):
        monkeypatch.setattr(diag.sys.stdout, 'isatty', lambda: False, raising=False)
        assert diag.use_color() is False

    def test_no_color_env_opts_out(self, diag, monkeypatch):
        monkeypatch.setattr(diag.sys.stdout, 'isatty', lambda: True, raising=False)
        monkeypatch.setenv('NO_COLOR', '1')
        assert diag.use_color() is False


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
        assert payload['result'] in ('OK', 'WARN', 'FAIL')
        assert payload['strict'] is False
        for check in payload['checks']:
            assert set(check) == {'group', 'name', 'status', 'detail', 'data'}
            assert check['status'] in ('OK', 'WARN', 'FAIL', 'SKIP')

    def test_json_groups_are_known_sections(self, diag):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        groups = {c['group'] for c in payload['checks']}
        assert groups <= set(diag.SECTIONS)

    def test_help_documents_the_exit_rule(self):
        result = _run('--help')
        assert result.returncode == 0
        assert 'Exits 1 when a check fails' in result.stdout
        assert '--strict' in result.stdout


class TestExitCode:
    def test_all_ok_passes(self, diag):
        assert diag.exit_code_for([diag.Check('devices', 'a', diag.OK)]) == 0

    def test_nothing_checked_passes(self, diag):
        assert diag.exit_code_for([]) == 0

    @pytest.mark.parametrize('status', ['WARN', 'SKIP'])
    def test_warn_and_skip_leave_the_car_usable(self, diag, status):
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', status)]
        assert diag.exit_code_for(checks) == 0

    def test_fail_fails(self, diag):
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', diag.FAIL)]
        assert diag.exit_code_for(checks) == 1

    @pytest.mark.parametrize('status', ['WARN', 'FAIL', 'SKIP'])
    def test_strict_fails_anything_but_ok(self, diag, status):
        # A skipped sensor check on a car with teleop stopped is not a healthy car.
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', status)]
        assert diag.exit_code_for(checks, strict=True) == 1

    def test_strict_passes_all_ok(self, diag):
        assert diag.exit_code_for([diag.Check('devices', 'a', diag.OK)], strict=True) == 0


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

    def test_two_addresses_warn(self, network, diag):
        # Dual mode can be deliberate.
        check = network(eth=_eth('192.168.52.200/24', '10.0.0.7/24'))['eth0 address']
        assert check.status == diag.WARN
        assert check.detail == '192.168.52.200/24, 10.0.0.7/24 (dual mode)'

    def test_no_address_warns(self, network, diag):
        assert network()['eth0 address'].status == diag.WARN

    def test_v6_default_warns_in_static_mode(self, network, diag):
        # No persisted config under HOME, so the mode reads as static.
        check = network(v6=V6_DEFAULT)['eth0 v6 default']
        assert (check.status, check.detail) == (diag.WARN, 'present in static mode')

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


class TestCpu:
    def _feed(self, diag, monkeypatch, samples, own=(0.0, 0.0)):
        stat = iter(samples)
        own_times = iter(own)
        monkeypatch.setattr(diag.sysinfo, 'read_cpu_times', lambda: next(stat))
        monkeypatch.setattr(diag, '_own_cpu_seconds', lambda: next(own_times))

    def test_busy_share_is_bounded_by_the_cores(self, diag, monkeypatch):
        # 4 cores over 1 s at 100 jiffies/s: 400 total, 300 busy.
        self._feed(diag, monkeypatch, [(1000, 4000), (1300, 4400)])
        assert diag.measure_cpu_busy(0) == pytest.approx(75.0)

    def test_own_time_is_excluded(self, diag, monkeypatch):
        # 0.5 s of this process's CPU is 50 of the 300 busy jiffies.
        tck = diag.os.sysconf('SC_CLK_TCK')
        self._feed(diag, monkeypatch, [(0, 0), (300, 400)], own=(0.0, 50 / tck))
        assert diag.measure_cpu_busy(0) == pytest.approx(62.5)

    def test_unreadable_stat(self, diag, monkeypatch):
        self._feed(diag, monkeypatch, [None, None])
        assert diag.measure_cpu_busy(0) is None

    @pytest.mark.parametrize('busy, status', [(77.0, 'OK'), (95.0, 'WARN'), (100.0, 'WARN')])
    def test_cpu_row(self, diag, monkeypatch, busy, status):
        monkeypatch.setattr(diag, 'measure_cpu_busy', lambda interval: busy)
        monkeypatch.setattr(diag.sysinfo, 'read_arm_clock', lambda: (1000, 2400))
        row = next(c for c in diag.check_system(0) if c.name == 'cpu')
        assert row.status == status
        assert row.detail == f'{busy:.0f}% busy, arm 1000 of 2400 MHz'


class TestUnderVoltageAlarm:
    def test_sticky_alarm_warns(self, diag, monkeypatch):
        # A live dip is reported, and failed, by the throttling row.
        monkeypatch.setattr(diag, 'measure_cpu_busy', lambda interval: 10.0)
        monkeypatch.setattr(diag.sysinfo, 'read_under_voltage_alarm', lambda: True)
        row = next(c for c in diag.check_system(0) if c.name == 'under-voltage')
        assert row.status == diag.WARN
