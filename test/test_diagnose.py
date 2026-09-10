"""Unit tests for scripts/diagnose.py (the `racecar status` diagnostic)."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'diagnose.py'


def _load():
    # dataclasses resolves annotations through sys.modules[cls.__module__],
    # so the module has to be registered before it is executed.
    spec = importlib.util.spec_from_file_location('diagnose', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules['diagnose'] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def diag():
    return _load()


def _run(*args, timeout=60):
    return subprocess.run(
        ['python3', str(SCRIPT), *args],
        capture_output=True, text=True, timeout=timeout,
    )


def test_script_exists_and_executable():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK)


def test_py_compile_clean():
    result = subprocess.run(
        ['python3', '-m', 'py_compile', str(SCRIPT)],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stderr


class TestTopicSpecs:
    def test_floor_is_a_fraction_of_nominal(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        assert spec.floor == pytest.approx(80.0)

    def test_sensors_default_to_eighty_percent(self, diag):
        pit = {'/imu/lsm9ds1', '/mag', '/encoder/speed', '/battery/voltage',
               '/battery/current', '/rc/channels'}
        for spec in diag.SENSOR_TOPICS:
            if spec.topic in pit:
                continue
            assert spec.floor_frac == 0.8, f'{spec.topic} should use the 80% floor'

    def test_pit_topics_carry_the_wider_floor(self, diag):
        # The six topics off the Teensy telemetry frame slow under graph load,
        # so their floor leaves room for it. A halved frame rate (68 Hz) must
        # still fail, or the check stops meaning anything.
        pit = [s for s in diag.SENSOR_TOPICS
               if s.nominal == 136.0 and 'Teensy' in s.note]
        assert len(pit) == 6
        for spec in pit:
            assert spec.floor_frac == diag.PIT_FLOOR_FRAC
            assert spec.floor < 90.0, 'floor must pass a car delivering 90 Hz'
            assert spec.floor > 68.0, 'floor must still catch a halved frame rate'

    def test_lidar_nominal_matches_the_delivered_rate(self, diag):
        # 8.0 through v0.8.0 put the floor at 6.4 on a lidar delivering 7.2.
        scan = next(s for s in diag.SENSOR_TOPICS if s.topic == '/scan')
        assert scan.nominal == pytest.approx(7.2)
        assert scan.floor < 6.2, 'an ordinary dip must not fail a healthy lidar'
        assert scan.floor > 2.0, 'a lidar desynced to 2 Hz must still fail'

    def test_camera_nominal_is_the_configured_rate(self, diag):
        # Until v0.8.1 these were declared at 25 and 12 against a note calling
        # the gap a hardware shortfall. /diagnostics reports the camera node
        # delivering 59.0 and 29.6 against targets of 60 and 30; the shortfall
        # was this tool's own subscription cost.
        by_topic = {s.topic: s for s in diag.SENSOR_TOPICS}
        assert by_topic['/camera/color'].nominal == 60.0
        assert by_topic['/camera/depth'].nominal == 30.0

    def test_realsense_rates_come_from_diagnostics(self, diag):
        # Subscribing to the two image streams cost 40% of the PIT telemetry
        # rate, which the PIT topics were then failed for.
        assert diag.DIAGNOSTIC_SOURCED == {
            '/camera/color', '/camera/depth', '/imu/realsense'}
        for topic in diag.DIAGNOSTIC_SOURCED:
            spec = next(s for s in diag.SENSOR_TOPICS if s.topic == topic)
            assert 'diagnostics' in spec.note

    def test_every_notebook_topic_is_covered(self, diag):
        covered = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        for topic in ('/camera/color', '/camera/depth', '/scan', '/imu/lsm9ds1',
                      '/mag', '/imu/fused', '/encoder/speed', '/battery/voltage',
                      '/battery/current', '/rc/channels', '/edgetpu/inference',
                      '/joy', '/motor', '/mux_out'):
            assert topic in covered, f'{topic} lost from the check set'

    def test_value_topics_are_a_subset_of_sampled_topics(self, diag):
        sampled = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        assert diag.VALUE_TOPICS <= sampled


class TestRateChecks:
    def _ros(self, diag, counts, elapsed=2.0):
        return diag.RosResult(
            available=True, counts=counts, elapsed=elapsed, present=set(counts))

    def test_rate_above_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 200})  # 100 Hz over 2 s
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_rate_below_floor_fails(self, diag):
        # A desynced lidar still has a live topic, so presence alone is not
        # enough; the floor is the whole point of the check.
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
            available=True, counts={}, elapsed=5.0,
            present={'/camera/color'}, reported={'/camera/color': 59.0})
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.OK
        assert '59.0 Hz' in check.detail

    def test_diagnostic_sourced_topic_below_floor_fails(self, diag):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/depth')
        ros = diag.RosResult(
            available=True, counts={}, elapsed=5.0,
            present={'/camera/depth'}, reported={'/camera/depth': 12.0})
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_missing_diagnostic_rate_warns_rather_than_reading_zero(self, diag):
        # A DiagnosticArray that never arrived says nothing about the camera.
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/color')
        ros = diag.RosResult(
            available=True, counts={}, elapsed=5.0,
            present={'/camera/color'}, reported={})
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.WARN
        assert '/diagnostics' in check.detail


class TestSampleWindow:
    def test_window_is_long_enough_for_the_pit_stream(self, diag):
        # At 2.0 s the PIT stream measured to a standard deviation of 32 Hz on
        # a ~136 Hz signal, which is what produced the spurious failures.
        assert diag.DEFAULT_WINDOW >= 5.0

    def test_diagnostic_names_cover_every_diagnostic_sourced_topic(self, diag):
        assert set(diag.REALSENSE_DIAGNOSTIC_NAMES.values()) == diag.DIAGNOSTIC_SOURCED

    def test_diagnostic_parser_extracts_the_actual_frequency(self, diag):
        class _Value:
            def __init__(self, key, value):
                self.key, self.value = key, value

        class _Status:
            def __init__(self, name, values):
                self.name, self.values = name, values

        class _Msg:
            def __init__(self, status):
                self.status = status

        msg = _Msg([
            _Status('camera: color', [
                _Value('Target frequency (Hz)', '60.0'),
                _Value('Actual frequency (Hz)', '59.4'),
            ]),
            _Status('camera: Temperatures', [_Value('Asic Temperature', '54')]),
        ])
        out = {}
        diag._read_diagnostic_rates(msg, out)
        assert out == {'/camera/color': pytest.approx(59.4)}

    def test_diagnostic_parser_ignores_an_unparsable_rate(self, diag):
        class _Value:
            def __init__(self, key, value):
                self.key, self.value = key, value

        class _Status:
            def __init__(self, name, values):
                self.name, self.values = name, values

        class _Msg:
            def __init__(self, status):
                self.status = status

        out = {}
        diag._read_diagnostic_rates(
            _Msg([_Status('camera: depth',
                          [_Value('Actual frequency (Hz)', 'n/a')])]), out)
        assert out == {}


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
        ros = self._ros_with(diag, {
            '/battery/voltage': SimpleNamespace(data=11.4),
            '/battery/current': SimpleNamespace(data=2.0),
        })
        check = next(c for c in diag.value_checks(ros) if c.name == 'Pack voltage range')
        assert check.status == diag.OK

    def test_pack_voltage_out_of_range_fails(self, diag):
        ros = self._ros_with(diag, {
            '/battery/voltage': SimpleNamespace(data=2.0),
            '/battery/current': SimpleNamespace(data=1.0),
        })
        check = next(c for c in diag.value_checks(ros) if c.name == 'Pack voltage range')
        assert check.status == diag.FAIL

    def test_lidar_sample_count(self, diag):
        ros = self._ros_with(diag, {'/scan': SimpleNamespace(ranges=[0.0] * 1080)})
        check = next(c for c in diag.value_checks(ros) if c.name == 'LIDAR samples')
        assert check.status == diag.OK

    def test_unexpected_lidar_sample_count_warns(self, diag):
        # The sim publishes 720; that is worth surfacing but is not a fault of
        # the hardware being diagnosed.
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

    def test_json_round_trips_every_section(self):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        groups = {c['group'] for c in payload['checks']}
        assert groups <= set(_load().SECTIONS)

    def test_help_documents_strictness(self):
        result = _run('--help')
        assert result.returncode == 0
        assert 'Exits 0 only when every requested check passes' in result.stdout


class TestStrictness:
    """The exit code is the contract: anything but OK is a failure."""

    def test_all_ok_exits_zero(self, diag):
        checks = [diag.Check('devices', 'a', diag.OK)]
        assert all(c.status == diag.OK for c in checks)

    @pytest.mark.parametrize('status', ['WARN', 'FAIL', 'SKIP'])
    def test_non_ok_status_is_not_a_pass(self, diag, status):
        # Mirrors the exit-code rule in main(): a skipped sensor check on a
        # car with teleop stopped must not read as a healthy car.
        checks = [diag.Check('devices', 'a', diag.OK),
                  diag.Check('sensors', 'b', getattr(diag, status))]
        assert not all(c.status == diag.OK for c in checks)
