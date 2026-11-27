"""racecar_neo_ros2_driver/readiness.py and scripts/racecar_check.py: the parked checks."""

import json
import math

from conftest import load_script
import pytest
from racecar_neo_ros2_driver import readiness as rd

G = rd.GRAVITY


def still(n=200, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, G)):
    return [[*accel, *gyro] for _ in range(n)]


class TestRows:
    def test_imu_still_passes(self):
        rows = rd.imu_rows(still())
        assert [r.status for r in rows] == [rd.OK, rd.OK] and rows[0].repair == ''

    def test_gyro_bias_case(self):
        # The parked gyro bias of 2026-11-02: -1.59 deg/s on z.
        rows = rd.imu_rows(still(gyro=(0.0, 0.0, math.radians(-1.59))))
        assert rows[0].status == rd.WARN and rows[0].repair == 'Calibrate imu'
        assert rd.imu_rows(still(gyro=(0.0, 0.0, 0.05)))[0].status == rd.FAIL
        assert rows[0].link == 'calibration'

    def test_gravity(self):
        assert rd.imu_rows(still(accel=(0, 0, G * 1.02)))[1].status == rd.WARN
        assert rd.imu_rows(still(accel=(0, 0, G * 1.05)))[1].status == rd.FAIL
        assert rd.imu_rows([])[0].status == rd.FAIL

    def test_odometry(self):
        assert rd.odom_rows([[0, 0, 0], [0.005, 0, 0.001]])[0].status == rd.OK
        assert rd.odom_rows([[0, 0, 0], [0.03, 0, 0]])[0].status == rd.WARN
        assert rd.odom_rows([[0, 0, 0], [0, 0, math.radians(5)]])[0].status == rd.FAIL
        assert rd.odom_rows([])[0].status == rd.FAIL

    def test_lidar(self):
        assert rd.lidar_rows([(300, 360)] * 21, 3.0)[0].status == rd.OK
        assert rd.lidar_rows([(30, 360)] * 21, 3.0)[0].status == rd.FAIL
        assert rd.lidar_rows([(300, 360)] * 15, 3.0)[0].status == rd.WARN
        assert rd.lidar_rows([], 3.0)[0].status == rd.FAIL

    def test_rate_teleop_battery(self):
        assert rd.rate_row('PIT', 199.0, 200.0).status == rd.OK
        assert rd.rate_row('PIT', 120.0, 200.0).status == rd.WARN
        assert rd.rate_row('PIT', None, 200.0).status == rd.FAIL
        assert rd.teleop_row(False).status == rd.FAIL
        ok = {'volts': 8.0, 'stale': False, 'estimate': {'level': 'ok', 'soc': 80}}
        assert rd.battery_row(ok).detail == '8.00 V, 80%'
        assert rd.battery_row({**ok, 'estimate': {'level': 'warn'}}).status == rd.WARN
        assert rd.battery_row({**ok, 'estimate': {'level': 'countdown'}}).status == rd.FAIL
        assert rd.battery_row(None).status == rd.WARN

    def test_calibration(self):
        steps = [('imu', 'calibrated', False), ('dynamics', 'default', True)]
        assert rd.calibration_rows(steps, {})[0].status == rd.OK
        assert rd.calibration_rows([('imu', 'default', False)], {})[0].status == rd.FAIL
        failed = rd.calibration_rows(steps, {'imu': {'result': 'FAIL'}})[0]
        assert failed.status == rd.WARN and 'imu' in failed.detail

    def test_health(self):
        power = {'group': 'power', 'name': 'ext5v', 'status': 'OK', 'detail': ''}
        link = {'group': 'link', 'name': 'gaps', 'status': 'FAIL', 'detail': '80 ms'}
        report = {'checks': [power, link]}
        rows = rd.health_rows(report, ['power', 'link', 'nvme'])
        assert [r.status for r in rows] == [rd.OK, rd.FAIL, rd.FAIL]
        assert rows[1].detail == 'gaps 80 ms' and rows[2].detail == 'not run'

    def test_worst(self):
        assert rd.worst([]) == rd.OK
        assert rd.worst([rd.row('a', rd.WARN, ''), rd.row('b', rd.OK, '')]) == rd.WARN


@pytest.fixture
def rc():
    return load_script('racecar_check')


def test_group_of(rc):
    assert rc.group_of('gyro at rest') == 'imu'
    assert rc.group_of('PIT sensor rate') == 'rates'
    assert rc.group_of('health nvme') == 'nvme'
    assert rc.group_of('lidar') == 'lidar'


def test_save_keeps_other_checks(rc, tmp_path, monkeypatch):
    from racecar_neo_ros2_driver import data_root

    monkeypatch.setattr(data_root, 'data_dir', lambda sub=None: tmp_path)
    monkeypatch.setattr(rc, 'teleop_started', lambda: 'now')
    monkeypatch.setattr(rc, 'calibration_mtime', lambda: 1.0)
    rc.save(
        [rd.row('lidar', rd.OK, 'fine'), rd.row('teleop', rd.FAIL, 'off')], ['lidar', 'teleop']
    )
    rc.save([rd.row('lidar', rd.WARN, 'slow')], ['lidar'])
    data = json.loads((tmp_path / rc.LAST_FILE).read_text())
    assert {(r['group'], r['status']) for r in data['rows']} == {
        ('teleop', 'FAIL'),
        ('lidar', 'WARN'),
    }
    assert data['result'] == rd.FAIL


def test_rates_from_diagnostics(rc):
    sample = rc.Sample()
    sample.seconds = 3.0
    sample.diag = {rc.LINK_STATUS_NAME: 200.0, 'camera: color': 59.0}
    rows = {r.name: r for r in rc.sampled_rows(['rates'], sample)}
    assert rows['PIT sensor rate'].status == rd.OK
    assert rows['camera depth rate'].status == rd.WARN


def test_main_exit_codes(rc, monkeypatch, capsys):
    monkeypatch.setattr(rc, 'save', lambda rows, names: None)
    monkeypatch.setattr(rc, 'run', lambda names, seconds: [rd.row('teleop', rd.OK, 'running')])
    assert rc.main(['teleop']) == 0
    monkeypatch.setattr(
        rc, 'run', lambda names, seconds: [rd.row('teleop', rd.FAIL, 'off', 'Start it')]
    )
    assert rc.main([]) == 1
    out = capsys.readouterr().out
    assert '[FAIL]  teleop  off' in out and 'Start it' in out
    with pytest.raises(SystemExit):
        rc.main(['nope'])


class TestPage:
    @pytest.fixture
    def pa(self):
        return load_script('profile_app')

    def test_argv(self, pa):
        assert pa.checks_argv([], True, '')[-1] == '--all'
        assert pa.checks_argv(['imu', 'lidar'], False, '')[-2:] == ['imu', 'lidar']
        assert pa.checks_argv([], False, 'shake')[-1] == 'shake'
        with pytest.raises(ValueError):
            pa.checks_argv(['nope'], False, '')
        with pytest.raises(ValueError):
            pa.checks_argv([], False, 'reboot')

    def test_view(self, pa):
        view = pa.checks_view()
        assert view['guided'] == ['shake', 'poke'] and 'imu' in view['parked']
        assert view['health'] == ['power', 'link', 'nvme']


def test_racecar_check_dispatches():
    from pathlib import Path

    tool = (Path(__file__).resolve().parent.parent / 'scripts' / 'racecar-tool.sh').read_text()
    assert 'python3 "$pkg_dir/scripts/racecar_check.py" "$@"' in tool


class TestGate:
    @pytest.fixture
    def last(self, rc, monkeypatch):
        monkeypatch.setattr(rc, 'boot_id', lambda: 'boot')
        monkeypatch.setattr(rc, 'teleop_started', lambda: 'start')
        monkeypatch.setattr(rc, 'calibration_mtime', lambda: 10.0)
        rows = [{'group': g, 'status': 'OK'} for g in rc.PARKED]
        return {'boot_id': 'boot', 'teleop_started': 'start', 'calibration_mtime': 10.0,
                'rows': rows, 'time': 't'}  # fmt: skip

    def test_pass_and_warn(self, rc, last):
        assert rc.gate(last)['passed']
        last['rows'][0]['status'] = 'WARN'
        assert rc.gate(last)['passed']

    def test_reasons(self, rc, last, monkeypatch):
        assert rc.gate({})['reason'] == 'The checks have not run.'
        assert 'Not every' in rc.gate({**last, 'rows': last['rows'][1:]})['reason']
        failed = [{**r, 'status': 'FAIL'} if i == 0 else r for i, r in enumerate(last['rows'])]
        assert rc.gate({**last, 'rows': failed})['reason'] == 'A check failed.'
        assert 'restarted' in rc.gate({**last, 'boot_id': 'old'})['reason']
        assert 'Teleop' in rc.gate({**last, 'teleop_started': 'old'})['reason']
        monkeypatch.setattr(rc, 'calibration_mtime', lambda: 11.0)
        assert 'calibration' in rc.gate(last)['reason']

    def test_health_rows_do_not_block(self, rc, last):
        last['rows'].append({'group': 'power', 'status': 'FAIL'})
        assert rc.gate(last)['passed']
