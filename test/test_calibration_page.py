"""Calibration page support: expected ranges, progress events, calibrate --verify."""

import json
from pathlib import Path
import subprocess

from conftest import load_script
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def ex():
    return load_script('expected')


@pytest.fixture
def pg():
    return load_script('progress')


class TestExpected:
    def test_every_range_is_ordered(self, ex):
        for key, rule in ex.ranges().items():
            fail, warn = rule['fail'], rule.get('warn')
            assert fail[0] < fail[1], key
            if warn:
                assert fail[0] <= warn[0] < warn[1] <= fail[1], key

    def test_rate(self, ex):
        assert ex.rate('laser_yaw_deg', 179.6)['verdict'] == ex.OK
        assert ex.rate('laser_yaw_deg', 178.0)['verdict'] == ex.WARN
        assert ex.rate('laser_yaw_deg', 170.0)['verdict'] == ex.FAIL
        assert ex.rate('accelerometer.scale', [1.0, 1.0, 1.2])['verdict'] == ex.FAIL
        assert ex.rate('no_such_key', 1.0) is None

    def test_require_refuses_a_fail(self, ex):
        assert ex.require({'laser_yaw_deg': 180.1})[0]['verdict'] == ex.OK
        with pytest.raises(ex.NotSaved, match='laser_yaw_deg 90'):
            ex.require({'laser_yaw_deg': 90})

    def test_bill_values_pass(self, ex):
        # The reference car's calibrations, 2026-11-20.
        bill = {
            'accelerometer.bias': [-0.294, -0.032, 0.144],
            'accelerometer.scale': [0.9997, 1.0058, 1.0040],
            'gyroscope.bias': [0.0345, -0.0012, -0.0949],
            'realsense_accel_bias': [0.288, 0.602, 0.164],
            'realsense_accel_scale': [1.0127, 1.0215, 1.0176],
            'laser_yaw_deg': 179.619,
            'camera_pitch_deg': 2.07,
            'camera_roll_deg': -1.27,
            'camera_bottom_z_mm': 101.5,
            'odometry.speed_scale': 0.92873,
            'odometry.yaw_rate_scale': 1.00212,
            'steering_trim': 0.0225,
        }
        assert all(r['verdict'] != ex.FAIL for r in ex.verdicts(bill))


class TestProgress:
    def test_off_without_the_file(self, pg, monkeypatch, tmp_path):
        monkeypatch.delenv(pg.PROGRESS_FILE, raising=False)
        pg.emit('stage', index=0)
        assert not pg.enabled()

    def test_lines(self, pg, monkeypatch, tmp_path):
        path = tmp_path / 'p.jsonl'
        monkeypatch.setenv(pg.PROGRESS_FILE, str(path))
        pg.emit('stage', index=1, total=6)
        pg.plot('t', [{'name': 'a', 'points': [[i, i] for i in range(1000)]}], x='x', y='y')
        events = [json.loads(line) for line in path.read_text().splitlines()]
        assert events[0]['kind'] == 'stage' and events[0]['total'] == 6
        points = events[1]['series'][0]['points']
        assert len(points) == pg.MAX_POINTS and points[-1] == [999, 999]

    def test_spread(self, pg):
        assert pg.spread([[1.0, 0.0], [3.0, 0.0]]) == pytest.approx([1.41421356, 0.0])
        assert pg.spread([[1.0, 2.0]]) == [0.0, 0.0]


def test_still_reading_skips_images():
    cc = load_script('calibrate_common')
    assert len(cc.still_reading([[0.0] * 6, [1.0] * 6])) == 6
    assert cc.still_reading([[[0.0] * 10] * 10]) == []
    assert cc.still_reading([]) == []


class TestVerify:
    @pytest.fixture
    def cal(self, car_profile_dir, monkeypatch):
        cal = load_script('calibrate')
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'car.yaml').write_text('name: bill\n')
        monkeypatch.setattr(cal, 'restart_teleop', lambda: True)
        return cal

    def fake_tool(self, cal, monkeypatch, write, verdict):
        def run(argv, **kw):
            if '--check' in argv:
                report = Path(argv[argv.index('--report') + 1])
                report.write_text(json.dumps({'overall': verdict, 'criteria': []}))
            elif write is not None:
                cal.car_profile.param_path('frames').write_text(write)
            return subprocess.CompletedProcess(argv, 0)

        monkeypatch.setattr(cal.subprocess, 'run', run)

    def step(self, cal):
        return next(s for s in cal.STEPS if s.key == 'lidar')

    def state(self, car_profile_dir):
        return yaml.safe_load((car_profile_dir / 'car.yaml').read_text())['calibration']['lidar']

    def test_pass_keeps_the_new_values(self, cal, car_profile_dir, monkeypatch):
        self.fake_tool(cal, monkeypatch, 'laser_yaw_deg: 180.1\n', 'PASS')
        assert cal.verify_step(self.step(cal)) == 0
        assert '180.1' in cal.car_profile.param_path('frames').read_text()
        assert self.state(car_profile_dir)['result'] == 'PASS'

    def test_failed_check_restores_the_old_file(self, cal, car_profile_dir, monkeypatch):
        cal.car_profile.param_path('frames').write_text('laser_yaw_deg: 179.6\n')
        self.fake_tool(cal, monkeypatch, 'laser_yaw_deg: 181.0\n', 'FAIL')
        assert cal.verify_step(self.step(cal)) == 1
        assert '179.6' in cal.car_profile.param_path('frames').read_text()
        assert self.state(car_profile_dir)['result'] == 'FAIL'

    def test_nothing_written_fails(self, cal, car_profile_dir, monkeypatch):
        self.fake_tool(cal, monkeypatch, None, 'PASS')
        assert cal.verify_step(self.step(cal)) == 1
        assert self.state(car_profile_dir)['detail'] == 'nothing was written'

    def test_steering_restores_its_table_too(self, cal):
        assert cal.EXTRA_FILES['steering'] == ('steering_table',)


class TestPage:
    @pytest.fixture
    def pa(self):
        return load_script('profile_app')

    def test_tile_state(self, pa):
        assert pa.tile_state('default', 'x', None) == 'open'
        assert pa.tile_state('calibrated', '2026-11-20 10:00', None) == 'unchecked'
        old = {'result': 'PASS', 'time': '2026-11-19 10:00'}
        assert pa.tile_state('calibrated', '2026-11-20 10:00', old) == 'unchecked'
        new = {'result': 'PASS', 'time': '2026-11-20 10:05'}
        assert pa.tile_state('calibrated', '2026-11-20 10:00', new) == 'valid'
        assert (
            pa.tile_state('calibrated', '2026-11-20 10:00', {**new, 'result': 'FAIL'}) == 'failed'
        )

    def test_argv(self, pa):
        assert pa.calibration_argv('imu', 'calibrate')[-2:] == ['imu', '--verify']
        assert pa.calibration_argv('lidar', 'check')[-2:] == ['check', 'lidar']
        with pytest.raises(ValueError):
            pa.calibration_argv('dynamics', 'check')
        with pytest.raises(ValueError):
            pa.calibration_argv('nope', 'calibrate')

    def test_read_progress(self, pa, tmp_path):
        path = tmp_path / 'p.jsonl'
        path.write_text('{"kind": "stage"}\nnot json\n{"kind": "plot"}\n')
        got = pa.read_progress(0, path)
        assert [e['kind'] for e in got['events']] == ['stage', 'plot'] and got['next'] == 3
        assert pa.read_progress(2, path)['events'] == [{'kind': 'plot'}]
        assert pa.read_progress(0, tmp_path / 'none')['next'] == 0

    def test_every_tool_emits_a_plot(self):
        tools = ROOT / 'scripts' / 'calibration'
        for name in ('calibrate_mag', 'calibrate_lidar', 'calibrate_camera', 'calibrate_odom',
                     'calibrate_steering', 'calibrate_dynamics', 'calibrate_common'):  # fmt: skip
            assert 'progress.plot(' in (tools / f'{name}.py').read_text(), name
