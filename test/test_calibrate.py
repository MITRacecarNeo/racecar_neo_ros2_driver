"""Unit tests for scripts/calibration/calibrate.py (`racecar calibrate`)."""

from pathlib import Path

from conftest import load_script
import pytest
import yaml


@pytest.fixture(scope='module')
def cal():
    return load_script('calibrate')


@pytest.fixture
def config(tmp_path):
    d = tmp_path / 'config'
    d.mkdir()
    return d


def _ros_yaml(path: Path, node: str, params: dict) -> None:
    path.write_text(yaml.safe_dump({node: {'ros__parameters': params}}))


class TestSteps:
    def test_tools_exist(self, cal):
        for step in cal.STEPS:
            if step.tool is not None:
                assert (cal.HERE / step.tool).is_file(), step.tool

    def test_pending_steps_say_when(self, cal):
        assert all(step.pending for step in cal.STEPS if step.tool is None)

    def test_order_puts_hand_held_steps_first(self, cal):
        assert cal.STEP_KEYS[:4] == ['imu', 'mag', 'realsense', 'lidar']


class TestReadValues:
    def test_ros_parameter_file(self, cal, config):
        f = config / 'lsm9ds1_cal.local.yaml'
        _ros_yaml(f, 'pit_node', {'gyroscope.bias': [0.1, 0.2, 0.3]})
        assert cal.read_values(f) == {'gyroscope.bias': [0.1, 0.2, 0.3]}

    def test_plain_file(self, cal, config):
        f = config / 'frames.local.yaml'
        f.write_text('# header\nlaser_yaw_deg: 179.2\n')
        assert cal.read_values(f) == {'laser_yaw_deg': 179.2}

    def test_missing_or_corrupt(self, cal, config):
        assert cal.read_values(config / 'absent.yaml') == {}
        (config / 'bad.yaml').write_text('a: [')
        assert cal.read_values(config / 'bad.yaml') == {}


class TestStatus:
    def test_rows(self, cal, config, tmp_path):
        backup = tmp_path / 'backup'
        backup.mkdir()
        _ros_yaml(config / 'lsm9ds1_cal.local.yaml', 'pit_node', {'gyroscope.bias': [0, 0, 0]})
        (config / 'frames.local.yaml').write_text('laser_yaw_deg: 179.2\n')
        (backup / 'frames.local.yaml').write_text('laser_yaw_deg: 179.2\n')
        rows = {key: (state, detail) for key, state, detail in cal.status_rows(config, backup)}
        assert rows['imu'][0] == 'calibrated' and 'no backup' in rows['imu'][1]
        assert rows['lidar'][0] == 'calibrated' and 'backed up' in rows['lidar'][1]
        assert rows['realsense'][0] == 'default'
        assert rows['odom'] == ('pending', 'arrives with phase 2')

    def test_changed_backup_is_flagged(self, cal, config, tmp_path):
        (config / 'frames.local.yaml').write_text('laser_yaw_deg: 179.2\n')
        (tmp_path / 'frames.local.yaml').write_text('laser_yaw_deg: 180.0\n')
        assert cal.backup_state(config / 'frames.local.yaml', tmp_path) == 'backup differs'


class TestRun:
    @pytest.fixture
    def harness(self, cal, config, tmp_path, monkeypatch):
        """Stub the tools and prompts; returns (answers list, commands run)."""
        ran = []
        answers = []

        def fake_run(cmd, check=False, **_kw):
            ran.append(cmd)
            if str(cmd[-1]).endswith('calibrate_lidar.py'):
                (config / 'frames.local.yaml').write_text('laser_yaw_deg: 179.5\n')
            return type('R', (), {'returncode': 0, 'stdout': 'active\n'})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        monkeypatch.setattr('builtins.input', lambda _p='': answers.pop(0))
        monkeypatch.setenv('RACECAR_SUDO', '')
        return answers, ran

    def test_run_backs_up_and_reports(self, cal, config, tmp_path, harness, capsys):
        answers, ran = harness
        answers += ['', 'n']  # run lidar; no restart
        lidar = [s for s in cal.STEPS if s.key == 'lidar']
        backup = tmp_path / 'backup'
        assert cal.run_steps(lidar, config, backup) == 0
        out = capsys.readouterr().out
        assert 'laser_yaw_deg: default -> 179.5' in out
        assert (backup / 'frames.local.yaml').read_text() == 'laser_yaw_deg: 179.5\n'
        assert 'Apply later: systemctl restart racecar-teleop' in out
        assert len(ran) == 1

    def test_skip_and_quit(self, cal, config, tmp_path, harness):
        answers, ran = harness
        answers += ['s', 'q']
        steps = [s for s in cal.STEPS if s.tool is not None]
        assert cal.run_steps(steps, config, tmp_path / 'b') == 0
        assert ran == []

    def test_restart_on_yes(self, cal, config, tmp_path, harness):
        answers, ran = harness
        answers += ['', 'y']
        lidar = [s for s in cal.STEPS if s.key == 'lidar']
        cal.run_steps(lidar, config, tmp_path / 'b')
        assert ran[-1] == ['systemctl', 'restart', 'racecar-teleop']


class TestMain:
    def test_unknown_step_is_a_usage_error(self, cal):
        assert cal.main(['gps']) == 2

    def test_pending_only_is_a_usage_error(self, cal):
        assert cal.main(['odom']) == 2

    def test_refuses_without_teleop(self, cal, monkeypatch):
        monkeypatch.setattr(cal, 'teleop_active', lambda: False)
        assert cal.main(['imu']) == 1

    def test_status_needs_no_ros_graph(self, cal, capsys):
        assert cal.main(['status']) == 0
        out = capsys.readouterr().out
        for key in cal.STEP_KEYS:
            assert key in out
