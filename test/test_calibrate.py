"""Unit tests for scripts/calibration/calibrate.py (`racecar calibrate`)."""

import json
from pathlib import Path
import subprocess
import sys

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
        assert rows['camera'] == ('pending', 'arrives before phase 3')
        assert rows['odom'][0] == 'default'

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
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
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
        assert cal.main(['camera']) == 2

    def test_refuses_without_teleop(self, cal, monkeypatch):
        monkeypatch.setattr(cal, 'teleop_active', lambda: False)
        assert cal.main(['imu']) == 1

    def test_status_needs_no_ros_graph(self, cal, capsys):
        assert cal.main(['status']) == 0
        out = capsys.readouterr().out
        for key in cal.STEP_KEYS:
            assert key in out


class TestChecks:
    def test_every_check_step_has_a_check_flag_in_its_tool(self, cal):
        for step in cal.STEPS:
            if step.check:
                result = subprocess.run(
                    [sys.executable, str(cal.HERE / step.tool), '--help'],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                assert '--check' in result.stdout, step.tool

    def test_run_checks_prints_every_criterion(self, cal, monkeypatch, capsys):
        reports = {
            'calibrate_imu.py': (
                'PASS',
                [('gravity', '9.815 m/s^2 (+0.09%)', 'within 1%', 'PASS')],
            ),
            'calibrate_mag.py': (
                'FAIL',
                [
                    ('centre offset', '10.2% of radius', 'under 10%', 'FAIL', 'recalibrate'),
                    ('field strength', '101.3 uT', '20 to 70 uT', 'WARN', 'gain?'),
                ],
            ),
        }
        ran = []

        def fake_run(cmd, check=False, **_kw):
            ran.append(cmd)
            verdict, rows = reports[Path(cmd[1]).name]
            crit = [
                dict(zip(('name', 'measured', 'limit', 'status', 'advice'), r, strict=False))
                for r in rows
            ]
            Path(cmd[cmd.index('--report') + 1]).write_text(
                json.dumps({'overall': verdict, 'criteria': crit})
            )
            return type('R', (), {'returncode': 1 if verdict == 'FAIL' else 0})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        answers = ['', '', 's']
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        steps = [s for s in cal.STEPS if s.key in ('imu', 'mag', 'lidar')]
        assert cal.run_checks(steps) == 1
        out = capsys.readouterr().out
        start = out.index('=== Check summary ===')
        summary = out[start:]
        assert 'imu        PASS' in summary and 'mag        FAIL' in summary
        assert 'lidar      skipped' in summary
        assert 'FAIL  centre offset' in summary and '-> recalibrate' in summary
        assert 'WARN  field strength' in summary
        assert 'RESULT FAIL  1 pass  0 warn  1 fail  1 skipped' in summary
        assert all('--check' in cmd for cmd in ran)

    def test_tool_without_a_report_counts_as_a_fail(self, cal, monkeypatch, capsys):
        monkeypatch.setattr(
            cal.subprocess, 'run', lambda *a, **k: type('R', (), {'returncode': 1})()
        )
        monkeypatch.setattr(cal, 'ask', lambda _p='': '')
        steps = [s for s in cal.STEPS if s.key == 'imu']
        assert cal.run_checks(steps) == 1
        assert 'imu        FAIL' in capsys.readouterr().out

    def test_check_routes_only_checkable_steps(self, cal, monkeypatch):
        seen = []
        monkeypatch.setattr(cal, 'teleop_active', lambda: True)
        monkeypatch.setattr(
            cal, 'run_checks', lambda steps: seen.extend(s.key for s in steps) or 0
        )
        assert cal.main(['check']) == 0
        assert seen == ['imu', 'mag', 'realsense', 'lidar', 'odom']

    def test_check_of_a_pending_step_is_a_usage_error(self, cal):
        assert cal.main(['check', 'camera']) == 2


class TestAsk:
    def _stdin(self, cal, monkeypatch, data: bytes):
        buf = bytearray(data)

        def read(fd, n):
            assert fd == 0
            out = bytes(buf[:n])
            del buf[:n]
            return out

        monkeypatch.setattr(cal.os, 'read', read)
        return buf

    def test_reads_exactly_one_line(self, cal, monkeypatch):
        rest = self._stdin(cal, monkeypatch, b'S\nfor the tool\n')
        assert cal.ask('? ') == 's'
        assert bytes(rest) == b'for the tool\n'

    def test_end_of_input_stops(self, cal, monkeypatch):
        self._stdin(cal, monkeypatch, b'')
        assert cal.ask('? ') == 'q'


class TestNotRun:
    def test_stopping_lists_the_rest(self, cal, monkeypatch, capsys):
        monkeypatch.setattr(cal, 'ask', lambda _p='': 'q')
        steps = [s for s in cal.STEPS if s.key in ('imu', 'mag')]
        assert cal.run_checks(steps) == 0
        out = capsys.readouterr().out
        assert 'imu        not run' in out and 'mag        not run' in out
        assert '2 not run' in out

    def test_read_line_raises_at_end_of_input(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.os, 'read', lambda fd, n: b'')
        with pytest.raises(EOFError):
            common.read_line()


class TestColor:
    def _results(self, cal):
        common = load_script('calibrate_common')
        step = next(s for s in cal.STEPS if s.key == 'mag')
        crit = [
            common.Criterion('centre offset', '4.8%', 'under 10%', 'PASS'),
            common.Criterion('field strength', '104 uT', '20 to 70', 'WARN', 'gain?'),
        ]
        return [(step, 'WARN', crit)]

    def test_plain_without_color(self, cal):
        text = '\n'.join(cal.summary_lines(self._results(cal), color=False))
        assert '\033[' not in text
        assert 'RESULT WARN  0 pass  1 warn  0 fail  0 skipped' in text

    def test_colors_match_racecar_status(self, cal):
        text = '\n'.join(cal.summary_lines(self._results(cal), color=True))
        assert '\033[32mPASS ' in text  # green criterion
        assert '\033[33mWARN ' in text  # yellow criterion
        assert '\033[1;33mWARN' in text  # bold verdict and result
        assert '\033[33m1 warn\033[0m' in text
        assert '0 fail' in text and '\033[31m0 fail' not in text  # zero counts stay plain

    def test_no_color_opt_out(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.sys.stdout, 'isatty', lambda: True)
        monkeypatch.setenv('TERM', 'xterm')
        monkeypatch.delenv('NO_COLOR', raising=False)
        assert common.use_color() is True
        monkeypatch.setenv('NO_COLOR', '1')
        assert common.use_color() is False

    def test_not_a_terminal(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.sys.stdout, 'isatty', lambda: False)
        assert common.use_color() is False
