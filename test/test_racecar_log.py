"""Unit tests for scripts/racecar_log.py (the `racecar log` implementation)."""

import importlib.util
import json
from pathlib import Path
import time

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'racecar_log.py'


def _load():
    """Import the script by path; scripts/ is not a package."""
    spec = importlib.util.spec_from_file_location('racecar_log', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rl = _load()


class TestNaming:
    def test_timestamp_then_test_name(self):
        now = time.mktime((2026, 9, 6, 14, 30, 0, 0, 0, -1))
        assert rl.build_bag_name('wallfollow', now) == '20260906_143000_wallfollow'

    def test_spaces_and_slashes_are_flattened(self):
        assert rl.sanitize_name('lap 3 / turn 2') == 'lap-3-turn-2'

    def test_empty_name_falls_back(self):
        assert rl.sanitize_name('') == 'run'
        assert rl.sanitize_name('///') == 'run'

    def test_safe_characters_survive(self):
        assert rl.sanitize_name('run_2.1-a') == 'run_2.1-a'


class TestRecordArgv:
    """The flags must match `ros2 bag record` on Jazzy, not older spellings."""

    def test_all_topics_uses_the_real_flag(self):
        argv = rl.build_record_argv('/data/bag')
        assert '--all-topics' in argv
        assert '--all' not in argv

    def test_exclude_uses_the_real_flag(self):
        argv = rl.build_record_argv('/data/bag', exclude=['/camera/.*'])
        assert argv[-2:] == ['--exclude-regex', '/camera/.*']
        assert '--exclude' not in argv

    def test_explicit_topics_replace_all(self):
        argv = rl.build_record_argv('/data/bag', topics=['/scan', '/odom'])
        assert '--all-topics' not in argv
        assert argv[-2:] == ['/scan', '/odom']

    def test_output_and_storage(self):
        argv = rl.build_record_argv('/data/bag', storage='sqlite3')
        assert argv[:3] == ['ros2', 'bag', 'record']
        assert '--output' in argv and '/data/bag' in argv
        assert argv[argv.index('--storage') + 1] == 'sqlite3'


class TestResolveLogRoot:
    def test_explicit_wins(self, tmp_path):
        assert rl.resolve_log_root(str(tmp_path)) == tmp_path

    def test_env_var_is_next(self, tmp_path):
        got = rl.resolve_log_root(None, environ={'RACECAR_LOG_DIR': str(tmp_path)})
        assert got == tmp_path

    def test_first_writable_candidate_wins(self, tmp_path):
        good = tmp_path / 'good'
        good.mkdir()
        got = rl.resolve_log_root(None, candidates=('/nonexistent', str(good)), environ={})
        assert got == good

    def test_falls_back_to_last_candidate(self):
        got = rl.resolve_log_root(
            None, candidates=('/nonexistent-a', '/nonexistent-b'), environ={},
        )
        assert str(got) == '/nonexistent-b'


class TestBandwidth:
    def test_all_topics_estimate_includes_both_cameras(self):
        assert rl.estimate_mbps([]) == pytest.approx(73.0)

    def test_named_topics_without_cameras_are_cheap(self):
        assert rl.estimate_mbps(['/scan', '/odom']) == 0.0

    def test_warns_when_cameras_go_to_an_sd_card(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: True)
        warning = rl.bandwidth_warning([], '/home/racecar', free_bytes=40_000_000_000)
        assert warning is not None
        assert 'dropped' in warning

    def test_silent_on_nvme(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: False)
        assert rl.bandwidth_warning([], '/data', free_bytes=400_000_000_000) is None

    def test_silent_for_a_cheap_selection(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: True)
        assert rl.bandwidth_warning(['/scan'], '/home/racecar') is None


class TestUnmountedNvme:
    def test_detects_a_raw_drive(self):
        out = 'mmcblk0 disk\nmmcblk0p2 part /\nnvme0n1 disk\n'
        assert rl.unmounted_nvme(out) == 'nvme0n1'

    def test_silent_when_the_drive_is_mounted(self):
        out = 'mmcblk0 disk\nmmcblk0p2 part /\nnvme0n1 disk\nnvme0n1p1 part /data\n'
        assert rl.unmounted_nvme(out) is None

    def test_silent_with_no_nvme(self):
        assert rl.unmounted_nvme('mmcblk0 disk\nmmcblk0p2 part /\n') is None


class TestFormatting:
    @pytest.mark.parametrize('n,expected', [
        (512, '512 B'), (2048, '2.0 KB'), (5 * 1024**2, '5.0 MB'), (3 * 1024**3, '3.0 GB'),
    ])
    def test_sizes(self, n, expected):
        assert rl.format_size(n) == expected

    @pytest.mark.parametrize('sec,expected', [
        (0, '0:00'), (65, '1:05'), (3661, '1:01:01'), (-5, '0:00'),
    ])
    def test_durations(self, sec, expected):
        assert rl.format_duration(sec) == expected


class TestConfig:
    def test_roundtrip(self, tmp_path):
        path = tmp_path / 'log.conf'
        rl.save_config({'TOPICS': '/scan,/odom', 'DIR': '/data'}, path)
        assert rl.load_config(path) == {'TOPICS': '/scan,/odom', 'DIR': '/data'}

    def test_empty_values_are_dropped(self, tmp_path):
        path = tmp_path / 'log.conf'
        rl.save_config({'TOPICS': '', 'DIR': '/data'}, path)
        assert rl.load_config(path) == {'DIR': '/data'}

    def test_missing_file_is_empty(self, tmp_path):
        assert rl.load_config(tmp_path / 'absent.conf') == {}

    def test_comments_ignored(self, tmp_path):
        path = tmp_path / 'log.conf'
        path.write_text('# a comment\nDIR=/data\n\n')
        assert rl.load_config(path) == {'DIR': '/data'}


class TestState:
    def test_roundtrip(self, tmp_path):
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 1, 'bag': '/data/b'}, path)
        assert rl.read_state(path)['bag'] == '/data/b'

    def test_missing_state_is_none(self, tmp_path):
        assert rl.read_state(tmp_path / 'absent') is None

    def test_corrupt_state_is_none(self, tmp_path):
        path = tmp_path / 'log.state'
        path.write_text('not json')
        assert rl.read_state(path) is None

    def test_clear_is_idempotent(self, tmp_path):
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 1}, path)
        rl.clear_state(path)
        rl.clear_state(path)
        assert rl.read_state(path) is None

    def test_pid_alive_on_self(self):
        import os
        assert rl.pid_alive(os.getpid()) is True

    def test_pid_alive_rejects_garbage(self):
        assert rl.pid_alive(None) is False
        assert rl.pid_alive('nope') is False


class TestSummarizeTopics:
    def test_sorted_by_count_descending(self):
        rows = rl.summarize_topics({'/a': 10, '/b': 300, '/c': 50}, 10.0)
        assert [r['topic'] for r in rows] == ['/b', '/c', '/a']

    def test_mean_rate(self):
        rows = rl.summarize_topics({'/scan': 100}, 10.0)
        assert rows[0]['hz'] == pytest.approx(10.0)

    def test_zero_duration_does_not_divide(self):
        rows = rl.summarize_topics({'/scan': 5}, 0.0)
        assert rows[0]['hz'] == 0.0

    def test_empty_bag(self):
        assert rl.summarize_topics({}, 10.0) == []


class TestDirSize:
    def test_sums_nested_files(self, tmp_path):
        (tmp_path / 'a').write_bytes(b'x' * 100)
        nested = tmp_path / 'sub'
        nested.mkdir()
        (nested / 'b').write_bytes(b'y' * 50)
        assert rl.dir_size(tmp_path) == 150

    def test_empty_dir(self, tmp_path):
        assert rl.dir_size(tmp_path) == 0


class TestCli:
    def test_parser_exposes_every_action(self):
        parser = rl.build_parser()
        actions = parser._subparsers._group_actions[0].choices.keys()
        assert set(actions) == {'start', 'stop', 'status', 'list', 'analyze', 'config'}

    def test_start_defaults_to_all_topics(self):
        args = rl.build_parser().parse_args(['start', 'lap1'])
        assert args.test_name == 'lap1'
        assert args.topics is None

    def test_stop_has_a_finalize_timeout(self):
        args = rl.build_parser().parse_args(['stop'])
        assert args.timeout == 15.0

    def test_status_reports_not_recording(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(rl, 'STATE_FILE', tmp_path / 'absent')
        assert rl.main(['status']) == 0
        assert 'not recording' in capsys.readouterr().out

    def test_status_json_when_idle(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(rl, 'STATE_FILE', tmp_path / 'absent')
        rl.main(['status', '--json'])
        assert json.loads(capsys.readouterr().out) == {'recording': False}

    def test_stale_state_is_cleared(self, tmp_path, monkeypatch):
        # A recorder killed by a reboot leaves state behind; treating that as
        # "recording" would make stop and status lie.
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 999999, 'bag': str(tmp_path / 'b'), 'started': 0}, path)
        monkeypatch.setattr(rl, 'STATE_FILE', path)
        assert rl.active_state() is None
        assert rl.read_state(path) is None
