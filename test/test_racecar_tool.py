"""Tests for scripts/racecar-tool.sh (the `racecar` shell function)."""

from pathlib import Path
import subprocess

import pytest

TOOL = Path(__file__).parent.parent / 'scripts' / 'racecar-tool.sh'


def _run(*args):
    """Source the tool in a non-interactive bash and invoke `racecar <args>`."""
    script = f'set +u; source "{TOOL}"; racecar {" ".join(args)}'
    return subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=15,
    )


def test_tool_file_exists():
    assert TOOL.is_file()


def test_bash_syntax_clean():
    result = subprocess.run(
        ['bash', '-n', str(TOOL)],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, f'bash -n failed:\n{result.stderr}'


def test_sourcing_defines_racecar_function():
    script = f'source "{TOOL}" && type -t racecar'
    result = subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.strip() == 'function'


@pytest.mark.parametrize('args', [[], ['help'], ['--help'], ['-h']])
def test_help_renders(args):
    result = _run(*args)
    assert result.returncode == 0
    assert 'racecar' in result.stdout
    assert 'Commands' in result.stdout
    expected = ('build', 'test', 'source', 'cd', 'teleop', 'launch',
                'clear', 'udev', 'watchdog', 'service', 'setup', 'library',
                'cleanup', 'selftest', 'status')
    for sub in expected:
        assert sub in result.stdout, f'help missing "{sub}"'


def test_unknown_command_errors():
    result = _run('bogus_subcommand')
    assert result.returncode == 2
    assert 'unknown command' in result.stderr


def test_launch_without_name_errors():
    result = _run('launch')
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_clear_without_target_errors():
    result = _run('clear')
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_clear_rejects_unknown_flag():
    result = _run('clear', '--cosmic-rays')
    assert result.returncode == 2
    assert 'unknown flag' in result.stderr


def test_selftest_without_target_errors():
    result = _run('selftest')
    assert result.returncode == 2
    assert 'usage:' in result.stderr
    assert '--dmatrix' in result.stderr


def test_selftest_rejects_unknown_flag():
    result = _run('selftest', '--maestro')
    assert result.returncode == 2
    assert 'unknown flag' in result.stderr


# Skipping a "dotmatrix_node is not running" test on purpose: it depends on
# host state (whether the user has a node running) and either side of that
# state is a valid test environment, so the assertion is unreliable.


def test_cd_changes_pwd_to_package_root():
    # `cd` must run in the user's shell context (no subshell), so a single
    # bash session that sources the tool, runs `racecar cd`, then echoes PWD
    # should print the package root.
    script = (
        f'set +u; source "{TOOL}"; '
        'racecar cd && pwd'
    )
    result = subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.strip().endswith('racecar_neo_ros2_driver')


def test_status_runs_without_error():
    # status is read-only and idempotent; it should always succeed even with
    # no ros2 daemon / no peripherals.
    result = _run('status')
    assert result.returncode == 0
    assert 'USB peripherals' in result.stdout
    assert 'Stable device symlinks' in result.stdout


class TestService:
    """`racecar service` covers install/start/stop/restart/enable/disable/logs/status."""

    def test_status_action_runs(self):
        # Default action is `status`, which just calls `systemctl is-active`
        # for each unit. No sudo required, no side effects.
        result = _run('service', 'status')
        assert result.returncode == 0
        # status output enumerates each unit name.
        for unit in ('racecar-teleop', 'racecar-watchdog',
                     'racecar-dashboard', 'racecar-jupyter'):
            assert unit in result.stdout, f'status missing {unit}'

    def test_default_action_is_status(self):
        # `racecar service` with no action should fall through to status.
        result = _run('service')
        assert result.returncode == 0
        assert 'racecar-teleop' in result.stdout

    def test_help_action(self):
        result = _run('service', 'help')
        assert result.returncode == 0
        for action in ('install', 'start', 'stop', 'status', 'logs'):
            assert action in result.stdout

    def test_rejects_unknown_action(self):
        result = _run('service', 'flambé')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr


class TestSetup:
    """`racecar setup` dispatches to setup_all.sh / setup_networking.sh."""

    def test_no_phase_errors(self):
        result = _run('setup')
        assert result.returncode == 2
        assert 'phases:' in result.stderr

    def test_unknown_phase_errors(self):
        result = _run('setup', 'whatever')
        assert result.returncode == 2
        assert 'unknown phase' in result.stderr

    def test_networking_help(self):
        result = _run('setup', 'networking', '--help')
        assert result.returncode == 0
        for flag in ('--ssid', '--psk', '--channel', '--ap-addr',
                     '--eth-static', '--show', '--reset'):
            assert flag in result.stdout

    def test_networking_unknown_flag_errors(self):
        result = _run('setup', 'networking', '--gloryhole')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_networking_show_with_no_persisted_file(self, tmp_path, monkeypatch):
        # --show with no $HOME/.config/racecar/networking.env should report
        # "No persisted networking config" and not invoke the script.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking --show'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert 'No persisted networking config' in result.stdout

    def test_networking_reset_removes_persisted_file(self, tmp_path, monkeypatch):
        monkeypatch.setenv('HOME', str(tmp_path))
        cfg_dir = tmp_path / '.config' / 'racecar'
        cfg_dir.mkdir(parents=True)
        cfg_file = cfg_dir / 'networking.env'
        cfg_file.write_text('RACECAR_AP_SSID="dummy"\n')
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking --reset'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert not cfg_file.exists()
        assert 'Cleared' in result.stdout

    def test_networking_flag_persists_when_combined_with_show(self, tmp_path, monkeypatch):
        # Regression: an earlier impl treated --show as a short-circuit BEFORE
        # writing vals[] to the file. The two-pass parse fixes that: --ssid
        # gathered, --show registered as action, persist runs, then --show
        # prints the (now-up-to-date) file.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; '
             'racecar setup networking --ssid=test-ssid --psk=test-pass --show'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        cfg_file = tmp_path / '.config' / 'racecar' / 'networking.env'
        assert cfg_file.exists(), '--show should NOT short-circuit before persistence'
        text = cfg_file.read_text()
        assert 'RACECAR_AP_SSID="test-ssid"' in text
        assert 'RACECAR_AP_PSK="test-pass"' in text
        # And --show output should reflect what was just written.
        assert 'Persisted networking config' in result.stdout
        assert 'test-ssid' in result.stdout

    def test_networking_reset_with_overrides_errors(self, tmp_path, monkeypatch):
        # --reset + --ssid=foo is nonsense; the new value would be lost
        # immediately. Reject rather than do something surprising.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; '
             'racecar setup networking --ssid=foo --reset'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 2
        assert 'cannot be combined' in result.stderr


class TestLibrary:
    """`racecar library` manages racecar_student.pth in user site-packages."""

    @staticmethod
    def _run_isolated(home, *args):
        # Override HOME so site.getusersitepackages() resolves to a tmp dir
        # and ~/jupyter_ws probes a tmp tree. PYTHONUSERBASE pins the user-site
        # path under HOME on systems where it would otherwise resolve elsewhere.
        env_setup = (
            f'export HOME="{home}"; '
            f'export PYTHONUSERBASE="{home}/.local"; '
        )
        return subprocess.run(
            ['bash', '-c',
             f'set +u; {env_setup} source "{TOOL}"; '
             f'racecar library {" ".join(args)}'],
            capture_output=True, text=True, timeout=10,
        )

    def test_no_action_errors(self):
        result = _run('library')
        assert result.returncode == 2
        assert 'usage:' in result.stderr

    def test_help_lists_actions(self):
        result = _run('library', '--help')
        assert result.returncode == 0
        for flag in ('--select', '--list', '--reset', '--status'):
            assert flag in result.stdout

    def test_unknown_flag_errors(self):
        result = _run('library', '--vaporize')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_status_with_no_pth(self, tmp_path):
        # Fresh HOME → no .pth file → friendly hint, exit 0.
        result = self._run_isolated(tmp_path, '--status')
        assert result.returncode == 0
        assert 'No racecar library is currently selected' in result.stdout
        assert '--select' in result.stdout

    def test_list_with_no_jupyter_ws(self, tmp_path):
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        assert 'No ~/jupyter_ws/ directory' in result.stdout

    def test_list_skips_folders_without_racecar_core(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        # Valid candidate
        (jws / 'goodlib' / 'library').mkdir(parents=True)
        (jws / 'goodlib' / 'library' / 'racecar_core.py').write_text('')
        # Bogus: no library/ at all
        (jws / 'badlib').mkdir(parents=True)
        # Bogus: library/ exists but no racecar_core.py
        (jws / 'emptylib' / 'library').mkdir(parents=True)
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        assert 'goodlib' in result.stdout
        assert 'badlib' not in result.stdout
        assert 'emptylib' not in result.stdout

    def test_select_writes_pth_file(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        result = self._run_isolated(tmp_path, '--select', 'mylib')
        assert result.returncode == 0, result.stderr
        assert 'Selected library' in result.stdout
        # The .pth file should land somewhere under HOME/.local and contain libdir.
        pth_files = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_files) == 1, f'expected one .pth, found {pth_files}'
        assert pth_files[0].read_text().strip() == str(libdir)

    def test_select_with_equals_form(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        result = self._run_isolated(tmp_path, '--select=mylib')
        assert result.returncode == 0, result.stderr
        pth_files = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_files) == 1
        assert pth_files[0].read_text().strip() == str(libdir)

    def test_select_rejects_missing_folder(self, tmp_path):
        (tmp_path / 'jupyter_ws').mkdir()
        result = self._run_isolated(tmp_path, '--select', 'ghost')
        assert result.returncode == 2
        assert 'not a folder' in result.stderr

    def test_select_rejects_folder_without_racecar_core(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        (jws / 'shell' / 'library').mkdir(parents=True)
        # Note: no racecar_core.py
        result = self._run_isolated(tmp_path, '--select', 'shell')
        assert result.returncode == 2
        assert 'racecar_core.py' in result.stderr

    def test_select_requires_target(self):
        # `--select` with no following arg.
        result = _run('library', '--select')
        assert result.returncode == 2
        assert 'requires a folder name' in result.stderr

    def test_reset_removes_pth(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        # Select, then reset.
        self._run_isolated(tmp_path, '--select', 'mylib')
        pth_before = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_before) == 1
        result = self._run_isolated(tmp_path, '--reset')
        assert result.returncode == 0
        assert 'removed' in result.stdout
        pth_after = list(tmp_path.rglob('racecar_student.pth'))
        assert pth_after == []

    def test_reset_when_nothing_to_remove(self, tmp_path):
        result = self._run_isolated(tmp_path, '--reset')
        assert result.returncode == 0
        assert 'no .pth file to remove' in result.stdout

    def test_status_after_select_reports_path(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        self._run_isolated(tmp_path, '--select', 'mylib')
        result = self._run_isolated(tmp_path, '--status')
        assert result.returncode == 0
        assert 'Current library' in result.stdout
        assert str(libdir) in result.stdout

    def test_list_marks_current_with_asterisk(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        for name in ('alpha', 'beta'):
            libdir = jws / name / 'library'
            libdir.mkdir(parents=True)
            (libdir / 'racecar_core.py').write_text('')
        self._run_isolated(tmp_path, '--select', 'beta')
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        # Find the line for beta and check it has a '*' marker.
        lines = [ln for ln in result.stdout.splitlines() if 'beta' in ln]
        assert lines, 'beta missing from --list output'
        assert '*' in lines[0]
        # alpha line should NOT have a star (just leading whitespace).
        alpha_lines = [ln for ln in result.stdout.splitlines()
                       if 'alpha' in ln]
        assert alpha_lines
        assert '*' not in alpha_lines[0]


class TestCleanup:
    def test_dry_run_default_is_safe(self):
        # Dry-run default: must always exit 0 and never invoke kill/rm.
        result = _run('cleanup')
        assert result.returncode == 0
        # Either the process inventory or the SHM section should appear; both
        # have predictable headings or 'No ...' fallback.
        assert 'racecar processes' in result.stdout.lower() or \
               'no racecar processes' in result.stdout.lower()
        assert 'fastrtps shm' in result.stdout.lower() or \
               'no fastrtps' in result.stdout.lower()

    def test_dry_run_marker_appears_when_things_found(self):
        # If the test environment has any racecar process or SHM orphan, the
        # output should label the action as dry-run (i.e. nothing was killed).
        # If nothing is found, the "No ..." messages stand alone — both fine.
        result = _run('cleanup')
        assert result.returncode == 0
        # The "(dry-run; pass --force to ...)" hint appears once per category
        # that found matches. We don't assert it must appear (clean system),
        # but if anything appeared, --force must not have been silently invoked.
        if 'pid=' in result.stdout:
            assert '(dry-run' in result.stdout

    def test_help_flag_describes_behavior(self):
        result = _run('cleanup', '--help')
        assert result.returncode == 0
        assert 'dry-run' in result.stdout
        assert '--force' in result.stdout

    def test_rejects_unknown_flag(self):
        result = _run('cleanup', '--burn-it-all')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr


class TestCompletionInstalled:
    def test_completion_function_defined(self):
        script = f'source "{TOOL}" && type -t _racecar_complete'
        result = subprocess.run(
            ['bash', '-c', script],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert result.stdout.strip() == 'function'

    def test_complete_command_registered(self):
        script = f'source "{TOOL}" && complete -p racecar'
        result = subprocess.run(
            ['bash', '-c', script],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert '_racecar_complete' in result.stdout
