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
    expected = ('build', 'test', 'source', 'teleop', 'launch',
                'clear', 'udev', 'selftest', 'status')
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


def test_status_runs_without_error():
    # status is read-only and idempotent; it should always succeed even with
    # no ros2 daemon / no peripherals.
    result = _run('status')
    assert result.returncode == 0
    assert 'USB peripherals' in result.stdout
    assert 'Stable device symlinks' in result.stdout


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
