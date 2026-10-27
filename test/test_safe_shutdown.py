"""Unit tests for scripts/safe_shutdown.py: step order, failures, per-step reports."""

import subprocess
from types import SimpleNamespace

from conftest import load_script

ss = load_script('safe_shutdown')


class TestShutdownSequence:
    def _fake(self, fail=(), raise_on=()):
        calls = []

        def run(argv, **kwargs):
            calls.append(argv)
            if any(x in argv for x in raise_on):
                raise subprocess.TimeoutExpired(argv, kwargs.get('timeout'))
            bad = any(x in argv for x in fail)
            return SimpleNamespace(returncode=1 if bad else 0, stdout='', stderr='denied')

        return run, calls

    def test_order(self):
        run, calls = self._fake()
        lines = ss.shutdown_sequence(run)
        assert calls[0][-2:] == [str(ss.SCRIPT_DIR / 'racecar_log.py'), 'stop']
        stopped = [c[-1] for c in calls if 'stop' in c and 'systemctl' in c]
        assert stopped == [*ss.DRIVING_UNITS, ss.AUTONOMY_UNIT, ss.TELEOP_UNIT]
        assert calls[-1][-1] == 'poweroff' and '--check-inhibitors=no' in calls[-1]
        assert all(line.endswith('ok') for line in lines)

    def test_every_call_skips_password_prompts(self):
        run, calls = self._fake()
        ss.shutdown_sequence(run)
        assert all('--no-ask-password' in c for c in calls if c[0] == 'systemctl')

    def test_failures_do_not_stop_the_power_off(self):
        run, calls = self._fake(fail=[ss.AUTONOMY_UNIT], raise_on=[ss.TELEOP_UNIT])
        lines = ss.shutdown_sequence(run)
        assert calls[-1][-1] == 'poweroff'
        assert any('failed (denied)' in line for line in lines)
        assert any(ss.TELEOP_UNIT in line and 'failed' in line for line in lines)

    def test_each_step_is_reported_as_it_finishes(self):
        order = []
        run, _ = self._fake()

        def logged_run(argv, **kwargs):
            order.append(('run', argv[-1]))
            return run(argv, **kwargs)

        ss.shutdown_sequence(logged_run, report=lambda line: order.append(('report', line)))
        kinds = [k for k, _ in order]
        assert kinds == ['run', 'report'] * (len(kinds) // 2)

    def test_dry_run_runs_nothing(self):
        run, calls = self._fake()
        lines = ss.shutdown_sequence(run, dry_run=True)
        assert calls == []
        assert lines[-1] == 'power off: dry run, not run'
