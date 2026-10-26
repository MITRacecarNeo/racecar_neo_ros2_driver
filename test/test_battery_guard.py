"""Unit tests for scripts/battery_guard.py: levels, countdown, log, shutdown sequence."""

import csv
import json
import subprocess
from types import SimpleNamespace

from conftest import load_script
import pytest
from racecar_neo_ros2_driver.battery import BatteryConfig

bg = load_script('battery_guard')

# At 2 A on the default curve and 4700 mAh: 7.40 V is about 27 min left,
# 7.30 V about 12 min, 7.22 V about 2 min; the floor is 7.1 V.
OK_V, WARN_V, SHORT_V, FLOOR_V = 7.40, 7.30, 7.22, 7.05


def _run(guard, volts, seconds, start, amps=2.0, feed=True):
    """Feed one sample a second (or none, with feed=False) and tick; return (end time, events)."""
    events = []
    t = start
    for _ in range(int(seconds)):
        if feed:
            guard.add((t, volts, amps))
        events += guard.tick(t)
        t += 1.0
    return t, events


class TestLevels:
    def test_ok_while_plenty_left(self):
        guard = bg.Guard(BatteryConfig())
        _, events = _run(guard, OK_V, 400, 0.0)
        assert guard.level == 'ok' and events == []

    def test_warns_after_the_hold(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, OK_V, 300, 0.0)
        t, events = _run(guard, WARN_V, 90, t)
        assert guard.level == 'ok' and events == []
        t, events = _run(guard, WARN_V, 60, t)
        assert guard.level == 'warn' and events == ['warn']

    def test_warning_clears_with_margin_and_hold(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, WARN_V, 400, 0.0)
        assert guard.level == 'warn'
        t, events = _run(guard, OK_V, 30, t)
        assert guard.level == 'warn'
        t, events = _run(guard, OK_V, 120, t)
        assert guard.level == 'ok' and 'clear' in events

    def test_stale_data_never_warns(self):
        guard = bg.Guard(BatteryConfig())
        # Valid from 30 samples; the warning would need 60 s more.
        t, _ = _run(guard, WARN_V, 45, 0.0)
        _, events = _run(guard, WARN_V, 300, t, feed=False)
        assert guard.level == 'ok' and events == []
        assert not guard.last.valid

    def test_implausible_data_never_starts_a_countdown(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, OK_V, 100, 0.0)
        _, events = _run(guard, 0.0, 200, t)
        assert guard.level == 'ok' and events == []


class TestFloor:
    def test_countdown_then_shutdown(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, OK_V, 100, 0.0)
        t, events = _run(guard, FLOOR_V, 40, t)
        assert events == ['countdown floor']
        assert guard.level == 'countdown' and guard.trigger == 'floor'
        # The 30 s median crosses once more than half the window is under the floor.
        start = guard.shutdown_at - bg.COUNTDOWN_SEC
        assert t - 25 <= start <= t - 15
        t, events = _run(guard, FLOOR_V, guard.shutdown_at - t, t)
        assert guard.level == 'countdown' and events == []
        _, events = _run(guard, FLOOR_V, 2, t)
        assert events == ['shutdown'] and guard.level == 'off'

    def test_a_short_dip_does_not_trigger(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, OK_V, 100, 0.0)
        t, _ = _run(guard, FLOOR_V, 10, t)
        _run(guard, OK_V, 60, t)
        assert guard.level == 'ok'

    def test_floor_cannot_be_canceled(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, FLOOR_V, 40, 0.0)
        assert not guard.cancel(t)
        assert guard.level == 'countdown'

    def test_countdown_finishes_without_data(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, FLOOR_V, 40, 0.0)
        _, events = _run(guard, FLOOR_V, 70, t, feed=False)
        assert events == ['shutdown']

    def test_floor_scales_with_cells(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, OK_V * 1.5, 100, 0.0)
        assert guard.cells == 3
        _, events = _run(guard, FLOOR_V * 1.5, 40, t)
        assert events == ['countdown floor']

    def test_floor_override(self):
        guard = bg.Guard(BatteryConfig(floor_cell_volts=3.75))
        _, events = _run(guard, OK_V, 40, 0.0)
        assert events == ['countdown floor']


class TestEstimateShutdown:
    def test_warns_only_when_disabled(self):
        guard = bg.Guard(BatteryConfig())
        _, events = _run(guard, SHORT_V, 400, 0.0)
        assert guard.level == 'warn' and events == ['warn']

    def test_countdown_when_enabled(self):
        guard = bg.Guard(BatteryConfig(shutdown_enabled=True))
        t, events = _run(guard, SHORT_V, 89, 0.0)
        assert guard.level != 'countdown'
        _, events = _run(guard, SHORT_V, 10, t)
        assert 'countdown estimate' in events

    def test_cancel_once(self):
        guard = bg.Guard(BatteryConfig(shutdown_enabled=True))
        t, _ = _run(guard, SHORT_V, 100, 0.0)
        assert guard.level == 'countdown'
        assert guard.cancel(t)
        assert guard.level == 'warn' and guard.shutdown_at is None
        assert not guard.cancel(t)
        t, events = _run(guard, SHORT_V, 300, t)
        assert guard.level == 'warn' and events == []

    def test_floor_still_triggers_after_a_cancel(self):
        guard = bg.Guard(BatteryConfig(shutdown_enabled=True))
        t, _ = _run(guard, SHORT_V, 100, 0.0)
        guard.cancel(t)
        _, events = _run(guard, FLOOR_V, 40, t)
        assert events == ['countdown floor']


class TestCells:
    def test_detected_from_the_first_sample(self):
        guard = bg.Guard(BatteryConfig())
        assert guard.add((0.0, 8.2, 2.0)) is True
        assert guard.add((1.0, 8.2, 2.0)) is False
        assert guard.cells == 2 and guard.cells_known

    def test_unknown_battery_still_gets_a_floor(self):
        guard = bg.Guard(BatteryConfig())
        guard.add((0.0, 9.0, 2.0))
        assert guard.cells == 2 and not guard.cells_known

    def test_config_sets_cells(self):
        guard = bg.Guard(BatteryConfig(cells=3))
        guard.add((0.0, 8.2, 2.0))
        assert guard.cells == 3


class TestStatusAndLog:
    def test_status_values(self):
        guard = bg.Guard(BatteryConfig())
        t, _ = _run(guard, FLOOR_V, 40, 0.0)
        values = bg.status_values(guard, t)
        assert values['level'] == 'countdown' and values['trigger'] == 'floor'
        assert values['cells'] == '2' and values['valid'] == 'true'
        assert values['minutes_left'] == '0'
        assert values['shutdown_in'] == f'{guard.shutdown_at - t:.0f}'
        assert set(values) >= {'volts', 'soc', 'power_w', 'capacity_mah', 'cancel_used'}

    def test_status_before_data(self):
        values = bg.status_values(bg.Guard(BatteryConfig()), 0.0)
        assert values['valid'] == 'false' and values['minutes_left'] == ''

    def test_csv_log_appends_with_one_header(self, tmp_path):
        guard = bg.Guard(BatteryConfig())
        _run(guard, OK_V, 40, 0.0)
        path = tmp_path / 'battery' / 'run.csv'
        log = bg.CsvLog(path)
        log.write(bg.csv_row((39.0, OK_V, 2.0), guard, ''))
        log.close()
        log = bg.CsvLog(path)
        log.write(bg.csv_row(None, guard, 'warn'))
        log.close()
        assert b'\r' not in path.read_bytes()
        rows = list(csv.reader(path.open()))
        assert rows[0] == list(bg.CSV_FIELDS)
        assert len(rows) == 3
        assert rows[1][:3] == ['39.000', '7.4000', '2.0000']
        assert rows[1][8] == 'ok' and rows[2][9] == 'warn'
        assert rows[2][1] == ''

    def test_last_shutdown_file(self, tmp_path):
        guard = bg.Guard(BatteryConfig())
        _run(guard, FLOOR_V, 40, 0.0)
        path = bg.write_last_shutdown(tmp_path, 'floor', 123.0, guard)
        data = json.loads(path.read_text())
        assert data['trigger'] == 'floor' and data['time'] == 123.0
        assert data['volts'] == pytest.approx(FLOOR_V)


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
        lines = bg.shutdown_sequence(run)
        assert calls[0][-2:] == [str(bg.SCRIPT_DIR / 'racecar_log.py'), 'stop']
        stopped = [c[-1] for c in calls if 'stop' in c and 'systemctl' in c]
        assert stopped == [*bg.DRIVING_UNITS, bg.AUTONOMY_UNIT, bg.TELEOP_UNIT]
        assert calls[-1][-1] == 'poweroff' and '--check-inhibitors=no' in calls[-1]
        assert all(line.endswith('ok') for line in lines)

    def test_every_call_skips_password_prompts(self):
        run, calls = self._fake()
        bg.shutdown_sequence(run)
        assert all('--no-ask-password' in c for c in calls if c[0] == 'systemctl')

    def test_failures_do_not_stop_the_power_off(self):
        run, calls = self._fake(fail=[bg.AUTONOMY_UNIT], raise_on=[bg.TELEOP_UNIT])
        lines = bg.shutdown_sequence(run)
        assert calls[-1][-1] == 'poweroff'
        assert any('failed (denied)' in line for line in lines)
        assert any(bg.TELEOP_UNIT in line and 'failed' in line for line in lines)

    def test_each_step_is_reported_as_it_finishes(self):
        order = []
        run, _ = self._fake()

        def logged_run(argv, **kwargs):
            order.append(('run', argv[-1]))
            return run(argv, **kwargs)

        bg.shutdown_sequence(logged_run, report=lambda line: order.append(('report', line)))
        kinds = [k for k, _ in order]
        assert kinds == ['run', 'report'] * (len(kinds) // 2)

    def test_dry_run_runs_nothing(self):
        run, calls = self._fake()
        lines = bg.shutdown_sequence(run, dry_run=True)
        assert calls == []
        assert lines[-1] == 'power off: dry run, not run'
