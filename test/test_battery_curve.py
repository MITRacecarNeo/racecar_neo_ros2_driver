"""Unit tests for scripts/battery_curve.py."""

from pathlib import Path

from conftest import load_script
import pytest

bc = load_script('battery_curve')


def _write(path: Path, header: str, rows) -> Path:
    path.write_text(header + '\n' + ''.join(f'{t:.3f},{v:.4f},{a:.4f}\n' for t, v, a in rows))
    return path


def _discharge(minutes: float, r_ohm: float = 0.03, start: float = 1000.0):
    """1 Hz rows: open circuit 8.30 V falling 1.25 V in 2 h; 2.3 A, a 5.3 A burst a minute."""
    rows = []
    for i in range(int(minutes * 60)):
        ocv = 8.30 - 1.25 * i / 7200
        amps = 5.3 if (i // 10) % 6 == 5 else 2.3
        rows.append((start + i, ocv - amps * r_ohm, amps))
    return rows


class TestLoad:
    def test_guard_and_capture_logs_join_in_time_order(self, tmp_path):
        guard = tmp_path / 'b_guard.csv'
        guard.write_text(
            'time,volts,amps,soc,minutes_left,power_w,capacity_mah,cells,level,event\n'
            '20.000,8.10,2.3,99,120,18,4700,2,ok,\n'
            '21.000,,,,,,,,ok,warn\n'
        )
        capture = _write(tmp_path / 'a.csv', 't,volts,amps', [(10, 8.2, 2.2), (15, 8.15, 2.2)])
        rows = bc.load([guard, capture])
        assert rows == [(10.0, 8.2, 2.2), (15.0, 8.15, 2.2), (20.0, 8.1, 2.3)]


class TestCharge:
    def test_counts_mah_and_fills_gaps_at_the_last_current(self):
        rows = [(0.0, 8.0, 3.6), (1.0, 8.0, 3.6), (11.0, 8.0, 1.0)]
        assert bc.drawn_mah(rows) == pytest.approx([0.0, 1.0, 11.0])

    def test_watt_hours(self):
        rows = [(0.0, 8.0, 2.0), (3600.0, 8.0, 2.0)]
        assert bc.drawn_wh(rows) == pytest.approx(16.0)


class TestFit:
    def test_recovers_the_resistance(self):
        fits = bc.fit_windows(_discharge(10, r_ohm=0.032))
        assert fits
        assert all(ohms == pytest.approx(0.032, abs=0.003) for _, ohms in fits)

    def test_steady_current_has_no_windows(self):
        rows = [(float(i), 8.0 - i * 1e-4, 2.3) for i in range(200)]
        assert bc.fit_windows(rows) == []

    def test_window_where_volts_ignore_amps_is_dropped(self):
        rows = [(float(i), 8.0 + (0.05 if i % 7 == 0 else 0.0), 2.3 + (i % 3)) for i in range(20)]
        assert bc.fit_windows(rows) == []

    def test_window_across_a_gap_is_skipped(self):
        rows = _discharge(1)
        rows = rows[:10] + [(t + 600, v, a) for t, v, a in rows[10:]]
        assert all(start != rows[0][0] for start, _ in bc.fit_windows(rows))


class TestReplay:
    def test_floor_ends_the_replay_and_marks_are_scored(self):
        rows = _discharge(120)
        rep = bc.replay(rows, bc.config())
        assert rep.floor_t is not None and rep.times[-1] == rep.floor_t
        errors = bc.mark_errors(rep)
        assert set(errors) == {15.0, 5.0}
        assert all(e is not None for e in errors.values())

    def test_no_floor_no_errors(self):
        rep = bc.replay(_discharge(5), bc.config())
        assert rep.floor_t is None
        assert bc.mark_errors(rep) == {15.0: None, 5.0: None}

    def test_candidate_resistance_reaches_the_estimate(self):
        assert bc.config(0.03).r_internal_ohm == 0.03


class TestReport:
    def test_full_run(self, tmp_path):
        log = _write(tmp_path / 'run_guard.csv', 'time,volts,amps', _discharge(120))
        text = bc.report([log])
        assert '- Floor (7.10 V, 30 s median): reached' in text
        median = int(text.split('median ')[1].split(' mOhm')[0])
        assert '## r_internal' in text and 29 <= median <= 32
        assert '| 0 |' in text and '| 100 |' in text
        assert '- 15 min: actual' in text

    def test_partial_run_says_so(self, tmp_path):
        log = _write(tmp_path / 'run_guard.csv', 'time,volts,amps', _discharge(5))
        text = bc.report([log])
        assert 'not reached' in text
        assert 'too little used to measure the capacity' in text
        assert '- 15 min: not reached' in text

    def test_out_file(self, tmp_path, capsys):
        log = _write(tmp_path / 'run_guard.csv', 'time,volts,amps', _discharge(5))
        out = tmp_path / 'report.md'
        assert bc.main([str(log), '--out', str(out)]) == 0
        assert out.read_text() == capsys.readouterr().out
