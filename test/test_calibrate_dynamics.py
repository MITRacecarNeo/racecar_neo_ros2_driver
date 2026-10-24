"""Unit tests for calibrate_dynamics.py: circle fit, Ackermann angles, timing, scan chaining."""

import math

from conftest import load_script
import numpy as np
import pytest


@pytest.fixture(scope='module')
def md():
    return load_script('calibrate_dynamics')


class TestGeometry:
    def test_circle_fit(self, md):
        a = np.linspace(0, 5, 60)
        xy = np.column_stack((1.0 + 0.7 * np.cos(a), -2.0 + 0.7 * np.sin(a)))
        cx, cy, r, resid = md.fit_circle(xy)
        assert (cx, cy, r) == (pytest.approx(1.0), pytest.approx(-2.0), pytest.approx(0.7))
        assert resid < 1e-9

    def test_ackermann_inner_turns_more(self, md):
        s = md.ackermann(0.6)
        assert s.inner_deg > s.bicycle_deg > s.outer_deg
        assert s.bicycle_deg == pytest.approx(math.degrees(math.atan(0.23 / 0.6)))

    def test_outer_front_track_diameter(self, md):
        s = md.ackermann(0.6, wheelbase=0.23, track=0.178)
        assert s.outer_front_track_diameter_m == pytest.approx(2 * math.hypot(0.689, 0.23))


class TestTiming:
    def test_transition_of_a_ramp(self, md):
        t = np.arange(0.0, 2.0, 0.005)
        kappa = np.clip((t - 1.0) / 0.1, 0.0, 1.0) * 3.0 - 1.5  # -1.5 to +1.5 over 0.1 s
        rise, total = md.transition_times(t, kappa, 1.0, -1.5, 1.5)
        assert rise == pytest.approx(0.08, abs=0.006)
        assert total == pytest.approx(0.095, abs=0.006)

    def test_straight_metrics(self, md):
        t = np.arange(0.0, 8.0, 0.01)
        v = np.where(t < 2.0, 0.0, np.minimum((t - 2.0) / 1.0, 1.0) * 0.75)
        v = np.where(t >= 5.0, np.maximum(0.75 - (t - 5.0) * 1.5, 0.0), v)
        s = np.concatenate(([0.0], np.cumsum(v[:-1] * 0.01)))
        r = md.straight_metrics(t, v, s, 2.0, 5.0)
        assert r.steady_mps == pytest.approx(0.75, rel=1e-3)
        assert r.rise_90_s == pytest.approx(0.9, abs=0.02)
        assert r.stop_s == pytest.approx(0.5, abs=0.02)
        assert r.stop_m == pytest.approx(0.1875, abs=0.005)


class TestChain:
    def test_chain_recovers_a_circle(self, md):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        laser = (0.24, 0.0, math.pi)
        radius, n = 0.7, 40
        base = [
            (1.0 + radius * math.sin(a), 0.5 - radius * math.cos(a) + radius, a)
            for a in np.linspace(0.0, 2.2 * math.pi, n)
        ]
        scans, odom_t, odom = [], [], []
        for k, b in enumerate(base):
            lp = sm.compose(b, laser)
            pts = _points(sm, _scan(lp, noise=0.003, seed=k))
            scans.append((float(k), pts, np.full(len(pts), float(k))))
            odom_t.append(float(k))
            odom.append(sm.compose(sm.invert(base[0]), b))
        odom = np.array(odom)
        odom[:, 2] = np.unwrap(odom[:, 2])
        times, poses, rms = md.chain_scans(scans, np.array(odom_t), odom, laser)
        _, _, r, _ = md.fit_circle(poses[:, :2])
        assert r == pytest.approx(radius, abs=0.005)
        assert rms < 0.01


class TestKeys:
    def test_speed_tags(self, md):
        assert [md.speed_tag(v) for v in (0.25, 0.5, 1.0)] == ['025', '05', '10']

    def test_calibrate_lists_every_key_the_tool_writes(self, md):
        cal = load_script('calibrate')
        (step,) = [s for s in cal.STEPS if s.key == 'dynamics']
        written = {
            f'{n}_{md.speed_tag(r[1][0])}'
            for r in [md.RUNS[k] for k in ('straight025', 'straight05', 'straight10')]
            for n in ('steady_speed_mps', 'rise_90_s', 'stop_distance_mm', 'stop_time_s')
        }
        assert written <= set(step.keys)


class TestDriveGate:
    def _driver(self, md, monkeypatch, tmp_path, release_after_driving: bool):
        class Proc:
            def send_signal(self, sig):
                pass

            def wait(self, timeout=None):
                return 0

        monkeypatch.setattr(md.subprocess, 'Popen', lambda *a, **k: Proc())
        monkeypatch.setattr(md, 'load_config', lambda: {})
        monkeypatch.setattr(md, 'resolve_log_root', lambda d: tmp_path)

        class Log:
            def __init__(self):
                self.lines = []

            def info(self, m):
                self.lines.append(m)

            error = info

        d = md.Driver.__new__(md.Driver)
        d.log = Log()
        d.rb = True
        held = []

        def hold(seconds, speed, steer, need_rb):
            held.append((speed, need_rb))
            if release_after_driving and speed == 0.0 and any(s != 0.0 for s, _ in held):
                d.rb = False
            return d.rb or not need_rb

        d._hold = hold
        return d, held

    def test_releasing_rb_after_the_drive_keeps_the_run(self, md, monkeypatch, tmp_path):
        d, held = self._driver(md, monkeypatch, tmp_path, release_after_driving=True)
        assert d.drive('straight05') is not None
        assert 'You can release RB now.' in d.log.lines
        assert all(need for speed, need in held if speed != 0.0)

    def test_releasing_rb_while_driving_stops_the_run(self, md, monkeypatch, tmp_path):
        d, held = self._driver(md, monkeypatch, tmp_path, release_after_driving=False)

        def hold(seconds, speed, steer, need_rb):
            if speed != 0.0:
                d.rb = False
            return d.rb or not need_rb

        d._hold = hold
        assert d.drive('circle_right') is None


class TestCircleRuns:
    @pytest.mark.parametrize('run,lock', [('circle_right', -1.0), ('circle_left', 1.0)])
    def test_straight_lead_in_then_full_lock(self, md, run, lock):
        speeds_steers = [(sp, st) for sp, st, _ in md.RUNS[run]]
        assert speeds_steers == [(0.0, 0.0), (0.25, 0.0), (0.25, lock), (0.0, 0.0)]

    def test_lock_to_lock_needs_both_sides(self, md):
        assert md.lock_to_lock({'servo_center_to_lock_s_right': 0.06}) == {}
        both = {'servo_center_to_lock_s_right': 0.06, 'servo_center_to_lock_s_left': 0.07}
        assert md.lock_to_lock(both) == {'servo_lock_to_lock_s': 0.13}

    def test_calibrate_lists_the_circle_keys(self, md):
        cal = load_script('calibrate')
        (step,) = [s for s in cal.STEPS if s.key == 'dynamics']
        for side in ('right', 'left'):
            assert f'servo_center_to_lock_s_{side}' in step.keys
        assert 'servo_lock_to_lock_s' in step.keys


class TestPauseBetweenMoves:
    def test_rb_is_held_through_the_pause_and_released_after_the_last_move(
        self, md, monkeypatch, tmp_path
    ):
        gate = TestDriveGate()
        d, held = gate._driver(md, monkeypatch, tmp_path, release_after_driving=False)
        plan = [(0, 0, 2), (0.25, 0, 2), (0, 0, 2), (-0.25, 0, 2), (0, 0, 3)]
        assert d.drive('yaw', plan, prefix='lidar') is not None
        needs = [need for _, need in held][-5:]
        assert needs == [False, True, True, True, False]
        assert d.log.lines.count('You can release RB now.') == 1
