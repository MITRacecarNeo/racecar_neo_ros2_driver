"""Unit tests for calibrate_steering.py: arc geometry, trim fit, criteria, a simulated run."""

import math

from conftest import load_script
import numpy as np
import pytest


@pytest.fixture(scope='module')
def st():
    return load_script('calibrate_steering')


def _arc(st, command, kappa):
    sm = load_script('scan_match')
    return st.Arc(command, kappa, 0.0, 1.0, sm.MatchResult((0, 0, 0), 0.01, 0.9, 5))


class TestArcGeometry:
    @pytest.mark.parametrize('kappa,heading_deg', [(1.2, 100.0), (-2.0, -250.0), (0.05, 6.0)])
    def test_curvature_from_chord_and_heading(self, st, kappa, heading_deg):
        th = math.radians(heading_deg)
        chord = abs(2 * math.sin(th / 2) / kappa)
        assert st.arc_curvature(th, chord) == pytest.approx(kappa)

    def test_straight_line_has_no_curvature(self, st):
        assert st.arc_curvature(0.0, 2.0) == 0.0


class TestTrimFit:
    def test_center_drift_and_slope_give_the_trim(self, st):
        # kappa = 0.09 + 3.6 * command near center: straight at -0.025.
        arcs = [_arc(st, c, 0.09 + 3.6 * c) for c in (0.0, 0.25, -0.25)]
        fit = st.fit_trim(arcs + [_arc(st, 0.5, 1.85), _arc(st, -0.5, -2.05)])
        assert fit.trim_delta == pytest.approx(-0.025)
        assert fit.drift_deg_per_m == pytest.approx(math.degrees(0.09))
        assert fit.gain_left == pytest.approx(3.6) and fit.gain_right == pytest.approx(3.6)

    def test_saturated_commands_do_not_move_the_trim(self, st):
        # The reference car: asymmetric at +/-0.5, nearly straight at 0.
        arcs = [_arc(st, 0.0, -0.016), _arc(st, 0.25, 0.9), _arc(st, -0.25, -0.9)]
        arcs += [_arc(st, 0.5, 1.845), _arc(st, -0.5, -2.05)]
        assert st.fit_trim(arcs).trim_delta == pytest.approx(0.016 / 3.6, rel=0.01)

    def test_one_side_is_enough(self, st):
        fit = st.fit_trim([_arc(st, 0.0, 0.036), _arc(st, -0.25, -0.864)])
        assert fit.trim_delta == pytest.approx(-0.01)

    def test_needs_the_straight_run(self, st):
        assert st.fit_trim([_arc(st, 0.25, 0.9), _arc(st, -0.25, -0.9)]) is None


class TestRuns:
    @pytest.mark.parametrize('command', [0.0, 0.25, -1.0])
    def test_steering_never_changes_within_a_run(self, st, command):
        assert {steer for _, steer, _ in st.segments(command)} == {command}
        assert st.segments(command)[0][0] == 0.0 and st.segments(command)[-1][0] == 0.0

    def test_curved_runs_turn_under_about_100_degrees(self, st):
        # Measured curvature on the reference car at 0.72 m/s.
        for command, kappa in ((0.25, 0.9), (0.5, 2.05), (1.0, 2.4)):
            drive = st.segments(command)[1][2]
            assert math.degrees(kappa * 0.72 * drive) < 110

    def test_names(self, st):
        assert [st.run_name(c) for c in (0.0, 0.25, -0.5, 1.0)] == ['c0', 'l025', 'r05', 'l10']


class TestCriteria:
    @pytest.mark.parametrize(
        'drift,status', [(0.4, 'PASS'), (-2.7, 'WARN'), (-4.0, 'WARN'), (5.2, 'FAIL')]
    )
    def test_drift(self, st, drift, status):
        assert st.drift_criterion(drift).status == status

    def test_warning_names_hardware_play(self, st):
        assert 'servo and linkage play' in st.drift_criterion(-2.7).advice
        assert 'check the servo horn' in st.drift_criterion(6.0).advice

    def test_trim_not_loaded_fails(self, st):
        assert st.trim_criterion(-0.045, 0.0).status == 'FAIL'
        assert st.trim_criterion(-0.045, -0.045).status == 'INFO'


class TestMeasureArc:
    def test_simulated_run(self, st):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        md = load_script('calibrate_dynamics')
        laser = (0.2383, 0.0, math.pi)
        kappa, turn = 1.2, math.radians(100.0)
        world0 = (-0.5, -0.3, 0.2)
        rel = (math.sin(turn) / kappa, (1 - math.cos(turn)) / kappa, turn)
        world1 = sm.compose(world0, rel)

        def scans(world, t0):
            pts = _points(sm, _scan(sm.compose(world, laser), noise=0.004, seed=int(t0)))
            return [(t0, pts, np.zeros(len(pts)))]

        t = np.linspace(0.0, 10.0, 501)
        frac = np.clip((t - 3.0) / 3.0, 0.0, 1.0)
        # Odometry reads the turn 3% high, as a gyro scale error would.
        odom = np.column_stack(
            (
                np.sin(frac * turn) / kappa,
                (1 - np.cos(frac * turn)) / kappa,
                frac * turn * 1.03,
            )
        )
        speed = np.where((t > 3.0) & (t < 6.0), 0.7, 0.0)
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=speed,
            path_m=np.cumsum(speed) * 0.02,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 6.0]),
            motor=np.array([[0.0, 0.5], [0.25, 0.5], [0.0, 0.5]]),
            scans=scans(world0, 1.5) + scans(world1, 8.3),
        )
        arc = st.measure_arc(bag, laser, 0.5)
        assert arc.curvature == pytest.approx(kappa, rel=0.005)
        assert arc.heading_deg == pytest.approx(100.0, abs=0.1)
        assert arc.command == 0.5


class TestArcProblem:
    def _arc(self, st, inliers, rms, lidar_deg, odom_deg, chord=1.5, odom_chord=1.52):
        sm = load_script('scan_match')
        match = sm.MatchResult((0, 0, 0), rms, inliers, 5)
        return st.Arc(0.25, 0.9, lidar_deg, chord, match, odom_deg, odom_chord)

    def test_a_long_run_with_43_percent_matched_is_trusted(self, st):
        assert st.arc_problem(self._arc(st, 0.43, 0.0274, 82.0, 80.5)) == ''

    def test_disagreement_with_the_odometry_is_refused(self, st):
        assert 'odometry' in st.arc_problem(self._arc(st, 0.6, 0.015, 82.0, 120.0))

    def test_too_few_points_is_refused(self, st):
        assert 'too poor' in st.arc_problem(self._arc(st, 0.2, 0.015, 82.0, 82.0))

    def test_wrong_chord_is_refused(self, st):
        # The 16:25 run: heading agreed, chord a third of the drive.
        arc = self._arc(st, 0.32, 0.0257, -128.2, -124.7, chord=0.477, odom_chord=1.40)
        assert 'chord' in st.arc_problem(arc)
