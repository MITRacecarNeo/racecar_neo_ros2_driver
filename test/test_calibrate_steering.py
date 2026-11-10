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
        # 0.625 m/s at 0.25 after half the 0.62 s rise; curvature 2026-11-09.
        for command, kappa in ((0.25, 0.9), (0.5, 2.2), (1.0, 2.55)):
            drive = st.segments(command)[1][2]
            assert math.degrees(kappa * 0.625 * (drive - 0.31)) < 100

    def test_names(self, st):
        assert [st.run_name(c) for c in (0.0, 0.25, -0.5, 1.0)] == ['c0', 'l025', 'r05', 'l10']


class TestCriteria:
    @pytest.mark.parametrize(
        'drift,status', [(0.4, 'PASS'), (-2.7, 'WARN'), (-4.0, 'WARN'), (5.2, 'FAIL')]
    )
    def test_drift(self, st, drift, status):
        assert st.drift_criterion(drift).status == status

    def test_warning_advice(self, st):
        assert st.drift_criterion(-2.7).advice == (
            'Run the check again. If the drift stays, run racecar calibrate steering'
        )
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
    def _arc(self, st, inliers, rms, lidar_deg, odom_deg, chord=1.5, odom_chord=1.52, speed=0.25):
        sm = load_script('scan_match')
        match = sm.MatchResult((0, 0, 0), rms, inliers, 5)
        return st.Arc(0.25, 0.9, lidar_deg, chord, match, odom_deg, odom_chord, speed=speed)

    def test_wheel_slip_backing_up_is_accepted(self, st):
        # 2026-11-09, steering +0.75 backing up: heading agreed, wheels slipped.
        arc = self._arc(st, 0.93, 0.011, -29.6, -29.8, chord=0.24, odom_chord=0.46, speed=-0.25)
        assert st.arc_problem(arc) == ''
        assert 'chord' in st.arc_problem(
            self._arc(st, 0.93, 0.011, -29.6, -29.8, chord=0.24, odom_chord=0.46)
        )

    def test_reverse_chord_past_the_slip_floor_is_refused(self, st):
        arc = self._arc(st, 0.9, 0.012, -30.0, -30.0, chord=0.15, odom_chord=0.46, speed=-0.25)
        assert 'chord' in st.arc_problem(arc)

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


class TestTurnGain:
    def _arc(self, st, command, kappa, enc, lidar):
        sm = load_script('scan_match')
        m = sm.MatchResult((0, 0, 0), 0.01, 0.9, 5)
        return st.Arc(command, kappa, 0.0, 1.0, m, 0.0, 1.0, enc, lidar)

    def _x(self, kappa, wheelbase=0.23):
        return 1 / math.cos(math.atan(wheelbase * abs(kappa))) - 1

    def test_recovers_half_with_a_short_straight_baseline(self, st):
        base = 0.983  # straight runs read 1.7% short this session
        arcs = [self._arc(st, 0.0, 0.0, base * 2.3, 2.3)]
        for c, k in ((0.5, 1.85), (-0.5, -2.06), (1.0, 2.37), (-1.0, -2.14)):
            arcs.append(self._arc(st, c, k, base * 4.0 * (1 + 0.5 * self._x(k)), 4.0))
        fit = st.fit_turn_gain(arcs, current_gain=0.0, wheelbase=0.23)
        assert fit.gain == pytest.approx(0.5, abs=1e-6)
        assert fit.straight_ratio == pytest.approx(base) and fit.runs == 4

    def test_a_correction_already_applied_is_put_back(self, st):
        # /odom/integrated already divides by the 0.4 factor; the fit still finds 0.5.
        arcs = [self._arc(st, 0.0, 0.0, 2.3, 2.3)]
        for c, k in ((0.5, 1.85), (1.0, 2.37)):
            x = self._x(k)
            arcs.append(self._arc(st, c, k, 4.0 * (1 + 0.5 * x) / (1 + 0.4 * x), 4.0))
        assert st.fit_turn_gain(arcs, 0.4, 0.23).gain == pytest.approx(0.5, abs=1e-6)

    def test_needs_the_straight_run_and_two_curved(self, st):
        arcs = [self._arc(st, 0.5, 1.85, 4.2, 4.0), self._arc(st, 1.0, 2.37, 4.3, 4.0)]
        assert st.fit_turn_gain(arcs, 0.0, 0.23) is None
        arcs.append(self._arc(st, 0.25, 0.9, 1.6, 1.6))  # carries too little steering
        arcs = arcs[:1] + [self._arc(st, 0.0, 0.0, 2.3, 2.3)]
        assert st.fit_turn_gain(arcs, 0.0, 0.23) is None


# Bill's dynamics, 2026-11-07.
DYNAMICS = {
    'steady_speed_mps_025': 0.625,
    'steady_speed_mps_rev025': -0.705,
    'rise_90_s_025': 0.62,
    'rise_90_s_rev025': 0.17,
}


class TestPairs:
    def test_every_command_pairs_forward_then_reverse_at_one_steering(self, st):
        segs = st.pair_segments(0.5, 0.25, DYNAMICS)
        assert [round(sp, 2) for sp, _, _ in segs] == [0.0, 0.25, 0.0, -0.25, 0.0]
        assert {steer for _, steer, _ in segs} == {0.5}

    def test_reverse_leg_covers_the_forward_distance(self, st):
        back = st.reverse_seconds(2.0, 0.25, DYNAMICS)
        forward_m = 0.625 * (2.0 - 0.62 / 2)
        assert 0.705 * (back - st.REVERSE_START_SEC) == pytest.approx(forward_m, abs=0.01)
        assert st.reverse_seconds(2.0, 0.25, {}) == 2.0

    def test_grid_rises_in_amplitude_with_both_sides(self, st):
        amps = [abs(c) for c in st.COMMANDS]
        assert amps == sorted(amps) and set(st.COMMANDS) == {-a for a in st.COMMANDS}
        assert len(st.COMMANDS) == 13

    def test_names(self, st):
        assert st.pair_name(-0.125, 0.5) == 'r0125_s05'

    def test_left_turns_first_then_right(self, st):
        order = [c for c, _ in st.plan_order(st.COMMANDS, 0.25)]
        assert order == [0.0, 0.125, 0.25, 0.375, 0.5, 0.75, 1.0,
                         -0.125, -0.25, -0.375, -0.5, -0.75, -1.0]  # fmt: skip
        assert [c for c, _ in st.plan_order(st.FAST_COMMANDS, 0.5)] == [0.5, 1.0, -0.5, -1.0]


class TestMeasurePair:
    def test_forward_and_back_along_one_arc(self, st):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        md = load_script('calibrate_dynamics')
        laser = (0.2383, 0.0, math.pi)
        kappa, turn = 1.2, math.radians(80.0)
        world0 = (-0.5, -0.3, 0.2)
        world1 = sm.compose(world0, (math.sin(turn) / kappa, (1 - math.cos(turn)) / kappa, turn))
        # Backing up with the same steering retraces three quarters of the arc.
        back = turn * 0.75
        world2 = sm.compose(
            world1, sm.invert((math.sin(back) / kappa, (1 - math.cos(back)) / kappa, back))
        )

        def scans(world, t0):
            pts = _points(sm, _scan(sm.compose(world, laser), noise=0.004, seed=int(t0 * 10)))
            return [(t0, pts, np.zeros(len(pts)))]

        t = np.linspace(0.0, 16.0, 801)
        f1 = np.clip((t - 3.0) / 3.0, 0.0, 1.0)
        f2 = np.clip((t - 9.0) / 3.0, 0.0, 1.0)
        odom = []
        for a, b in zip(f1, f2, strict=True):
            th = a * turn - b * back
            odom.append((math.sin(th) / kappa, (1 - math.cos(th)) / kappa, th))
        odom = np.array(odom)
        speed = np.where((t > 3.0) & (t < 6.0), 0.7, np.where((t > 9.0) & (t < 12.0), -0.7, 0.0))
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=speed,
            path_m=np.cumsum(np.abs(speed)) * 0.02,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 6.0, 9.0, 12.0]),
            motor=np.array([[0.0, 0.5], [0.25, 0.5], [0.0, 0.5], [-0.25, 0.5], [0.0, 0.5]]),
            scans=scans(world0, 1.5) + scans(world1, 8.0) + scans(world2, 14.5),
        )
        fwd, rev = st.measure_pair(bag, laser, 0.5)
        assert fwd.curvature == pytest.approx(kappa, rel=0.01)
        assert rev.curvature == pytest.approx(kappa, rel=0.01)  # per meter traveled: same sign
        assert rev.heading_deg == pytest.approx(-60.0, abs=0.5)
        offset = sm.compose(fwd.motion, rev.motion)
        assert math.degrees(offset[2]) == pytest.approx(20.0, abs=0.5)


class TestTable:
    FORWARD = {-1.0: -2.15, -0.5: -2.05, 0.0: 0.0, 0.5: 1.85, 1.0: 2.39}

    def test_written_in_the_new_trims_terms(self, st):
        table = st.build_table(self.FORWARD, None, 0.0, 0.02, 'today')
        assert table.command[2] == pytest.approx(-0.02) and table.trim == 0.02
        assert table.curvature_for(-0.02) == pytest.approx(0.0)
        assert table.curvature_reverse is None

    def test_reverse_kept_only_when_it_differs(self, st):
        close = {c: k * 1.05 for c, k in self.FORWARD.items()}
        far = {c: k * 1.2 for c, k in self.FORWARD.items()}
        assert not st.compare_lines('t', self.FORWARD, close, st.REVERSE_MATCH)[1]
        lines, off = st.compare_lines('t', self.FORWARD, far, st.REVERSE_MATCH)
        assert off and 'differs by more than 10%' in lines[-1]

    def test_near_zero_compared_absolutely(self, st):
        assert not st.differs(0.01, 0.03, 0.10)

    def test_check_against_the_stored_table(self, st):
        stored = st.build_table(self.FORWARD, None, 0.0, 0.0, 'today')

        def legs(k_fwd, k_rev):
            return (
                st.Arc(0.5, k_fwd, 0, 1, None),
                st.Arc(0.5, k_rev, 0, 1, None, speed=-0.25),
            )

        good = st.table_criteria(stored, {(0.5, 0.25): legs(1.86, 1.80)}, 0.0)
        assert [c.status for c in good] == ['PASS', 'PASS']
        bad = st.table_criteria(stored, {(0.5, 0.25): legs(1.40, 1.80)}, 0.0)
        assert bad[0].status == 'FAIL' and 'racecar calibrate steering' in bad[0].advice
        assert st.table_criteria(None, {}, 0.0)[0].status == 'FAIL'


class TestReverseScale:
    def test_learns_from_a_short_reverse_leg(self, st):
        fwd = st.Arc(0.125, 0.4, 27, 1.29, None, lidar_arc_m=1.30)
        rev = st.Arc(0.125, 0.48, -23, 0.82, None, speed=-0.25, lidar_arc_m=0.83)
        assert st.next_reverse_scale(1.0, fwd, rev) == pytest.approx(1.30 / 0.83, abs=0.001)
        assert st.next_reverse_scale(1.5, fwd, rev) == st.REVERSE_SCALE_LIMITS[1]

    def test_stretches_only_the_reverse_leg(self, st):
        plain = st.pair_segments(0.125, 0.25, DYNAMICS)
        longer = st.pair_segments(0.125, 0.25, DYNAMICS, 1.5)
        assert longer[1] == plain[1]
        assert longer[3][2] == pytest.approx(plain[3][2] * 1.5, abs=0.01)

    def test_pair_is_the_last_motion_in_the_bag(self, st):
        md = load_script('calibrate_dynamics')
        t = np.linspace(0.0, 30.0, 1501)
        # Driven into place with the gamepad first, then the pair.
        motor_t = np.array([0.0, 2.0, 4.0, 10.0, 13.0, 16.0, 18.0])
        motor = np.array(
            [[0, 0], [-1.0, 0.3], [0, 0], [0.25, 0.5], [0, 0.5], [-0.25, 0.5], [0, 0.5]], float
        )
        bag = md.Bag(
            odom_t=t,
            odom=np.zeros((len(t), 3)),
            speed=np.zeros_like(t),
            path_m=np.zeros_like(t),
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=motor_t,
            motor=motor,
            scans=[],
        )
        seen = []
        st_measure = st.measure_leg
        try:
            st.measure_leg = lambda b, laser, c, launch, sp, win: seen.append((launch, sp, win))
            st.measure_pair(bag, None, 0.5)
        finally:
            st.measure_leg = st_measure
        assert [(launch, sp) for launch, sp, _ in seen] == [(10.0, 0.25), (16.0, -0.25)]


class TestRunFlow:
    """Drive everything, analyze everything, then offer the failed runs again."""

    def _calibrator(self, st, monkeypatch, answers, fail_first):
        cal = st.SteeringCalibrator.__new__(st.SteeringCalibrator)
        logged = []
        cal.log = type('Log', (), {'info': logged.append, 'error': logged.append})()
        driven = []

        def drive_pairs(plan):
            driven.append(list(plan))
            return {key: f'bag-{key}' for key in plan}

        analyzed = []

        def analyze(bags):
            analyzed.append(list(bags))
            failed = [k for k in bags if k in fail_first and len(analyzed) == 1]
            return {k: ('fwd', 'rev') for k in bags if k not in failed}, failed

        cal.drive_pairs = drive_pairs
        cal.analyze = analyze
        replies = iter(answers)
        monkeypatch.setattr(st, 'read_line', lambda prompt='': next(replies))
        return cal, driven, analyzed, logged

    def test_failed_runs_driven_again_after_analysis(self, st, monkeypatch):
        plan = [(0.0, 0.25), (0.75, 0.25), (-1.0, 0.25)]
        cal, driven, analyzed, logged = self._calibrator(
            st, monkeypatch, ['', ''], fail_first={(0.75, 0.25)}
        )
        results = cal.run_pairs(plan)
        assert driven == [plan, [(0.75, 0.25)]]
        assert set(results) == set(plan)
        assert any(
            '1 runs need to be driven again: steering +0.75 at 0.25' in line for line in logged
        )

    def test_operator_can_finish_without_them(self, st, monkeypatch):
        plan = [(0.0, 0.25), (0.75, 0.25)]
        cal, driven, _, _ = self._calibrator(st, monkeypatch, ['', 's'], fail_first={(0.75, 0.25)})
        assert set(cal.run_pairs(plan)) == {(0.0, 0.25)}
        assert driven == [plan]


class TestFullLockPooling:
    # 2026-11-09 12:13, forward right side: full lock reached by -0.75.
    CMDS = [-1.0, -0.75, -0.5, -0.375, 0.0]

    def test_small_fall_at_full_lock_is_pooled(self, st):
        pooled, notes, problem = st.pool_flat(self.CMDS, [-2.255, -2.282, -2.193, -1.685, 0.0])
        assert problem == ''
        assert pooled[0] == pooled[1] == pytest.approx((-2.255 - 2.282) / 2, abs=1e-4)
        assert pooled[2:] == [-2.193, -1.685, 0.0]
        assert len(notes) == 1 and '-1 (-2.255)' in notes[0]

    def test_reverse_scatter_pools_three(self, st):
        pooled, notes, problem = st.pool_flat(self.CMDS, [-2.070, -2.236, -2.109, -1.528, 0.0])
        assert problem == '' and pooled[0] == pooled[1] == pytest.approx(-2.153, abs=1e-3)
        assert pooled[2] == -2.109
        assert all(b >= a for a, b in zip(pooled, pooled[1:], strict=False))

    def test_large_fall_is_refused(self, st):
        _, _, problem = st.pool_flat(self.CMDS, [-1.5, -2.282, -2.193, -1.685, 0.0])
        assert 'falls across' in problem


class TestTiming:
    def test_reverse_start_time_floor(self, st):
        # Forward 1.3 s at 0.25: 0.625 m/s after half the 0.62 s rise.
        back = st.reverse_seconds(1.3, 0.25, DYNAMICS)
        assert back == pytest.approx(0.625 * (1.3 - 0.31) / 0.705 + 0.48, abs=0.01)

    def test_bag_names_parse_back(self, st):
        for command in st.COMMANDS:
            for speed in (0.25, 0.5):
                name = f'20261109_121806_steer_{st.pair_name(command, speed)}'
                assert st.parse_pair_name(name) == (command, speed)
        assert st.parse_pair_name('20261109_121806_dyn_straight') is None
