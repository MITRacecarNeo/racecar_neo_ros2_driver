"""Unit tests for calibrate_odom.py: turn resolution, scale limits, criteria, YAML merge."""

import math

from conftest import load_script
import numpy as np
import pytest
import yaml


@pytest.fixture(scope='module')
def odom():
    return load_script('calibrate_odom')


@pytest.fixture(scope='module')
def common():
    return load_script('calibrate_common')


QUARTER_TURN_PER_S = math.radians(90)


def _spin(turns, scale=1.0, bias=0.0, rate=QUARTER_TURN_PER_S, dt=0.006):
    """Gyro samples for `turns` at `rate` (true), read with gain `scale` plus `bias`."""
    n = round(turns * 2 * math.pi / rate / dt)
    return [(i * dt, rate * scale + bias) for i in range(n + 1)]


class TestIntegrate:
    def test_constant_rate(self, odom):
        assert odom.integrate_rate([(0.0, 1.0), (2.0, 1.0)]) == pytest.approx(2.0)

    def test_bias_removed(self, odom):
        samples = _spin(1.0, bias=0.01)
        assert odom.integrate_rate(samples, bias=0.01) == pytest.approx(2 * math.pi, rel=1e-3)

    def test_too_few_samples(self, odom):
        assert odom.integrate_rate([(0.0, 1.0)]) == 0.0


class TestResolveTurns:
    def test_lidar_heading_plus_the_gyro_turn_count(self, odom):
        lidar = math.radians(3.0)  # modulo one turn
        true = odom.resolve_turns(lidar, math.radians(3 * 360 + 8.0))
        assert true == pytest.approx(math.radians(3 * 360 + 3.0))

    def test_turning_right(self, odom):
        true = odom.resolve_turns(math.radians(-2.0), math.radians(-725.0))
        assert true == pytest.approx(math.radians(-722.0))

    def test_gyro_a_half_turn_off_is_refused(self, odom):
        assert odom.resolve_turns(0.0, math.radians(720 + 174)) is None

    def test_scale_from_the_resolved_heading(self, odom):
        gyro = odom.integrate_rate(_spin(3.0, scale=1.04))
        true = odom.resolve_turns(math.radians(0.4), gyro)
        assert true / gyro == pytest.approx(1 / 1.04, rel=1e-3)


class TestOdomMotion:
    def test_heading_unwraps_through_full_turns(self, odom):
        yaws = [math.atan2(math.sin(a), math.cos(a)) for a in np.linspace(0, 4 * math.pi, 400)]
        poses = [[i * 0.02, 0.0, 0.0, y, 0.5] for i, y in enumerate(yaws)]
        assert odom.odom_motion(poses)[2] == pytest.approx(4 * math.pi)

    def test_translation_in_the_start_frame(self, odom):
        poses = [[0.0, 1.0, 1.0, math.pi / 2, 0.0], [1.0, 1.0, 3.0, math.pi / 2, 0.0]]
        x, y, th = odom.odom_motion(poses)
        assert (x, y, th) == (pytest.approx(2.0), pytest.approx(0.0, abs=1e-12), 0.0)


class TestScaleLimits:
    def test_degrees_for_radians_is_refused(self, odom):
        # A deg/s-vs-rad/s mix-up reads 57x; the fitted scale is far outside the band.
        assert not odom.scale_ok(1 / 57.3)
        assert not odom.scale_ok(57.3)

    def test_ordinary_corrections_pass(self, odom):
        assert odom.scale_ok(0.96) and odom.scale_ok(1.08)


class TestCriteria:
    @pytest.mark.parametrize(
        'measured,status', [(721.0, 'PASS'), (735.0, 'WARN'), (760.0, 'FAIL'), (-722.0, 'PASS')]
    )
    def test_spin(self, odom, measured, status):
        lidar = 720.0 if measured > 0 else -720.0
        assert odom.spin_criterion(measured, lidar).status == status

    def test_spin_a_quarter_too_high_fails(self, odom):
        assert odom.spin_criterion(-893.9, -720.0).status == 'FAIL'

    @pytest.mark.parametrize('measured,status', [(3.03, 'PASS'), (3.12, 'WARN'), (3.3, 'FAIL')])
    def test_drive(self, odom, measured, status):
        assert odom.drive_criterion(measured, 3.0).status == status

    def test_poor_scan_match_fails(self, odom):
        sm = load_script('scan_match')
        good = sm.MatchResult((0.0, 0.0, 0.0), 0.01, 0.9, 10)
        moved = sm.MatchResult((0.0, 0.0, 0.0), 0.08, 0.3, 10)
        assert odom.match_criterion('m', good).status == 'PASS'
        assert odom.match_criterion('m', moved).status == 'FAIL'


class TestMergeWrite:
    def test_merge_keeps_the_other_step(self, common, tmp_path):
        common.write_local_yaml(
            'odom', 'pit_node', {'odometry.yaw_rate_scale': 0.98}, [tmp_path], 't', merge=True
        )
        (path,) = common.write_local_yaml(
            'odom', 'pit_node', {'odometry.speed_scale': 1.05}, [tmp_path], 't', merge=True
        )
        params = yaml.safe_load(path.read_text())['pit_node']['ros__parameters']
        assert params == {'odometry.yaw_rate_scale': 0.98, 'odometry.speed_scale': 1.05}

    def test_without_merge_the_file_is_replaced(self, common, tmp_path):
        common.write_local_yaml('x', 'n', {'a': 1}, [tmp_path], 't')
        (path,) = common.write_local_yaml('x', 'n', {'b': 2}, [tmp_path], 't')
        assert yaml.safe_load(path.read_text())['n']['ros__parameters'] == {'b': 2}

    def test_keys_match_what_pit_node_declares(self):
        from pathlib import Path

        src = (
            Path(__file__).resolve().parent.parent / 'racecar_neo_ros2_driver' / 'pit_node.py'
        ).read_text()
        for key in ('odometry.yaw_rate_scale', 'odometry.speed_scale'):
            assert f"declare_parameter('{key}'" in src


class TestInstructions:
    def test_circle_count_is_in_the_text(self):
        odom = load_script('calibrate_odom')
        assert 'circle (fully right or fully left) 2 times.' in odom.spin_lines(2)[0]

    def test_one_sentence_per_line(self):
        odom = load_script('calibrate_odom')
        for line in odom.spin_lines(3) + odom.DRIVE_LINES + [odom.SPIN_PLACE, odom.DRIVE_PLACE]:
            assert line.count('. ') <= 1
            assert '; ' not in line


class TestScalesInUse:
    def test_matching_scales_are_info(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.00462, 0.90338), (1.00462, 0.90338))
        assert c.status == 'INFO'

    def test_teleop_not_restarted_fails(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.00462, 0.90338), (1.0, 1.0))
        assert c.status == 'FAIL' and 'restart teleop' in c.advice

    def test_unreachable_node_is_reported_not_judged(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.0, 1.0), None)
        assert c.status == 'INFO' and 'not reachable' in c.limit


class TestBestMatch:
    def test_recovers_from_a_gyro_guess_35_degrees_off(self, odom):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        ref = _points(sm, _scan((0.0, 0.0, 0.0), noise=0.005, seed=1))
        cur = _points(sm, _scan((0.3, -0.2, math.radians(3.0)), noise=0.005, seed=2))
        result = odom.best_match(ref, cur, [(0.0, 0.0, math.radians(38.0))])
        assert math.degrees(result.pose[2]) == pytest.approx(3.0, abs=0.1)
        assert result.pose[:2] == (pytest.approx(0.3, abs=0.005), pytest.approx(-0.2, abs=0.005))
