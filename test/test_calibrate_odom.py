"""Unit tests for calibrate_odom.py: spin fit, scale limits, criteria, local YAML merge."""

import math

from conftest import load_script
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


class TestFitSpin:
    def test_exact_turns(self, odom):
        fit = odom.fit_spin(odom.integrate_rate(_spin(3.0)))
        assert (fit.turns, fit.scale) == (3, pytest.approx(1.0, rel=1e-3))

    def test_gyro_reading_four_percent_high(self, odom):
        fit = odom.fit_spin(odom.integrate_rate(_spin(3.0, scale=1.04)))
        assert fit.turns == 3
        assert fit.scale == pytest.approx(1 / 1.04, rel=1e-3)

    def test_turning_right_counts(self, odom):
        fit = odom.fit_spin(-odom.integrate_rate(_spin(2.0)))
        assert fit.turns == 2 and fit.measured_deg < 0

    def test_half_turn_is_refused(self, odom):
        assert odom.fit_spin(math.radians(540)) is None

    def test_no_turn_is_refused(self, odom):
        assert odom.fit_spin(math.radians(20)) is None


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
        assert odom.spin_criterion(measured, 2).status == status

    @pytest.mark.parametrize('measured,status', [(3.03, 'PASS'), (3.12, 'WARN'), (3.3, 'FAIL')])
    def test_tape(self, odom, measured, status):
        assert odom.tape_criterion(measured, 3.0).status == status


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
        assert 'circle (fully right or fully left) 2 times.' in odom.drive_circles(2)[0]

    def test_one_sentence_per_line(self):
        odom = load_script('calibrate_odom')
        for line in odom.drive_circles(3) + [odom.WHEELS_ON_TAPE]:
            assert line.count('. ') == 0


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
