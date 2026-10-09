"""Unit tests for throttle math helpers."""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import pytest
from racecar_neo_ros2_driver.throttle_node import (
    cap_speed,
    scale_speed,
    scale_steering,
    status_mode,
)


class TestScaleSpeed:
    def test_zero_passes_through(self):
        assert scale_speed(0.0, 0.5, 0.6) == 0.0

    def test_full_forward(self):
        assert scale_speed(1.0, 0.5, 0.6) == pytest.approx(0.5)

    def test_full_reverse(self):
        assert scale_speed(-1.0, 0.5, 0.6) == pytest.approx(-0.6)

    def test_half_forward(self):
        assert scale_speed(0.5, 0.5, 0.6) == pytest.approx(0.25)

    def test_clamp_above_one(self):
        assert scale_speed(2.5, 0.5, 0.6) == pytest.approx(0.5)

    def test_clamp_below_neg_one(self):
        assert scale_speed(-2.5, 0.5, 0.6) == pytest.approx(-0.6)

    def test_asymmetric_caps(self):
        # Reverse cap should apply only when speed < 0
        assert scale_speed(1.0, 0.17, 0.24) == pytest.approx(0.17)
        assert scale_speed(-1.0, 0.17, 0.24) == pytest.approx(-0.24)


class TestScaleSteering:
    def test_zero_passes_through(self):
        assert scale_steering(0.0, 0.625) == 0.0

    def test_full_left(self):
        assert scale_steering(1.0, 0.625) == pytest.approx(0.625)

    def test_full_right(self):
        assert scale_steering(-1.0, 0.625) == pytest.approx(-0.625)

    def test_clamp_above_one(self):
        assert scale_steering(5.0, 0.625) == pytest.approx(0.625)

    def test_clamp_below_neg_one(self):
        assert scale_steering(-5.0, 0.625) == pytest.approx(-0.625)


class TestSteeringTrim:
    def test_trim_shifts_center(self):
        assert scale_steering(0.0, 1.0, trim=-0.05) == pytest.approx(-0.05)

    def test_trim_is_added_after_the_cap(self):
        assert scale_steering(1.0, 0.5, trim=0.05) == pytest.approx(0.55)

    def test_trimmed_output_stays_in_range(self):
        assert scale_steering(1.0, 1.0, trim=0.05) == 1.0
        assert scale_steering(-1.0, 1.0, trim=-0.05) == -1.0


class TestMappingSpeedCap:
    @pytest.mark.parametrize(
        'speed,expected', [(1.0, 0.02), (-0.4, -0.02), (0.015, 0.015), (0.005, 0.005), (0.0, 0.0)]
    )
    def test_clips_and_keeps_the_deadband(self, speed, expected):
        assert cap_speed(speed, 0.02) == pytest.approx(expected)

    def test_mode_from_status(self):
        msg = DiagnosticStatus(
            values=[KeyValue(key='map', value='room'), KeyValue(key='mode', value='mapping')]
        )
        assert status_mode(msg) == 'mapping'
        assert status_mode(DiagnosticStatus()) == ''
