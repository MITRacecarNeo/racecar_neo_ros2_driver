"""Unit tests for throttle math helpers."""

import pytest
from racecar_neo_ros2_driver.throttle_node import scale_speed, scale_steering


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
