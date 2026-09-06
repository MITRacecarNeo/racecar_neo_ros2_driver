"""Unit tests for scan_rotate_node's pure helper."""

import math

import pytest
from racecar_neo_ros2_driver.scan_rotate_node import rotate_scan
from sensor_msgs.msg import LaserScan

# The reference car: 1080 samples spanning -180 to +180 inclusive, so the
# increment is 360/1079 degrees, not 360/1080.
SAMPLES = 1080
INCREMENT = math.radians(360.0 / (SAMPLES - 1))


def _scan(ranges=None, intensities=None):
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = INCREMENT
    msg.range_min = 0.05
    msg.range_max = 12.0
    msg.ranges = list(ranges if ranges is not None else [1.0] * SAMPLES)
    msg.intensities = list(intensities or [])
    return msg


def _mean_at(msg, deg):
    """Apply the dashboards' index arithmetic: 0 = nose, + = right."""
    n = len(msg.ranges)
    center = round((math.radians(-deg) - msg.angle_min) / msg.angle_increment)
    return msg.ranges[center % n]


def _raw_with_object_at(scan_angle_deg, value=0.6, half_width=3):
    """
    Place a return at a known angle in the raw, uncorrected scan.

    Spread over a few rays: the fixture and the reader round independently, so
    a single ray can land one index off. The real _mean_at averages over a
    window and never sees that.
    """
    ranges = [9.0] * SAMPLES
    idx = round((math.radians(scan_angle_deg) + math.pi) / INCREMENT)
    for k in range(-half_width, half_width + 1):
        ranges[(idx + k) % SAMPLES] = value
    return _scan(ranges)


class TestRotateScan:
    def test_window_moves_by_the_rotation(self):
        out = rotate_scan(_scan(), 180.0)
        assert out.angle_min == pytest.approx(0.0)
        assert out.angle_max == pytest.approx(2 * math.pi)

    def test_zero_rotation_leaves_the_window(self):
        msg = _scan()
        out = rotate_scan(msg, 0.0)
        assert out.angle_min == pytest.approx(msg.angle_min)
        assert out.angle_max == pytest.approx(msg.angle_max)

    def test_negative_rotation_moves_the_other_way(self):
        out = rotate_scan(_scan(), -90.0)
        assert out.angle_min == pytest.approx(-math.pi - math.pi / 2)

    def test_ranges_are_not_copied_or_reordered(self):
        # The whole point of rotating the header: consumers that index the
        # array directly, like racecar-neo-library, must see no change.
        msg = _scan([float(i) for i in range(SAMPLES)])
        out = rotate_scan(msg, 180.0)
        assert list(out.ranges) == list(msg.ranges)

    def test_intensities_are_untouched(self):
        msg = _scan(intensities=[float(i) for i in range(SAMPLES)])
        out = rotate_scan(msg, 180.0)
        assert list(out.intensities) == list(msg.intensities)

    def test_increment_and_limits_are_preserved(self):
        msg = _scan()
        out = rotate_scan(msg, 180.0)
        assert out.angle_increment == msg.angle_increment
        assert out.range_min == msg.range_min
        assert out.range_max == msg.range_max

    def test_empty_scan_does_not_raise(self):
        assert list(rotate_scan(_scan(ranges=[]), 180.0).ranges) == []

    def test_rotation_is_exact_not_quantized(self):
        # Rolling the array would snap to a whole sample; moving the window
        # does not, so an angle that is not a sample multiple stays exact.
        out = rotate_scan(_scan(), 1.0)
        assert out.angle_min == pytest.approx(-math.pi + math.radians(1.0))


class TestMountCorrection:
    """The geometry measured on the car, pinned so a regression is visible."""

    def test_front_object_reads_as_the_nose_after_rotation(self):
        # Measured: an object in front appears at raw scan 180 deg, which the
        # dashboards read as 180 (behind them) rather than 0.
        raw = _raw_with_object_at(180.0)
        assert _mean_at(raw, 0) == pytest.approx(9.0)
        fixed = rotate_scan(raw, 180.0)
        assert _mean_at(fixed, 0) == pytest.approx(0.6)

    def test_right_object_reads_as_right_after_rotation(self):
        # Measured: an object to the right appears at raw scan +90, which the
        # dashboards read as -90, their left.
        raw = _raw_with_object_at(90.0)
        assert _mean_at(raw, -90) == pytest.approx(0.6)
        fixed = rotate_scan(raw, 180.0)
        assert _mean_at(fixed, 90) == pytest.approx(0.6)

    def test_left_object_reads_as_left_after_rotation(self):
        raw = _raw_with_object_at(-90.0)
        fixed = rotate_scan(raw, 180.0)
        assert _mean_at(fixed, -90) == pytest.approx(0.6)

    def test_rear_object_reads_as_rear_after_rotation(self):
        raw = _raw_with_object_at(0.0)
        fixed = rotate_scan(raw, 180.0)
        assert _mean_at(fixed, 180) == pytest.approx(0.6)

    def test_library_indexing_is_unaffected(self):
        # racecar-neo-library flips the array and reads index 0 as the car's
        # front. That must keep working across the correction.
        raw = _raw_with_object_at(180.0)
        fixed = rotate_scan(raw, 180.0)
        assert list(reversed(list(raw.ranges)))[0] == pytest.approx(
            list(reversed(list(fixed.ranges)))[0])
