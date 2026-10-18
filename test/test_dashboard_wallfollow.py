"""
Scan-geometry tests for wallfollow.py.

Runs without a ROS graph or a lidar: LaserScan messages are built by hand at
the RACECAR Neo's pattern (1080 points over 360 degrees) and at the neoracer's
270 degree pattern, so the angle mapping is exercised on both.

    source /opt/ros/jazzy/setup.bash
    source /home/racecar/ros2_ws/install/setup.bash
    pytest -q
"""

import importlib.util
import math
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import LaserScan

BASE = Path(__file__).resolve().parents[1] / 'scripts' / 'dashboards' / 'wallfollow_dashboard'
_spec = importlib.util.spec_from_file_location('wallfollow', BASE / 'wallfollow.py')
wf = importlib.util.module_from_spec(_spec)
sys.modules['wallfollow'] = wf
_spec.loader.exec_module(wf)

N = wf.WallFollowNode
NEO_POINTS = 1080


def make_scan(points=NEO_POINTS, span_deg=360.0, fill=5.0):
    """Build a LaserScan with `points` rays evenly covering `span_deg`, centered on raw 0."""
    msg = LaserScan()
    msg.angle_min = math.radians(-span_deg / 2)
    msg.angle_max = math.radians(span_deg / 2)
    msg.angle_increment = (msg.angle_max - msg.angle_min) / (points - 1)
    msg.range_min = 0.05
    msg.range_max = 12.0
    msg.ranges = [fill] * points
    return msg


def place(msg, raw_deg, value):
    """Put a return of `value` at raw angle `raw_deg`."""
    i = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment)
    msg.ranges[i % len(msg.ranges)] = value
    return i % len(msg.ranges)


def place_wedge(msg, raw_deg, half_deg, value):
    """
    Fill every ray within `half_deg` of `raw_deg` with `value`.

    A windowed mean there sees only `value`. Returns the center index.
    """
    n = len(msg.ranges)
    centre = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment) % n
    half = round(math.radians(half_deg) / msg.angle_increment)
    for k in range(-half, half + 1):
        msg.ranges[(centre + k) % n] = value
    return centre


class TestScanPattern:
    def test_neo_scan_is_1080_points_over_360_degrees(self):
        msg = make_scan()
        assert len(msg.ranges) == 1080
        span = math.degrees(msg.angle_max - msg.angle_min)
        assert span == pytest.approx(360.0)
        assert math.degrees(msg.angle_increment) == pytest.approx(0.3336, abs=1e-3)

    def test_mount_yaw_puts_the_nose_at_the_raw_180_edge(self):
        # Measured on hardware: an object in front reports at raw 180. The
        # neoracer read 0 there, which is the whole of the difference.
        assert wf.LIDAR_MOUNT_YAW_DEG == 180.0


class TestAngleMapping:
    @pytest.mark.parametrize('student', [0, 15, 45, 90, 135, 180, -15, -45, -90, -135])
    def test_index_and_angle_round_trip(self, student):
        msg = make_scan()
        back = N._student_deg(msg, N._index_at(msg, student))
        err = (back - student + 180) % 360 - 180
        assert abs(err) <= math.degrees(msg.angle_increment)

    def test_object_in_front_reads_as_zero_degrees(self):
        # The hardware case the constant exists for: raw 180 is the nose.
        msg = make_scan()
        i = place_wedge(msg, 180.0, 5.0, 1.0)
        assert N._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)
        assert N._mean_at(N, msg, 0, 4.0) == pytest.approx(1.0, abs=0.01)

    def test_object_off_the_right_reads_as_positive(self):
        # Measured on hardware: an object off the car's right sits at raw +90.
        msg = make_scan()
        i = place_wedge(msg, 90.0, 5.0, 2.0)
        assert N._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)
        assert N._mean_at(N, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)

    def test_left_and_right_are_not_mirrored(self):
        # A handedness flip would swap these two and steer the car into the
        # wall it is trying to hold away from.
        msg = make_scan()
        place_wedge(msg, 90.0, 5.0, 2.0)    # car's right
        place_wedge(msg, -90.0, 5.0, 3.0)   # car's left
        assert N._mean_at(N, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)
        assert N._mean_at(N, msg, -90, 4.0) == pytest.approx(3.0, abs=0.01)

    def test_mapping_is_a_yaw_not_a_flip(self):
        # student_old = -raw; student_new = 180 - raw. The two differ by a
        # constant 180, so ordering around the circle is preserved.
        msg = make_scan()
        for raw in (-150, -90, -30, 0, 30, 90, 150):
            i = place(msg, raw, 1.0)
            old = -raw
            new = N._student_deg(msg, i)
            assert ((new - old) % 360) == pytest.approx(180.0, abs=0.5)


class TestOtherScanPatterns:
    def test_270_degree_scan_still_maps(self):
        # Nothing here assumes a sweep width; indices come from the message.
        msg = make_scan(points=811, span_deg=270.0)
        i = place_wedge(msg, 90.0, 5.0, 2.0)
        assert N._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)
        assert N._mean_at(N, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)

    def test_forward_facing_lidar_needs_no_offset(self, monkeypatch):
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', 0.0)
        msg = make_scan()
        i = place(msg, 0.0, 1.5)
        assert N._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


class TestWindowAveraging:
    def test_window_averages_over_its_cone(self):
        msg = make_scan(fill=4.0)
        for raw in (178, 179, 180, -179, -178):
            place(msg, raw, 1.0)
        # A 10 degree window at the nose spans about 30 rays, only 5 of which
        # carry the near return, so the mean sits between the two values.
        got = N._mean_at(N, msg, 0, 10.0)
        assert 1.0 < got < 4.0

    def test_no_return_reads_as_open_space_not_zero(self):
        msg = make_scan(fill=float('inf'))
        assert N._mean_at(N, msg, 0, 5.0) == msg.range_max
