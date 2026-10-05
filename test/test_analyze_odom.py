"""Unit tests for scripts/analyze_odom.py: run windows, criteria, bag reading."""

import math

from conftest import load_script
import numpy as np
import pytest

DT = 0.02  # /odom/integrated at 50 Hz


@pytest.fixture(scope='module')
def odom():
    return load_script('analyze_odom')


def _arc(odom, radius, laps, v=0.5, still=1.0, over_read=0.0, steer=None):
    """Build a run that sits still, drives `laps` around `radius` at speed v, then sits still."""
    t, x, y, yaw, vs, ws = [], [], [], [], [], []
    omega = v / radius if radius else 0.0
    moving = (laps * 2 * math.pi / omega) if omega else laps
    n_still, n_move = round(still / DT), round(moving / DT)
    theta = px = py = 0.0
    for i in range(n_still + n_move + n_still):
        is_moving = n_still <= i < n_still + n_move
        if is_moving:
            mid = theta + 0.5 * omega * DT
            px += v * math.cos(mid) * DT
            py += v * math.sin(mid) * DT
            theta += omega * DT
        t.append(i * DT)
        x.append(px)
        y.append(py)
        yaw.append(math.atan2(math.sin(theta), math.cos(theta)))  # wrapped, as in the quaternion
        vs.append(v * (1 + over_read) if is_moving else 0.0)
        ws.append(omega if is_moving else 0.0)
    steer_t = t if steer is not None else None
    steer_v = [steer] * len(t) if steer is not None else None
    return odom.run_from_arrays(t, x, y, yaw, vs, ws, steer_t, steer_v)


def _status(criteria, name_start):
    return next(c.status for c in criteria if c.name.startswith(name_start))


class TestWindow:
    def test_trims_the_still_ends(self, odom):
        run = _arc(odom, 1.0, 0.25, still=2.0).window()
        assert run.v.min() > 0
        assert run.t[0] == pytest.approx(2.0, abs=DT)

    def test_never_moving_is_an_error(self, odom):
        run = odom.run_from_arrays([0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0])
        with pytest.raises(ValueError, match='never turned'):
            run.window()

    def test_yaw_is_unwrapped(self, odom):
        run = _arc(odom, 1.0, 1.5)
        assert math.degrees(run.theta[-1] - run.theta[0]) == pytest.approx(540.0, abs=1.0)


class TestCircle:
    def test_one_lap_passes(self, odom):
        crit = odom.circle_criteria(_arc(odom, 0.8, 1.0))
        assert _status(crit, 'heading closure') == 'PASS'

    def test_seven_degrees_over_fails(self, odom):
        crit = odom.circle_criteria(_arc(odom, 0.8, 1.0 + 7.0 / 360.0))
        assert _status(crit, 'heading closure') == 'FAIL'

    def test_right_turns_count_too(self, odom):
        run = _arc(odom, 0.8, 1.0)
        run.theta = -run.theta
        assert _status(odom.circle_criteria(run), 'heading closure') == 'PASS'


class TestStraight:
    def _line(self, odom, length, v=0.5):
        t = np.arange(0, abs(length) / abs(v) + 2.0, DT)
        x = np.clip((t - 1.0) * v, min(0, length), max(0, length))
        moving = (t > 1.0) & (np.abs(x) < abs(length))
        vs = np.where(moving, v, 0.0)
        z = np.zeros_like(t)
        return odom.run_from_arrays(
            t.tolist(), x.tolist(), z.tolist(), z.tolist(), vs.tolist(), z.tolist()
        )

    def test_within_five_percent_passes(self, odom):
        crit = odom.straight_criteria(self._line(odom, 3.09), 3.0)
        assert _status(crit, 'distance, forward') == 'PASS'

    def test_ten_percent_long_fails(self, odom):
        crit = odom.straight_criteria(self._line(odom, 3.3), 3.0)
        assert _status(crit, 'distance, forward') == 'FAIL'

    def test_reverse_is_labelled(self, odom):
        crit = odom.straight_criteria(self._line(odom, -3.0, v=-0.5), 3.0)
        assert _status(crit, 'distance, reverse') == 'PASS'


class TestTurn:
    def test_exact_encoder_passes(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 2.0, steer=1.0), 0.75)
        assert _status(crit, 'turning bias, left') == 'PASS'
        steer = next(c for c in crit if c.name == 'steering command')
        assert steer.measured == '+1.000'

    def test_six_percent_over_read_warns(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 2.0, over_read=0.06), 0.75)
        bias = next(c for c in crit if c.name.startswith('turning bias'))
        assert bias.status == 'WARN'
        assert bias.measured.startswith('+6.0%')

    def test_curvature_matches_the_radius(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 1.0), 0.75)
        curve = next(c for c in crit if c.name == 'curvature')
        assert 'radius 0.750 m' in curve.measured


class TestCpu:
    def test_criteria(self, odom):
        assert odom.cpu_criteria(60.3, 52.5)[0].status == 'PASS'
        assert odom.cpu_criteria(67.3, 52.5)[0].status == 'FAIL'
        assert odom.cpu_criteria(60.3, None)[0].status == 'INFO'

    def test_own_process_cpu_reads(self, odom):
        import os

        assert odom.cpu_seconds(os.getpid()) > 0


class TestReadBag:
    def test_round_trip_through_an_mcap_bag(self, odom, tmp_path):
        rosbag2_py = pytest.importorskip('rosbag2_py')
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Odometry
        from rclpy.serialization import serialize_message

        bag = tmp_path / 'run'
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        for topic_id, (name, typ) in enumerate(
            (
                ('/odom/integrated', 'nav_msgs/msg/Odometry'),
                ('/motor', 'ackermann_msgs/msg/AckermannDriveStamped'),
            )
        ):
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=topic_id, name=name, type=typ, serialization_format='cdr'
                )
            )
        for i in range(50):
            o = Odometry()
            o.header.stamp.sec, o.header.stamp.nanosec = 100 + i // 50, (i % 50) * 20_000_000
            o.pose.pose.position.x = 0.01 * i
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(0.1), math.cos(0.1)
            o.twist.twist.linear.x = 0.5
            writer.write('/odom/integrated', serialize_message(o), 1_000_000 * i)
            m = AckermannDriveStamped()
            m.header.stamp = o.header.stamp
            m.drive.steering_angle = 0.25
            writer.write('/motor', serialize_message(m), 1_000_000 * i)
        del writer

        run = odom.read_bag(bag)
        assert len(run.t) == 50
        assert run.x[-1] == pytest.approx(0.49)
        assert run.theta[0] == pytest.approx(0.2)
        assert run.steer.tolist() == [0.25] * 50

    def test_bag_without_odometry_is_explained(self, odom, tmp_path):
        rosbag2_py = pytest.importorskip('rosbag2_py')
        bag = tmp_path / 'empty'
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0, name='/scan', type='sensor_msgs/msg/LaserScan', serialization_format='cdr'
            )
        )
        del writer
        with pytest.raises(ValueError, match='/odom/integrated'):
            odom.read_bag(bag)
