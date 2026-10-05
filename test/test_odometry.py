"""Unit tests for odometry.py and its use in pit_node: integration, bias, config."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from racecar_neo_ros2_driver.odometry import OdomIntegrator, OdomState, yaw_quaternion
from racecar_neo_ros2_driver.pit_node import build_integrated_odom, PitNode
import yaml

ROOT = Path(__file__).resolve().parent.parent
DT = 0.006  # telemetry frame period, s


def _drive(integ, seconds, v, omega_raw, t0=0.0):
    """Feed constant samples for `seconds`; return the end time."""
    t = t0
    for _ in range(round(seconds / DT)):
        t += DT
        integ.step(t, v, omega_raw)
    return t


class TestIntegration:
    def test_first_sample_only_sets_the_clock(self):
        integ = OdomIntegrator()
        assert integ.step(1.0, 0.5, 0.0) is False
        assert (integ.state.x, integ.state.t) == (0.0, 1.0)

    def test_straight_line(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 2.0, 0.5, 0.0)
        assert integ.state.x == pytest.approx(0.5 * t, abs=1e-9)
        assert integ.state.y == pytest.approx(0.0, abs=1e-12)

    def test_circle_closes(self):
        # 1 m radius at 0.5 m/s: one lap in 4 pi s.
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        _drive(integ, 4 * math.pi, 0.5, 0.5)
        s = integ.state
        assert math.hypot(s.x, s.y) < 0.01
        assert abs(s.theta) < math.radians(0.5)

    def test_quarter_turn_lands_on_the_arc(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, math.pi, 0.5, 0.5)  # about 90 deg of a 1 m circle, turning left
        s, heading = integ.state, 0.5 * t
        assert s.theta == pytest.approx(heading, abs=1e-9)
        assert s.x == pytest.approx(math.sin(heading), abs=1e-5)
        assert s.y == pytest.approx(1.0 - math.cos(heading), abs=1e-5)

    def test_reverse_moves_backwards(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 1.0, -0.5, 0.0)
        assert integ.state.x == pytest.approx(-0.5 * t, abs=1e-9)

    def test_time_not_moving_forward_is_ignored(self):
        integ = OdomIntegrator()
        integ.step(1.0, 0.5, 0.0)
        assert integ.step(1.0, 0.5, 0.0) is False
        assert integ.step(0.9, 0.5, 0.0) is False

    def test_gap_holds_the_pose(self):
        integ = OdomIntegrator(stale_sec=0.25)
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 1.0, 0.5, 0.0)
        x = integ.state.x
        assert integ.step(t + 1.0, 0.5, 0.0) is False  # 1 s gap: no 0.5 m jump
        assert integ.state.x == x and integ.gaps == 1
        end = _drive(integ, 1.0, 0.5, 0.0, t0=t + 1.0)
        assert integ.state.x == pytest.approx(x + 0.5 * (end - t - 1.0), abs=1e-9)


class TestBias:
    BIAS = math.radians(0.2)  # residual after calibration, rad/s

    def test_stopped_car_learns_the_bias_and_holds_heading(self):
        integ = OdomIntegrator(zero_hold_sec=1.0, bias_tau_sec=30.0)
        integ.step(0.0, 0.0, self.BIAS)
        t = _drive(integ, 1.0, 0.0, self.BIAS)
        heading_at_hold = integ.state.theta
        _drive(integ, 150.0, 0.0, self.BIAS, t0=t)  # five time constants
        assert integ.holding
        assert integ.bias == pytest.approx(self.BIAS, rel=0.01)
        assert integ.state.theta == heading_at_hold

    def test_learned_bias_stops_drift_when_driving(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, self.BIAS)
        t = _drive(integ, 200.0, 0.0, self.BIAS)
        start = integ.state.theta
        _drive(integ, 60.0, 0.5, self.BIAS, t0=t)  # straight for a minute
        assert abs(integ.state.theta - start) < math.radians(0.1)
        assert not integ.holding

    def test_without_the_update_a_minute_drifts(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.5, self.BIAS)
        _drive(integ, 60.0, 0.5, self.BIAS)
        assert math.degrees(integ.state.theta) == pytest.approx(12.0, abs=0.1)

    def test_no_update_before_the_hold_time(self):
        integ = OdomIntegrator(zero_hold_sec=1.0)
        integ.step(0.0, 0.0, self.BIAS)
        _drive(integ, 0.9, 0.0, self.BIAS)
        assert integ.bias == 0.0 and not integ.holding

    def test_car_carried_by_hand_is_not_learned(self):
        integ = OdomIntegrator(zero_max_rate=math.radians(2.0))
        integ.step(0.0, 0.0, 0.0)
        _drive(integ, 10.0, 0.0, math.radians(30.0))  # wheels still, car turning
        assert integ.bias == 0.0
        assert math.degrees(integ.state.theta) == pytest.approx(-60.0, abs=0.5)  # wrapped 300

    def test_moving_car_is_not_learned(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.5, self.BIAS)
        _drive(integ, 10.0, 0.5, self.BIAS)
        assert integ.bias == 0.0


class TestQuaternion:
    @pytest.mark.parametrize('theta', [0.0, 0.7, -2.5, math.pi])
    def test_unit_and_about_z(self, theta):
        x, y, z, w = yaw_quaternion(theta)
        assert (x, y) == (0.0, 0.0)
        assert z * z + w * w == pytest.approx(1.0)
        assert 2 * math.atan2(z, w) == pytest.approx(theta)


class _OdomPit:
    """PitNode._integrate_odometry over plain attributes."""

    _integrate_odometry = PitNode._integrate_odometry

    _publish_odometry = PitNode._publish_odometry

    def __init__(self):
        self._odometry = OdomIntegrator()
        self._odom_period = 0.02
        self._odom_next_pub = None
        self._odom_frame, self._odom_child_frame = 'odom', 'base_link'
        self.sent = []
        self._tf = SimpleNamespace(sendTransform=self.sent.append)
        self._pub_odom_integrated = SimpleNamespace(publish=self.sent.append)
        self._yaw_axis = 1
        self._yaw_gain = -1.0
        self._speed_scale = 1.0
        self._odom_gaps_logged = 0
        self._odom_bias_logged = 0.0
        self.logged = []

    def get_logger(self):
        return SimpleNamespace(info=self.logged.append, warn=self.logged.append)


class TestPitNodeOdometry:
    def test_yaw_is_minus_optical_y(self):
        pit = _OdomPit()
        pit._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        pit._integrate_odometry(1_006_000_000, 0.5, [0.3, -0.2, 0.9])
        assert pit._odometry.state.omega == pytest.approx(0.2)  # left turn

    def test_speed_scale_applies(self):
        pit = _OdomPit()
        pit._speed_scale = 1.1
        pit._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        pit._integrate_odometry(1_006_000_000, 0.5, [0.0, 0.0, 0.0])
        assert pit._odometry.state.v == pytest.approx(0.55)

    def test_gap_is_logged_once(self):
        pit = _OdomPit()
        pit._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        pit._integrate_odometry(2_000_000_000, 0.5, [0.0, 0.0, 0.0])
        assert sum('gap' in m for m in pit.logged) == 1

    def test_publishes_at_the_configured_rate_from_the_reader_thread(self):
        pit = _OdomPit()
        for i in range(167):  # 1 s of 6 ms frames
            pit._integrate_odometry(1_000_000_000 + i * 6_000_000, 0.5, [0.0, 0.0, 0.0])
        transforms = [m for m in pit.sent if hasattr(m, 'transform')]
        assert 49 <= len(transforms) <= 51  # 50 Hz average from 6 ms frames
        assert len(pit.sent) == 2 * len(transforms)

    def test_message_and_transform_agree(self):
        state = OdomState(x=1.5, y=-0.25, theta=0.6, v=0.5, omega=0.1, t=12.5)
        tf, odom = build_integrated_odom(state, 'odom', 'base_link')
        assert (tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_link')
        assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_link')
        assert tf.header.stamp == odom.header.stamp
        assert (tf.header.stamp.sec, tf.header.stamp.nanosec) == (12, 500_000_000)
        assert tf.transform.translation.x == odom.pose.pose.position.x == 1.5
        assert tf.transform.rotation.z == odom.pose.pose.orientation.z
        assert 2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w) == pytest.approx(
            0.6
        )
        assert (odom.twist.twist.linear.x, odom.twist.twist.angular.z) == (0.5, 0.1)


def _odom_params():
    return yaml.safe_load((ROOT / 'config' / 'odom.yaml').read_text())['pit_node'][
        'ros__parameters'
    ]


class TestConfig:
    def test_yaml_keys_are_declared_by_pit_node(self):
        src = (ROOT / 'racecar_neo_ros2_driver' / 'pit_node.py').read_text()
        missing = [k for k in _odom_params() if f"declare_parameter('{k}'" not in src]
        assert missing == []

    def test_yaw_axis_is_up_for_optical_imu_axes(self):
        p = _odom_params()
        assert (p['odometry.yaw_rate_axis'], p['odometry.yaw_rate_sign']) == (1, -1.0)

    def test_pit_launch_loads_odom_yaml(self):
        assert "'odom.yaml'" in (ROOT / 'launch' / 'pit.launch.py').read_text()
