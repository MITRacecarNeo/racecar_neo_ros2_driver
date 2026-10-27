"""Unit tests for pit_node: SystemState bits, axis remap, odometry, display frame."""

import numpy as np
import pytest

from racecar_neo_ros2_driver.mux_node import MuxMode
from racecar_neo_ros2_driver.pit_node import (
    _MODE_BITS,
    build_battery_state,
    build_odom,
    override_led_frame,
    OVERRIDE_LED_RGB,
    PitNode,
    POSE_UNESTIMATED,
    PowerAverage,
    remap_axes,
    SPEED_VARIANCE,
    SYS_DOTMATRIX_ACTIVE,
    SYS_DRIVER_STARTING,
    SYS_LED_ACTIVE,
    SYS_MODE_AUTO,
    SYS_MODE_IDLE,
    SYS_MODE_MANUAL,
    SYS_MODE_MASK,
    TWIST_COMPONENT_UNKNOWN,
)


class TestSystemStateBits:
    """The SystemState byte layout must match firmware cfg::SYS_STATE."""

    def test_bit_layout(self):
        assert SYS_MODE_MASK == 0x03
        assert SYS_DOTMATRIX_ACTIVE == 0x04
        assert SYS_LED_ACTIVE == 0x08
        assert SYS_DRIVER_STARTING == 0x10

    def test_mode_values(self):
        assert [SYS_MODE_IDLE, SYS_MODE_MANUAL, SYS_MODE_AUTO] == [0, 1, 2]

    def test_mode_mapping(self):
        assert _MODE_BITS[MuxMode.IDLE] == SYS_MODE_IDLE
        assert _MODE_BITS[MuxMode.GAMEPAD] == SYS_MODE_MANUAL
        assert _MODE_BITS[MuxMode.AUTONOMY] == SYS_MODE_AUTO

    def test_flags_disjoint_from_mode(self):
        flags = SYS_DOTMATRIX_ACTIVE | SYS_LED_ACTIVE | SYS_DRIVER_STARTING
        assert flags & SYS_MODE_MASK == 0


class TestRemap:
    def test_identity(self):
        out = remap_axes([1.0, 2.0, 3.0], [0, 1, 2], [1.0, 1.0, 1.0])
        assert np.allclose(out, [1.0, 2.0, 3.0])

    def test_reorder_and_flip(self):
        # Map raw (x=front, y=right, z=up) into a z=front frame with a sign flip.
        out = remap_axes([1.0, 2.0, 3.0], [2, 1, 0], [1.0, 1.0, -1.0])
        assert np.allclose(out, [3.0, 2.0, -1.0])


class TestRemapScale:
    def test_deg_to_rad_scale(self):
        # gyro_scale converts deg/s -> rad/s; 180 deg/s -> pi rad/s.
        out = remap_axes([180.0, 0.0, 0.0], [0, 1, 2], [1.0, 1.0, 1.0], np.pi / 180.0)
        assert np.allclose(out, [np.pi, 0.0, 0.0])

    def test_scale_applies_after_remap(self):
        out = remap_axes([1.0, 2.0, 3.0], [2, 1, 0], [-1.0, 1.0, 1.0], 2.0)
        assert np.allclose(out, [-6.0, 4.0, 2.0])


class TestBuildOdom:
    def test_speed_lands_in_forward_twist(self):
        odom = build_odom(2.5, 'odom', 'base_link')
        assert odom.twist.twist.linear.x == pytest.approx(2.5)

    def test_frames_are_set(self):
        odom = build_odom(0.0, 'odom', 'base_link')
        assert odom.header.frame_id == 'odom'
        assert odom.child_frame_id == 'base_link'

    def test_reverse_speed_is_signed(self):
        odom = build_odom(-1.25, 'odom', 'base_link')
        assert odom.twist.twist.linear.x == pytest.approx(-1.25)

    def test_pose_is_marked_unestimated(self):
        # -1 in the first element is the ROS signal for "this block is not a
        # measurement". Zeros would read as a car parked at the origin.
        odom = build_odom(1.0, 'odom', 'base_link')
        assert odom.pose.covariance[0] == POSE_UNESTIMATED

    def test_unestimated_twist_components_are_flagged(self):
        odom = build_odom(1.0, 'odom', 'base_link')
        assert odom.twist.covariance[0] == pytest.approx(SPEED_VARIANCE)
        for i in (7, 14, 21, 28, 35):
            assert odom.twist.covariance[i] == pytest.approx(TWIST_COMPONENT_UNKNOWN)

    def test_heading_is_not_derived(self):
        # No wheelbase or steering calibration exists on this platform, and
        # /motor carries steering normalized to [-1, 1] rather than radians.
        odom = build_odom(3.0, 'odom', 'base_link')
        assert odom.twist.twist.angular.z == 0.0


class _Pit:
    """PitNode._build_display over plain attributes, without rclpy or a serial port."""

    _build_display = PitNode._build_display

    def __init__(self, start=0.0) -> None:
        self._joy_buttons = []
        self._gamepad_btn = 4
        self._auto_btn = 5
        self._dot_frame = None
        self._dot_stamp = 0.0
        self._led_frame = None
        self._led_stamp = 0.0
        self._dot_alert = None
        self._dot_alert_stamp = 0.0
        self._led_alert = None
        self._led_alert_stamp = 0.0
        self._display_timeout = 0.5
        self._start_time = start
        self._led_startup = 10.0
        self._override = False


class TestBuildDisplay:
    def test_idle_with_nothing_to_show(self):
        assert _Pit()._build_display(100.0) == (SYS_MODE_IDLE, None, None)

    def test_bumpers_set_the_mode_bits(self):
        pit = _Pit()
        pit._joy_buttons = [0, 0, 0, 0, 1, 0, 0, 0]
        assert pit._build_display(100.0)[0] == SYS_MODE_MANUAL
        pit._joy_buttons = [0, 0, 0, 0, 0, 1, 0, 0]
        assert pit._build_display(100.0)[0] == SYS_MODE_AUTO

    def test_fresh_frames_are_forwarded_and_flagged(self):
        pit = _Pit()
        pit._dot_frame, pit._dot_stamp = b'\x01' * 24, 99.8
        pit._led_frame, pit._led_stamp = b'\x02' * 252, 99.9
        state, dot, led = pit._build_display(100.0)
        assert state == SYS_DOTMATRIX_ACTIVE | SYS_LED_ACTIVE
        assert dot == b'\x01' * 24
        assert led == b'\x02' * 252

    def test_stale_frames_are_dropped(self):
        pit = _Pit()
        pit._dot_frame, pit._dot_stamp = b'\x01', 99.0
        pit._led_frame, pit._led_stamp = b'\x02', 99.0
        assert pit._build_display(100.0) == (SYS_MODE_IDLE, None, None)

    def test_alert_beats_a_fresh_user_frame(self):
        pit = _Pit()
        pit._dot_frame, pit._dot_stamp = b'\x01' * 192, 99.9
        pit._led_frame, pit._led_stamp = b'\x02' * 252, 99.9
        pit._dot_alert, pit._dot_alert_stamp = b'\x03' * 192, 99.9
        pit._led_alert, pit._led_alert_stamp = b'\x04' * 252, 99.9
        state, dot, led = pit._build_display(100.0)
        assert state == SYS_DOTMATRIX_ACTIVE | SYS_LED_ACTIVE
        assert dot == b'\x03' * 192
        assert led == b'\x04' * 252

    def test_stale_alert_falls_back_to_the_user_frame(self):
        pit = _Pit()
        pit._dot_frame, pit._dot_stamp = b'\x01' * 192, 99.9
        pit._dot_alert, pit._dot_alert_stamp = b'\x03' * 192, 99.0
        assert pit._build_display(100.0)[1] == b'\x01' * 192

    def test_alert_beats_the_override_flash(self):
        pit = _Pit()
        pit._override = True
        pit._led_alert, pit._led_alert_stamp = b'\x04' * 252, 100.0
        assert pit._build_display(100.0)[2] == b'\x04' * 252
        pit._led_alert_stamp = 99.0
        assert pit._build_display(100.0)[2] == override_led_frame(100.0)

    def test_alert_alone_sets_the_flags(self):
        pit = _Pit()
        pit._dot_alert, pit._dot_alert_stamp = b'\x03' * 192, 100.0
        pit._led_alert, pit._led_alert_stamp = b'\x04' * 252, 100.0
        assert pit._build_display(100.0)[0] == SYS_DOTMATRIX_ACTIVE | SYS_LED_ACTIVE

    def test_driver_starting_flag_during_the_startup_window(self):
        pit = _Pit(start=100.0)
        assert pit._build_display(105.0)[0] & SYS_DRIVER_STARTING
        assert not pit._build_display(110.0)[0] & SYS_DRIVER_STARTING

    def test_flags_combine_with_the_mode(self):
        pit = _Pit(start=100.0)
        pit._joy_buttons = [0, 0, 0, 0, 1, 0, 0, 0]
        pit._dot_frame, pit._dot_stamp = b'\x01', 100.0
        state, _, _ = pit._build_display(100.0)
        assert state & SYS_MODE_MASK == SYS_MODE_MANUAL
        assert state & SYS_DOTMATRIX_ACTIVE
        assert state & SYS_DRIVER_STARTING


class _Clock:
    def __init__(self, ns):
        self.ns = ns

    def now(self):
        from rclpy.time import Time

        return Time(nanoseconds=self.ns)


class _StampPit:
    """PitNode._telemetry_stamp over plain attributes."""

    _telemetry_stamp = PitNode._telemetry_stamp

    def __init__(self, use_teensy, receive_ns):
        from racecar_neo_ros2_driver.teensy_clock import TeensyClock

        self._teensy_clock = TeensyClock() if use_teensy else None
        self._clock = _Clock(receive_ns)
        self.logged = []

    def get_clock(self):
        return self._clock

    def get_logger(self):
        pit = self

        class _Log:
            def info(self, msg):
                pit.logged.append(msg)

        return _Log()


class TestTelemetryStamp:
    def test_receive_time_when_disabled(self):
        pit = _StampPit(False, 5_000_000_123)
        stamp = pit._telemetry_stamp(1234)
        assert (stamp.sec, stamp.nanosec) == (5, 123)

    def test_teensy_spacing_survives_bursty_arrival(self):
        # A prompt frame (1 ms transport delay) sets the floor; the next four,
        # 6 ms apart at the Teensy, arrive in a burst 0.5 ms apart.
        pit = _StampPit(True, 0)
        stamps = []
        for i, rx_ms in enumerate([1.0, 30.0, 30.5, 31.0, 31.5]):
            pit._clock.ns = int(rx_ms * 1e6)
            s = pit._telemetry_stamp(i * 6000)
            stamps.append(s.sec * 10**9 + s.nanosec)
        assert np.diff(stamps).tolist() == [6_000_000] * 4
        assert stamps[-1] == 25_000_000  # sample time + the 1 ms floor

    def test_restart_is_logged(self):
        pit = _StampPit(True, 10**9)
        pit._telemetry_stamp(1_000_000_000)
        pit._clock.ns = 10**9 + 6_000_000
        pit._telemetry_stamp(5)  # Teensy reset
        assert any('restarted' in m for m in pit.logged)


class TestBatteryState:
    def test_average_closes_each_period(self):
        avg = PowerAverage(period_sec=1.0)
        assert avg.add(0.0, 8.0, 1.0) is None
        assert avg.add(0.5, 8.2, 3.0) is None
        assert avg.add(1.0, 8.4, 2.0) == pytest.approx((8.2, 2.0))
        assert avg.add(1.5, 7.0, 1.0) is None
        assert avg.add(2.0, 7.2, 1.0) == pytest.approx((7.1, 1.0))

    def test_discharge_current_is_negative(self):
        msg = build_battery_state(8.1, 2.5)
        assert msg.voltage == pytest.approx(8.1) and msg.current == pytest.approx(-2.5)
        assert msg.present
        assert msg.power_supply_status == msg.POWER_SUPPLY_STATUS_DISCHARGING
        assert np.isnan(msg.percentage)


class TestOverrideDisplay:
    def test_override_flashes_amber_and_shows_auto(self):
        pit = _Pit()
        pit._override = True
        state, _, led = pit._build_display(100.0)
        assert state == SYS_MODE_AUTO | SYS_LED_ACTIVE
        assert led == bytes(OVERRIDE_LED_RGB) * 84

    def test_flash_goes_dark_between_beats(self):
        assert override_led_frame(100.25) == b'\x00' * 252

    def test_program_frame_wins_over_the_flash(self):
        pit = _Pit()
        pit._override = True
        pit._led_frame, pit._led_stamp = b'\x02' * 252, 99.9
        assert pit._build_display(100.0)[2] == b'\x02' * 252

    def test_lb_shows_manual_during_override(self):
        pit = _Pit()
        pit._override = True
        pit._joy_buttons = [0, 0, 0, 0, 1, 0, 0, 0]
        assert pit._build_display(100.0)[0] & SYS_MODE_MASK == SYS_MODE_MANUAL


class _Pub:
    def __init__(self, subscribers):
        self.subscribers = subscribers

    def get_subscription_count(self):
        return self.subscribers


class _Gate:
    """PitNode's subscriber gate over plain publishers."""

    _refresh_listeners = PitNode._refresh_listeners
    _wanted = PitNode._wanted

    def __init__(self, pubs):
        self.pubs = pubs
        self._listened = {}
        self._listeners_at = 0.0

    def _gated_publishers(self):
        return self.pubs


class TestSubscriberGate:
    def test_publishes_until_the_first_count(self):
        pub = _Pub(0)
        assert _Gate([pub])._wanted(pub)

    def test_topic_with_no_subscriber_is_skipped(self, monkeypatch):
        listened, idle = _Pub(1), _Pub(0)
        gate = _Gate([listened, idle])
        monkeypatch.setattr('racecar_neo_ros2_driver.pit_node.time.monotonic', lambda: 100.0)
        gate._refresh_listeners()
        assert gate._wanted(listened) and not gate._wanted(idle)
        assert not gate._wanted(None)

    def test_new_subscriber_is_seen_at_the_next_refresh(self, monkeypatch):
        pub = _Pub(0)
        gate = _Gate([pub])
        clock = [100.0]
        monkeypatch.setattr('racecar_neo_ros2_driver.pit_node.time.monotonic', lambda: clock[0])
        gate._refresh_listeners()
        pub.subscribers = 1
        clock[0] = 100.5
        gate._refresh_listeners()
        assert not gate._wanted(pub)
        clock[0] = 101.0
        gate._refresh_listeners()
        assert gate._wanted(pub)
