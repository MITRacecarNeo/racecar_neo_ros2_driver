"""Unit tests for mux_node.select_mode and joy_is_centered."""

from racecar_neo_ros2_driver.mux_node import (
    joy_is_centered,
    MuxMode,
    RcAuthority,
    select_mode,
    select_rc_mode,
)


GAMEPAD = 4
AUTO = 5


def _btns(**kwargs):
    """Build a buttons list with named buttons set to 1."""
    arr = [0] * 8
    for name, val in kwargs.items():
        idx = {'gp': GAMEPAD, 'ao': AUTO}[name]
        arr[idx] = val
    return arr


class TestModeSelection:
    def test_neither_pressed_is_idle(self):
        assert select_mode(_btns(), GAMEPAD, AUTO) == MuxMode.IDLE

    def test_gamepad_only_is_gamepad(self):
        assert select_mode(_btns(gp=1), GAMEPAD, AUTO) == MuxMode.GAMEPAD

    def test_auto_only_is_autonomy(self):
        assert select_mode(_btns(ao=1), GAMEPAD, AUTO) == MuxMode.AUTONOMY

    def test_both_pressed_is_idle(self):
        """Both bumpers = safety idle (matches v1 behavior)."""
        assert select_mode(_btns(gp=1, ao=1), GAMEPAD, AUTO) == MuxMode.IDLE

    def test_empty_buttons_is_idle(self):
        """Short Joy message (controller not fully reporting) defaults to idle."""
        assert select_mode([], GAMEPAD, AUTO) == MuxMode.IDLE

    def test_buttons_shorter_than_indices(self):
        """Indices past the array length must not throw."""
        assert select_mode([0, 0, 0], GAMEPAD, AUTO) == MuxMode.IDLE

    def test_custom_button_indices(self):
        """The function should respect configured indices, not hardcoded 4/5."""
        buttons = [1, 0, 0, 0, 0, 0]
        assert select_mode(buttons, 0, 1) == MuxMode.GAMEPAD
        assert select_mode([0, 1, 0, 0, 0, 0], 0, 1) == MuxMode.AUTONOMY


class TestJoyCentered:
    def test_all_zero_is_centered(self):
        assert joy_is_centered([0.0, 0.0, 0.0, 0.0])

    def test_empty_axes_is_centered(self):
        """An empty axes list trivially satisfies the threshold."""
        assert joy_is_centered([])

    def test_below_threshold_is_centered(self):
        assert joy_is_centered([0.05, -0.1, 0.0, 0.15], threshold=0.2)

    def test_above_threshold_is_not_centered(self):
        """A single stuck axis must block arming."""
        assert not joy_is_centered([0.0, 0.5, 0.0, 0.0], threshold=0.2)

    def test_negative_above_threshold_is_not_centered(self):
        assert not joy_is_centered([0.0, -0.9, 0.0, 0.0], threshold=0.2)

    def test_xbox_trigger_rest_blocks_without_ignore(self):
        # EasySMX in Xbox-360 mode: axes[2] (LT) and axes[5] (RT) rest at +1.0,
        # not 0.0. Without ignore_axes the arming gate never opens.
        axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert not joy_is_centered(axes, threshold=0.2)

    def test_xbox_trigger_rest_ok_with_ignore(self):
        # Same axes, but axes 2 and 5 excluded — should arm.
        axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert joy_is_centered(axes, threshold=0.2, ignore_axes=(2, 5))

    def test_ignore_axes_does_not_excuse_stuck_stick(self):
        # axes 2 and 5 ignored, but axis 1 (left-stick Y) is stuck forward.
        # Must still block arming — ignoring triggers shouldn't excuse sticks.
        axes = [0.0, 0.8, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert not joy_is_centered(axes, threshold=0.2, ignore_axes=(2, 5))


# A frame with the mode channel centered; index 1 moves between the two so the
# authority sees live data rather than a frozen buffer.
LIVE_A = [0.0, 0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
LIVE_B = [0.0, 0.12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _granted(auth, t0=100.0, hold=1.0):
    """Drive an RcAuthority to the granted state; return the time it happened."""
    auth.observe_channels(LIVE_A)
    auth.observe_channels(LIVE_B)
    auth.update(t0, True, t0, LIVE_B, t0)
    t = t0 + hold + 0.01
    assert auth.update(t, True, t, LIVE_B, t) is True
    return t


class TestSelectRcMode:
    def test_middle_is_idle(self):
        assert select_rc_mode([0.0] * 8, 5, 0.35) == MuxMode.IDLE

    def test_up_is_manual(self):
        ch = [0.0] * 8
        ch[5] = 1.0
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.GAMEPAD

    def test_down_is_autonomous(self):
        ch = [0.0] * 8
        ch[5] = -1.0
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.AUTONOMY

    def test_inside_deadband_is_idle(self):
        ch = [0.0] * 8
        ch[5] = 0.30
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.IDLE

    def test_short_frame_is_idle(self):
        assert select_rc_mode([0.0, 0.0], 5, 0.35) == MuxMode.IDLE

    def test_empty_frame_is_idle(self):
        assert select_rc_mode([], 5, 0.35) == MuxMode.IDLE

    def test_channel_index_is_honored(self):
        ch = [0.0] * 8
        ch[2] = 1.0
        assert select_rc_mode(ch, 2, 0.35) == MuxMode.GAMEPAD
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.IDLE


class TestRcAuthorityGrant:
    def test_not_granted_before_the_hold_elapses(self):
        a = RcAuthority(hold_sec=1.0)
        a.observe_channels(LIVE_A)
        a.observe_channels(LIVE_B)
        assert a.update(100.0, True, 100.0, LIVE_B, 100.0) is False
        assert a.update(100.5, True, 100.5, LIVE_B, 100.5) is False

    def test_granted_after_the_hold(self):
        assert _granted(RcAuthority(hold_sec=1.0)) > 0

    def test_frozen_channels_never_grant(self):
        # An uninitialized buffer holding a plausible constant passes the band
        # check; it must not pass this one.
        a = RcAuthority(hold_sec=1.0)
        t = 100.0
        for _ in range(50):
            a.observe_channels(LIVE_A)
            assert a.update(t, True, t, LIVE_A, t) is False
            t += 0.1

    def test_link_down_is_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        a.observe_channels(LIVE_A)
        a.observe_channels(LIVE_B)
        assert a.update(100.0, False, 100.0, LIVE_B, 100.0) is False
        assert a.update(200.0, False, 200.0, LIVE_B, 200.0) is False

    def test_empty_channels_are_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        assert a.update(100.0, True, 100.0, [], 100.0) is False

    def test_missing_stamps_are_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        assert a.update(100.0, True, None, LIVE_B, None) is False


class TestRcAuthorityRevoke:
    def test_one_bad_frame_revokes_immediately(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        assert a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01) is False

    def test_stale_link_revokes(self):
        a = RcAuthority(timeout_sec=0.5, hold_sec=1.0)
        t = _granted(a)
        # link stamp stops advancing while channels keep arriving
        assert a.update(t + 1.0, True, t, LIVE_B, t + 1.0) is False

    def test_stale_channels_revoke(self):
        a = RcAuthority(timeout_sec=0.5, hold_sec=1.0)
        t = _granted(a)
        assert a.update(t + 1.0, True, t + 1.0, LIVE_B, t) is False

    def test_revoking_restarts_the_full_hold(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01)
        # link returns; authority must serve the hold again, not resume
        a.observe_channels(LIVE_A)
        t2 = t + 1.0
        assert a.update(t2, True, t2, LIVE_A, t2) is False
        assert a.update(t2 + 0.5, True, t2 + 0.5, LIVE_A, t2 + 0.5) is False


class TestRcArming:
    def test_autonomy_is_held_until_the_switch_is_seen_at_middle(self):
        # A car powered on with SWB already down must not go autonomous.
        a = RcAuthority()
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE
        assert a.gate(MuxMode.GAMEPAD) == MuxMode.IDLE

    def test_middle_arms_and_then_modes_pass(self):
        a = RcAuthority()
        assert a.gate(MuxMode.IDLE) == MuxMode.IDLE
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.AUTONOMY
        assert a.gate(MuxMode.GAMEPAD) == MuxMode.GAMEPAD

    def test_reset_disarms(self):
        a = RcAuthority()
        a.gate(MuxMode.IDLE)
        a.reset()
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE

    def test_losing_the_link_disarms(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        a.gate(MuxMode.IDLE)
        a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01)
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE
