"""Unit tests for mux_node.select_mode and joy_is_centered."""

from racecar_neo_ros2_driver.mux_node import joy_is_centered, MuxMode, select_mode


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
