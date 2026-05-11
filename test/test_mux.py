"""Unit tests for mux_node.select_mode."""

from racecar_neo_ros2_driver.mux_node import MuxMode, select_mode


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
