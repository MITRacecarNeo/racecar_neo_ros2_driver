"""Unit tests for the dotmatrix_node rasterizer helpers."""

import pytest

from racecar_neo_ros2_driver.dotmatrix_node import (
    decode_pixel_array,
    GLYPH_AUTO,
    GLYPH_IDLE,
    GLYPH_TELEOP,
    mode_glyph,
    mode_label,
    rendered_text_width,
    scroll_offset,
)
from racecar_neo_ros2_driver.mux_node import MuxMode


class TestGlyphShapes:
    @pytest.mark.parametrize('glyph', [GLYPH_IDLE, GLYPH_TELEOP, GLYPH_AUTO])
    def test_glyph_is_8x8(self, glyph):
        assert len(glyph) == 8
        for row in glyph:
            assert len(row) == 8
            assert set(row) <= {'.', 'X'}
        assert any('X' in row for row in glyph)


class TestModeGlyphAndLabel:
    def test_glyph_mapping(self):
        assert mode_glyph(MuxMode.IDLE) is GLYPH_IDLE
        assert mode_glyph(MuxMode.GAMEPAD) is GLYPH_TELEOP
        assert mode_glyph(MuxMode.AUTONOMY) is GLYPH_AUTO

    def test_label_mapping(self):
        assert mode_label(MuxMode.IDLE) == 'IDLE'
        assert mode_label(MuxMode.GAMEPAD) == 'MAN'
        assert mode_label(MuxMode.AUTONOMY) == 'AUTO'

    @pytest.mark.parametrize('mode', list(MuxMode))
    def test_label_fits_16px_region(self, mode):
        from luma.core.legacy.font import proportional
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT

        assert rendered_text_width(mode_label(mode), proportional(TINY_FONT)) <= 16


class TestSplashConfig:
    def test_default_splash_in_config_yaml(self):
        from pathlib import Path

        cfg = (Path(__file__).parent.parent / 'config' / 'dotmatrix.yaml').read_text()
        assert 'splash_message:' in cfg
        assert 'Welcome to RACECAR Neo' in cfg


class TestDecodePixelArray:
    def test_perfect_8x24_array_decodes_row_major(self):
        data = ([0] * 24 + [1] * 24) * 4
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert len(rows) == 8
        assert rows[0] == '.' * 24
        assert rows[1] == 'X' * 24
        assert rows[6] == '.' * 24
        assert rows[7] == 'X' * 24

    def test_nonzero_value_is_on(self):
        data = [0, 255, 1, 17, 0] + [0] * 19 + [0] * 24 * 7
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert rows[0][:5] == '.XXX.'

    def test_short_row_pads_with_off(self):
        data = [1] * 24
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert rows[0] == 'X' * 24
        for r in rows[1:]:
            assert r == '.' * 24

    def test_long_data_is_truncated(self):
        data = [1] * (24 * 9)
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert len(rows) == 8
        assert all(row == 'X' * 24 for row in rows)

    def test_partial_last_row_pads(self):
        data = [0] * (24 * 7) + [1] * 5
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert rows[7] == 'X' * 5 + '.' * 19

    def test_too_short_raises(self):
        with pytest.raises(ValueError):
            decode_pixel_array([1, 1, 1], expected_height=8, expected_width=24)

    def test_accepts_bytes(self):
        rows = decode_pixel_array(bytes([1] * 24), expected_height=8, expected_width=24)
        assert rows[0] == 'X' * 24

    def test_works_for_different_widths(self):
        data = [1] * (32 * 8)
        rows = decode_pixel_array(data, expected_height=8, expected_width=32)
        assert len(rows) == 8
        assert all(len(r) == 32 and r == 'X' * 32 for r in rows)


class TestPatchedTinyFont:
    """The module-level TINY_FONT replaces luma's 'N' with a clearer diagonal."""

    def test_n_glyph_is_overridden(self):
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        from luma.core.legacy.font import TINY_FONT as STOCK_TINY_FONT

        assert TINY_FONT[ord('N')] != STOCK_TINY_FONT[ord('N')]

    def test_n_glyph_has_diagonal(self):
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT

        n = TINY_FONT[ord('N')]
        assert len(n) == 4
        c1, c2 = n[1], n[2]
        assert bin(c1).count('1') == 1
        assert bin(c2).count('1') == 1
        assert (c2.bit_length() - 1) == (c1.bit_length() - 1) + 1


class TestRenderedTextWidth:
    def test_empty_string_is_zero(self):
        from luma.core.legacy.font import TINY_FONT, proportional

        assert rendered_text_width('', proportional(TINY_FONT)) == 0

    def test_known_widths_in_tiny_font(self):
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        from luma.core.legacy.font import proportional

        font = proportional(TINY_FONT)
        assert rendered_text_width('IDLE', font) == 15
        assert rendered_text_width('AUTO', font) == 15
        # 'MAN' uses the patched 4-column 'N', so 12 px (vs luma's stock N).
        assert rendered_text_width('MAN', font) == 12


class TestScrollOffset:
    def test_fits_returns_zero(self):
        assert scroll_offset(0.0, 10, 32, 4.0) == 0
        assert scroll_offset(5.0, 10, 32, 4.0) == 0

    def test_at_phase_zero_starts_off_right(self):
        assert scroll_offset(0.0, 64, 32, 4.0) == -32

    def test_mid_phase_advances_left(self):
        assert scroll_offset(2.0, 64, 32, 4.0) == 16

    def test_period_loops(self):
        assert scroll_offset(4.0, 64, 32, 4.0) == -32

    def test_zero_or_negative_period_is_safe(self):
        assert scroll_offset(1.0, 64, 32, 0.0) == 0
        assert scroll_offset(1.0, 64, 32, -1.0) == 0
