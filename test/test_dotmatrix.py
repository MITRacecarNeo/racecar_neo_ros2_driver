"""Unit tests for dotmatrix_node pure helpers."""

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
    text_pixel_width,
)
from racecar_neo_ros2_driver.mux_node import MuxMode


class TestGlyphShapes:
    @pytest.mark.parametrize('glyph', [GLYPH_IDLE, GLYPH_TELEOP, GLYPH_AUTO])
    def test_glyph_is_8x8(self, glyph):
        assert len(glyph) == 8
        for row in glyph:
            assert len(row) == 8
            assert set(row) <= {'.', 'X'}

    def test_idle_glyph_is_nonempty(self):
        assert any('X' in row for row in GLYPH_IDLE)

    def test_teleop_glyph_is_nonempty(self):
        assert any('X' in row for row in GLYPH_TELEOP)

    def test_auto_glyph_is_nonempty(self):
        assert any('X' in row for row in GLYPH_AUTO)


class TestModeGlyph:
    def test_idle_returns_idle_bitmap(self):
        assert mode_glyph(MuxMode.IDLE) is GLYPH_IDLE

    def test_gamepad_returns_teleop_bitmap(self):
        assert mode_glyph(MuxMode.GAMEPAD) is GLYPH_TELEOP

    def test_autonomy_returns_auto_bitmap(self):
        assert mode_glyph(MuxMode.AUTONOMY) is GLYPH_AUTO


class TestModeLabel:
    def test_idle_label(self):
        assert mode_label(MuxMode.IDLE) == 'IDLE'

    def test_gamepad_label_is_man(self):
        # GAMEPAD enum maps to "MAN" (abbreviated MANUAL): the full "MANUAL"
        # renders at 23 px in TINY_FONT, overflowing the 16 px label region.
        assert mode_label(MuxMode.GAMEPAD) == 'MAN'

    def test_autonomy_label(self):
        assert mode_label(MuxMode.AUTONOMY) == 'AUTO'

    @pytest.mark.parametrize('mode', list(MuxMode))
    def test_label_fits_label_region_when_rendered(self, mode):
        # Actually render with luma and check the rightmost lit pixel. Our
        # text_pixel_width over-counts (includes the per-char trailing zero
        # column) so we measure pixels for real.
        from luma.core.legacy import text
        from luma.core.legacy.font import TINY_FONT, proportional
        from PIL import Image, ImageDraw

        img = Image.new('1', (32, 8))
        draw = ImageDraw.Draw(img)
        text(draw, (0, 0), mode_label(mode), fill='white',
             font=proportional(TINY_FONT))
        cols = [x for x in range(32) for y in range(8) if img.getpixel((x, y))]
        rendered_width = (max(cols) + 1) if cols else 0
        assert rendered_width <= 16, (
            f'Label {mode_label(mode)!r} for {mode.name} renders at '
            f'{rendered_width} px; only 16 px available right of the glyph.'
        )


class TestDecodePixelArray:
    def test_perfect_8x24_array_decodes_row_major(self):
        # All zero row, then all-one row, alternating × 4 = 8 rows × 24 cols.
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
        # Only 24 entries provided (one full row); remaining 7 rows pad to off.
        data = [1] * 24
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert rows[0] == 'X' * 24
        for r in rows[1:]:
            assert r == '.' * 24

    def test_long_data_is_truncated(self):
        # 9 rows worth of ones — only first 8 used.
        data = [1] * (24 * 9)
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert len(rows) == 8
        assert all(row == 'X' * 24 for row in rows)

    def test_partial_last_row_pads(self):
        # 7 full rows + 5 entries of an 8th row → 8th row gets 5 lit + 19 off.
        data = [0] * (24 * 7) + [1] * 5
        rows = decode_pixel_array(data, expected_height=8, expected_width=24)
        assert rows[7] == 'X' * 5 + '.' * 19

    def test_too_short_raises(self):
        # Fewer than one full row → can't possibly be intentional.
        with pytest.raises(ValueError):
            decode_pixel_array([1, 1, 1], expected_height=8, expected_width=24)

    def test_accepts_bytes(self):
        # UInt8MultiArray.data can come across as a bytes object in some serializers.
        rows = decode_pixel_array(bytes([1] * 24), expected_height=8, expected_width=24)
        assert rows[0] == 'X' * 24

    def test_works_for_different_widths(self):
        # Future-proof: a 4-cascade (32px) display.
        data = [1] * (32 * 8)
        rows = decode_pixel_array(data, expected_height=8, expected_width=32)
        assert len(rows) == 8
        assert all(len(r) == 32 and r == 'X' * 32 for r in rows)


class TestPatchedTinyFont:
    """The module-level TINY_FONT replaces luma's 'N' with a clearer diagonal."""

    def test_n_glyph_is_overridden(self):
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        from luma.core.legacy.font import TINY_FONT as STOCK_TINY_FONT
        assert TINY_FONT[ord('N')] != STOCK_TINY_FONT[ord('N')], (
            'TINY_FONT N should be patched away from luma default'
        )

    def test_n_glyph_has_diagonal(self):
        # The diagonal stroke means cols 1 and 2 each have exactly one bit set,
        # and those bits are at adjacent successive rows (col 1 row R, col 2 row R+1).
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        n = TINY_FONT[ord('N')]
        assert len(n) == 4, f'patched N should be 4 cols, got {len(n)}'
        # Find the single bit position in cols 1 and 2.
        c1, c2 = n[1], n[2]
        # popcount == 1 in both inner columns.
        assert bin(c1).count('1') == 1, f'col 1 should have one bit, got {c1:b}'
        assert bin(c2).count('1') == 1, f'col 2 should have one bit, got {c2:b}'
        # Diagonal: col 2's bit is one row below col 1's bit.
        row1 = c1.bit_length() - 1
        row2 = c2.bit_length() - 1
        assert row2 == row1 + 1, f'expected diagonal (rows {row1} → {row2})'

    def test_man_still_fits_label_region(self):
        # Patched N is still 4 cols wide, so "MAN" should still render at 11 px.
        from luma.core.legacy.font import proportional
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        assert rendered_text_width('MAN', proportional(TINY_FONT)) <= 16


class TestRenderedTextWidth:
    """rendered_text_width measures what luma actually paints."""

    def test_empty_string_is_zero(self):
        from luma.core.legacy.font import TINY_FONT, proportional
        assert rendered_text_width('', proportional(TINY_FONT)) == 0

    def test_known_label_widths_in_tiny_font(self):
        # These are the actual painted widths luma produces; if luma ever
        # changes its renderer these values will need updating.
        from luma.core.legacy.font import TINY_FONT, proportional
        font = proportional(TINY_FONT)
        assert rendered_text_width('IDLE', font) == 15
        assert rendered_text_width('AUTO', font) == 15
        assert rendered_text_width('MAN', font) == 11

    def test_short_label_centers_in_label_region(self):
        # Centering math: x = 8 + (16 - rendered) / 2.
        # MAN at 11 px → 8 + (16-11)/2 = 8 + 2 = 10. Glyph ends at x=7, so
        # there's a 2 px gap on each side of "MAN".
        from luma.core.legacy.font import TINY_FONT, proportional
        font = proportional(TINY_FONT)
        rendered = rendered_text_width('MAN', font)
        origin = 8 + max(0, (16 - rendered) // 2)
        assert origin == 10


class TestTextPixelWidth:
    def test_empty_string_is_zero(self):
        # Font argument irrelevant for the empty-string short-circuit.
        assert text_pixel_width('', font=None) == 0

    def test_single_char_uses_font_glyph_length(self):
        # 256-entry list mimicking luma's font shape; 'A' at index 65 is 3 cols.
        font = [[0]] * 256
        font[ord('A')] = [0b1, 0b1, 0b1]
        # single char: width = len(glyph) + 1, then -1 for trailing gap = len(glyph)
        assert text_pixel_width('A', font) == 3

    def test_two_chars_include_gap(self):
        font = [[0]] * 256
        font[ord('A')] = [0b1, 0b1]  # 2 columns
        # AA: 2 + 1 + 2 = 5 (two glyphs + one gap)
        assert text_pixel_width('AA', font) == 5

    def test_out_of_range_char_falls_back_to_space(self):
        # When a glyph lookup raises (short list / proportional), fall back to space.
        font = [[0]] * 33  # only entries 0..32 (32 = space)
        font[ord(' ')] = [0b1]  # 1 column space
        # ord('Q')=81 raises IndexError → falls back to font[ord(' ')] → 1 col
        assert text_pixel_width('Q', font) == 1

    def test_works_with_proportional_wrapper(self):
        # Integration check against the real luma proportional() wrapper used
        # at runtime — must not call len() on it.
        from luma.core.legacy.font import proportional
        from racecar_neo_ros2_driver.dotmatrix_node import TINY_FONT
        width = text_pixel_width('HELLO', proportional(TINY_FONT))
        assert width > 0


class TestScrollOffset:
    def test_fits_returns_zero(self):
        assert scroll_offset(0.0, total_width=10, viewport_width=32,
                             scroll_period_s=4.0) == 0
        assert scroll_offset(5.0, total_width=10, viewport_width=32,
                             scroll_period_s=4.0) == 0

    def test_at_phase_zero_starts_off_right(self):
        # phase=0 → offset = 0 - viewport_width = -viewport_width
        # i.e. text origin sits to the right of the viewport (text not yet visible)
        assert scroll_offset(0.0, total_width=64, viewport_width=32,
                             scroll_period_s=4.0) == -32

    def test_mid_phase_advances_left(self):
        # at half-period, phase=0.5, travel=64+32=96, offset = 0.5*96 - 32 = 16
        assert scroll_offset(2.0, total_width=64, viewport_width=32,
                             scroll_period_s=4.0) == 16

    def test_period_loops(self):
        # elapsed=4.0 == one full period → phase wraps to 0
        assert scroll_offset(4.0, total_width=64, viewport_width=32,
                             scroll_period_s=4.0) == -32

    def test_zero_or_negative_period_is_safe(self):
        assert scroll_offset(1.0, total_width=64, viewport_width=32,
                             scroll_period_s=0.0) == 0
        assert scroll_offset(1.0, total_width=64, viewport_width=32,
                             scroll_period_s=-1.0) == 0
