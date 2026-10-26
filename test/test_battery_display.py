"""Unit tests for scripts/battery_display.py: dot matrix and LED alert frames."""

from conftest import load_script
import pytest

bd = load_script('battery_display')


def _rows(frame):
    return [
        ''.join('X' if frame[r * bd.COLS + c] else '.' for c in range(bd.COLS)) for r in range(8)
    ]


def _led(frame, i):
    start = 3 * i
    end = start + 3
    return frame[start:end]


def _lit(frame):
    return [_led(frame, i) != b'\x00\x00\x00' for i in range(bd.LED_COUNT)]


class TestDotMatrix:
    @pytest.mark.parametrize('text', ['LOW', 'BATT', '!'])
    def test_words_fit_inside_the_outline(self, text):
        rows = bd.battery_rows(text)
        assert len(rows) == bd.ROWS and all(len(r) == bd.COLS for r in rows)
        assert rows[0] == rows[-1] == '.XXXXXXXXXXXXXXXXXXXX...'
        # The outline's sides and terminal stay intact.
        assert all(r[1] == 'X' and r[20] == 'X' for r in rows)
        assert all(rows[r][21] == 'X' for r in range(2, 6))
        assert rows[1] == '.X..................X...'

    def test_exclamation_mark_is_centered(self):
        rows = bd.battery_rows('!')
        assert [r[10] for r in rows[2:7]] == ['X', 'X', 'X', '.', 'X']

    def test_frame_format(self):
        frame = bd.rows_to_frame(bd.battery_rows('LOW'))
        assert len(frame) == bd.ROWS * bd.COLS == 192
        assert set(frame) <= {0, 1}
        assert _rows(frame) == bd.battery_rows('LOW')

    @pytest.mark.parametrize('text', ['0', '1', '7', '10', '20', 'SHUT', 'DOWN'])
    def test_large_text_fits_and_centers(self, text):
        rows = bd.large_rows(text)
        assert len(rows) == bd.ROWS and all(len(r) == bd.COLS for r in rows)
        lit_cols = [c for c in range(bd.COLS) if any(r[c] == 'X' for r in rows)]
        left, right = lit_cols[0], bd.COLS - 1 - lit_cols[-1]
        assert abs(left - right) <= 1

    @pytest.mark.parametrize('now,text', [(0.2, 'LOW'), (1.2, 'BATT'), (2.2, '!'), (3.2, 'LOW')])
    def test_warn_cycles_low_batt_mark(self, now, text):
        assert _rows(bd.dot_frame('warn', None, now)) == bd.battery_rows(text)

    @pytest.mark.parametrize(
        'now,expected',
        [
            (40.2, bd.battery_rows('BATT')),
            (79.2, bd.battery_rows('BATT')),
            (80.2, bd.number_rows(20)),
            (87.2, bd.number_rows(13)),
            (88.2, bd.battery_rows('LOW')),
            (89.2, bd.battery_rows('BATT')),
            (90.2, bd.number_rows(10)),
            (99.9, bd.number_rows(1)),
        ],
    )
    def test_countdown_timeline(self, now, expected):
        # Shutdown at 100 s: the cycle above 20 s, then the seconds, LOW and BATT
        # at 12 and 11 s.
        assert _rows(bd.dot_frame('countdown', 100.0, now)) == expected

    def test_shut_then_down(self):
        assert _rows(bd.dot_frame('off', 100.0, 100.4)) == bd.large_rows('SHUT')
        assert _rows(bd.dot_frame('off', 100.0, 101.4)) == bd.large_rows('DOWN')
        assert _rows(bd.dot_frame('off', 100.0, 105.0)) == bd.large_rows('DOWN')
        assert bd.SHUTDOWN_FRAMES_SEC == 2.0

    def test_nothing_when_ok(self):
        assert bd.dot_frame('ok', None, 0.0) is None


class TestLed:
    def test_frame_format(self):
        assert len(bd.led_frame('warn', None, 0.1)) == bd.LED_COUNT * 3 == 252

    def test_segments_alternate(self):
        lit = _lit(bd.led_frame('warn', None, 0.1))
        assert lit == [(i // bd.SEGMENT) % 2 == 0 for i in range(bd.LED_COUNT)]
        assert _lit(bd.led_frame('warn', None, 1.1)) == [not x for x in lit]

    def test_one_color_at_a_time(self):
        orange = bd.led_frame('warn', None, 0.1)
        red = bd.led_frame('warn', None, 1.1)
        assert {_led(orange, i) for i in range(bd.LED_COUNT)} == {bytes(bd.ORANGE), b'\x00' * 3}
        assert {_led(red, i) for i in range(bd.LED_COUNT)} == {bytes(bd.RED), b'\x00' * 3}

    def test_red_matches_the_firmware_charge_bar(self):
        assert bd.RED == (255, 0, 0)

    @pytest.mark.parametrize(
        'level,at,now,swaps_per_sec',
        [
            ('warn', None, 0.0, 1.0),
            ('countdown', 100.0, 50.0, 2.0),
            ('countdown', 100.0, 95.0, 4.0),
            ('off', 100.0, 100.0, 4.0),
        ],
    )
    def test_swap_rate(self, level, at, now, swaps_per_sec):
        swaps = 0
        prev = _lit(bd.led_frame(level, at, now))
        for k in range(1, 800):
            cur = _lit(bd.led_frame(level, at, now + k * 0.005))
            swaps += cur != prev
            prev = cur
        # Four seconds sampled at 200 Hz.
        assert swaps == pytest.approx(4 * swaps_per_sec, abs=1)

    def test_nothing_when_ok(self):
        assert bd.led_frame('ok', None, 0.0) is None
