"""
Low-battery frames for the dot matrix and the LED strip.

The battery guard (battery_guard.py) publishes these on /dotmatrix/alert and
/led/alert, which pit_node shows ahead of every other frame. Formats match
pit_node: the dot matrix is 8 rows x 24 columns, one byte per pixel (0 or
1), row by row; the LED strip is 84 RGB triplets. The firmware scales LED
brightness down, so colors are full scale here.

Dot matrix, one frame per second:

- warn, and the countdown above 20 s: LOW, BATT, then "!" inside the
  battery outline.
- countdown from 20 s: the seconds left, with LOW and BATT at 12 and 11 s.
- off (shutting down): SHUT, then DOWN; the guard waits SHUTDOWN_FRAMES_SEC
  for them before it stops anything.

LED strip: orange on the even segments, then red on the odd ones, one color
at a time; 1 swap a second while warning, 2 in the countdown, 4 from 10 s
and while shutting down.
"""

from __future__ import annotations

from collections.abc import Mapping
import math

ROWS, COLS = 8, 24
LED_COUNT = 84
SEGMENT = 7
ORANGE = (255, 90, 0)
# The firmware's charge bar red (CRGB::Red).
RED = (255, 0, 0)
LED_SWAPS_PER_SEC = {'warn': 1.0, 'countdown': 2.0, 'final': 4.0, 'off': 4.0}
FINAL_SECONDS = 10
NUMBERS_FROM = 20
# Seconds left that show LOW and BATT between the numbers.
WORD_SECONDS = {12: 'LOW', 11: 'BATT'}
WARN_CYCLE = ('LOW', 'BATT', '!')
FRAME_SEC = 1.0
SHUTDOWN_WORDS = ('SHUT', 'DOWN')
SHUTDOWN_FRAMES_SEC = FRAME_SEC * len(SHUTDOWN_WORDS)

_OUTLINE_TOP = '.XXXXXXXXXXXXXXXXXXXX...'
_OUTLINE_SIDE = '.X..................X...'
# The terminal on the right, rows 2 to 5.
_OUTLINE_SIDE_TERMINAL = '.X..................XX..'
# Columns inside the outline.
_INSIDE_LEFT, _INSIDE_WIDTH = 2, 18

# 3 x 5 letters for words inside the outline.
_SMALL = {
    'A': ('.X.', 'X.X', 'XXX', 'X.X', 'X.X'),
    'B': ('XX.', 'X.X', 'XX.', 'X.X', 'XX.'),
    'L': ('X..', 'X..', 'X..', 'X..', 'XXX'),
    'O': ('XXX', 'X.X', 'X.X', 'X.X', 'XXX'),
    'T': ('XXX', '.X.', '.X.', '.X.', '.X.'),
    'W': ('X.X', 'X.X', 'X.X', 'XXX', 'X.X'),
    '!': ('.X.', '.X.', '.X.', '...', '.X.'),
}

# 5 x 7 glyphs for full-size numbers and words.
_LARGE = {
    '0': ('.XXX.', 'X...X', 'X..XX', 'X.X.X', 'XX..X', 'X...X', '.XXX.'),
    '1': ('..X..', '.XX..', '..X..', '..X..', '..X..', '..X..', '.XXX.'),
    '2': ('.XXX.', 'X...X', '....X', '...X.', '..X..', '.X...', 'XXXXX'),
    '3': ('XXXX.', '....X', '....X', '.XXX.', '....X', '....X', 'XXXX.'),
    '4': ('...X.', '..XX.', '.X.X.', 'X..X.', 'XXXXX', '...X.', '...X.'),
    '5': ('XXXXX', 'X....', 'XXXX.', '....X', '....X', 'X...X', '.XXX.'),
    '6': ('..XX.', '.X...', 'X....', 'XXXX.', 'X...X', 'X...X', '.XXX.'),
    '7': ('XXXXX', '....X', '...X.', '..X..', '.X...', '.X...', '.X...'),
    '8': ('.XXX.', 'X...X', 'X...X', '.XXX.', 'X...X', 'X...X', '.XXX.'),
    '9': ('.XXX.', 'X...X', 'X...X', '.XXXX', '....X', '...X.', '.XX..'),
    'D': ('XXXX.', 'X...X', 'X...X', 'X...X', 'X...X', 'X...X', 'XXXX.'),
    'H': ('X...X', 'X...X', 'X...X', 'XXXXX', 'X...X', 'X...X', 'X...X'),
    'N': ('X...X', 'XX..X', 'X.X.X', 'X..XX', 'X...X', 'X...X', 'X...X'),
    'O': ('.XXX.', 'X...X', 'X...X', 'X...X', 'X...X', 'X...X', '.XXX.'),
    'S': ('.XXXX', 'X....', 'X....', '.XXX.', '....X', '....X', 'XXXX.'),
    'T': ('XXXXX', '..X..', '..X..', '..X..', '..X..', '..X..', '..X..'),
    'U': ('X...X', 'X...X', 'X...X', 'X...X', 'X...X', 'X...X', '.XXX.'),
    'W': ('X...X', 'X...X', 'X...X', 'X.X.X', 'X.X.X', 'XX.XX', 'X...X'),
}


def _join(glyphs: Mapping[str, tuple[str, ...]], text: str) -> list[str]:
    """Return the rows of `text` in `glyphs`, one blank column between characters."""
    height = len(next(iter(glyphs.values())))
    return ['.'.join(glyphs[ch][r] for ch in text) for r in range(height)]


def rows_to_frame(rows: tuple[str, ...] | list[str]) -> bytes:
    """Return the 192-byte dot matrix frame for 8 rows of 'X' (on) and '.' (off)."""
    return bytes(
        1 if c < len(row) and row[c] == 'X' else 0
        for row in (*rows, *[''] * (ROWS - len(rows)))
        for c in range(COLS)
    )


def large_rows(text: str) -> list[str]:
    """Return `text` in 5 x 7 glyphs, centered, below one blank row."""
    glyph_rows = _join(_LARGE, text)
    left = (COLS - len(glyph_rows[0])) // 2
    return ['.' * COLS] + [('.' * left + r).ljust(COLS, '.') for r in glyph_rows]


def number_rows(n: int) -> list[str]:
    return large_rows(str(max(0, n)))


def battery_rows(text: str) -> list[str]:
    """Return the battery outline with `text` in 3 x 5 letters centered inside it."""
    rows = [_OUTLINE_TOP, _OUTLINE_SIDE]
    rows += [_OUTLINE_SIDE_TERMINAL] * 4 + [_OUTLINE_SIDE, _OUTLINE_TOP]
    glyph_rows = _join(_SMALL, text)
    left = _INSIDE_LEFT + (_INSIDE_WIDTH - len(glyph_rows[0])) // 2
    for r, glyph_row in enumerate(glyph_rows, start=2):
        row = list(rows[r])
        for c, ch in enumerate(glyph_row):
            if ch == 'X':
                row[left + c] = 'X'
        rows[r] = ''.join(row)
    return rows


def seconds_left(shutdown_at: float | None, now: float) -> int:
    return 0 if shutdown_at is None else max(0, math.ceil(shutdown_at - now))


def dot_frame(level: str, shutdown_at: float | None, now: float) -> bytes | None:
    """Return the dot matrix frame for `level` at `now`, or None when there is no alert."""
    warn_word = WARN_CYCLE[math.floor(now / FRAME_SEC) % len(WARN_CYCLE)]
    if level == 'warn':
        return rows_to_frame(battery_rows(warn_word))
    if level == 'countdown':
        left = seconds_left(shutdown_at, now)
        if left > NUMBERS_FROM:
            return rows_to_frame(battery_rows(warn_word))
        if left in WORD_SECONDS:
            return rows_to_frame(battery_rows(WORD_SECONDS[left]))
        return rows_to_frame(number_rows(left))
    if level == 'off':
        since = 0.0 if shutdown_at is None else max(0.0, now - shutdown_at)
        index = min(int(since // FRAME_SEC), len(SHUTDOWN_WORDS) - 1)
        return rows_to_frame(large_rows(SHUTDOWN_WORDS[index]))
    return None


def led_frame(level: str, shutdown_at: float | None, now: float) -> bytes | None:
    """Return the LED frame for `level` at `now`, or None when there is no alert."""
    if level not in LED_SWAPS_PER_SEC:
        return None
    rate = LED_SWAPS_PER_SEC[level]
    if level == 'countdown' and seconds_left(shutdown_at, now) <= FINAL_SECONDS:
        rate = LED_SWAPS_PER_SEC['final']
    phase = math.floor(now * rate) % 2
    color = bytes(ORANGE if phase == 0 else RED)
    dark = b'\x00\x00\x00'
    return b''.join(color if (i // SEGMENT) % 2 == phase else dark for i in range(LED_COUNT))
