"""
Teensy UART link statistics, counted by pit_node's serial reader.

The firmware sends a telemetry frame every FRAME_US. Counted since pit_node
started:

- frames: decoded frames, CRC good or not
- crc_failures: frames whose CRC did not match (counted whether or not
  require_crc drops them)
- dropped_frames: frames missing between two decoded ones, from the gap in
  the Teensy's own timestamps; a lost or corrupted frame shows here
- skipped_bytes: bytes thrown away while finding the next frame (noise, a
  frame cut short, the partial frame at startup)
- reconnects: serial port reopens after a read error
- resets: Teensy restarts, seen as its timestamp going back

Each report also carries the last second: frame rate and the longest gap
between frames by the Teensy's clock. The report is a DiagnosticStatus in
dictionary form; pit_node publishes it on /diagnostics once a second from the
reader thread.

No ROS dependency, so it is unit-testable.
"""

from __future__ import annotations

from typing import Any

FRAME_US = 5000
REPORT_SEC = 1.0
STATUS_NAME = 'pit: teensy link'
# The same key the RealSense driver uses for its stream rates, so the rate
# readers (scripts/sysinfo.py DIAGNOSTIC_RATE_NAMES) take this one too.
RATE_KEY = 'Actual frequency (Hz)'
# The firmware drops the wheels to neutral after this long without a command;
# a telemetry gap this long means the link stalled in both directions.
STALL_MS = 50.0

OK, WARN, ERROR = 0, 1, 2

_WRAP = 1 << 32
COUNTERS = ('frames', 'crc_failures', 'dropped_frames', 'skipped_bytes', 'reconnects', 'resets')


class LinkStats:
    """Counters and the 1 s window; every method takes the time, so it is testable."""

    def __init__(self, now: float, frame_us: int = FRAME_US) -> None:
        self.frame_us = frame_us
        self.counts = dict.fromkeys(COUNTERS, 0)
        self.longest_gap_ms = 0.0
        self._last_raw: int | None = None
        self._window_start = now
        self._window_frames = 0
        self._window_gap_ms = 0.0
        self._window_problems = 0

    def frame(self, timestamp_us: int, crc_ok: bool) -> None:
        self.counts['frames'] += 1
        self._window_frames += 1
        if not crc_ok:
            self.counts['crc_failures'] += 1
            self._window_problems += 1
        raw = timestamp_us & (_WRAP - 1)
        if self._last_raw is not None:
            step = (raw - self._last_raw) % _WRAP
            if step >= _WRAP // 2:
                self.counts['resets'] += 1
                self._window_problems += 1
            elif step:
                gap_ms = step / 1000.0
                missing = round(step / self.frame_us) - 1
                if missing > 0:
                    self.counts['dropped_frames'] += missing
                    self._window_problems += 1
                self._window_gap_ms = max(self._window_gap_ms, gap_ms)
                self.longest_gap_ms = max(self.longest_gap_ms, gap_ms)
        self._last_raw = raw

    def skipped(self, count: int) -> None:
        if count > 0:
            self.counts['skipped_bytes'] += count

    def reconnect(self) -> None:
        self.counts['reconnects'] += 1
        self._window_problems += 1
        # The next frame's timestamp is not comparable with the last one.
        self._last_raw = None

    def due(self, now: float) -> bool:
        return now - self._window_start >= REPORT_SEC

    def report(self, now: float) -> dict[str, Any]:
        """Return {level, message, values} for the window ending now, and start a new window."""
        span = max(now - self._window_start, 1e-6)
        rate = self._window_frames / span
        if self._window_frames == 0:
            level, message = ERROR, 'no frames'
        elif self._window_problems or self._window_gap_ms > STALL_MS:
            level, message = WARN, f'{rate:.0f} Hz, frames lost'
        else:
            level, message = OK, f'{rate:.0f} Hz'
        values = {
            RATE_KEY: f'{rate:.1f}',
            'gap_ms': f'{self._window_gap_ms:.1f}',
            'longest_gap_ms': f'{self.longest_gap_ms:.1f}',
            **{key: str(n) for key, n in self.counts.items()},
        }
        self._window_start = now
        self._window_frames = 0
        self._window_gap_ms = 0.0
        self._window_problems = 0
        return {'level': level, 'message': message, 'values': values}
