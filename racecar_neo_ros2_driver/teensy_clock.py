"""
Map the Teensy's micros() telemetry timestamp onto ROS time.

pit_node stamps telemetry when its serial reader parses a frame, and the link
delivers frames in bursts: receive times jitter by up to 17 ms (99th
percentile) against a 6 ms frame period. Each frame also carries the
Teensy's micros() at assembly, which is exact. TeensyClock turns that into a
ROS time:

- Unwrap: the field is a signed 32-bit microsecond count and wraps every
  35.8 min (2^31 us); it is extended to a 64-bit count.
- Offset: receive time minus Teensy time is the transport delay plus a
  constant. The minimum over a sliding window (WINDOW_SEC) is the
  least-delayed frame, so stamp = Teensy time + that minimum.
- Drift: the two clocks differ by a few ppm, under a microsecond across the
  window; the sliding minimum follows it. Stamps never move backwards.
- Restart: when the Teensy time and the receive time advance by different
  amounts (a Teensy reset, a serial reconnect, a Pi clock step), the mapping
  starts over from the next frame.

No ROS dependency, so it is unit-testable; times are integer nanoseconds.
"""

from __future__ import annotations

from collections import deque

WINDOW_SEC = 10.0
# Teensy and receive clocks may disagree on the time between two frames by
# transport jitter (tens of ms); more than this is a reset or a clock step.
RESTART_TOLERANCE_SEC = 0.5

_NS_PER_US = 1000
_WRAP = 1 << 32


class TeensyClock:
    """Stateful mapping from telemetry timestamps to receive-clock nanoseconds."""

    def __init__(
        self, window_sec: float = WINDOW_SEC, restart_tolerance_sec: float = RESTART_TOLERANCE_SEC
    ) -> None:
        self._window_ns = int(window_sec * 1e9)
        self._tolerance_ns = int(restart_tolerance_sec * 1e9)
        self.restarts = 0
        self._reset_state()

    def _reset_state(self) -> None:
        self._last_raw: int | None = None
        self._teensy_ns = 0  # unwrapped Teensy time, ns
        self._last_receive_ns = 0
        self._last_stamp_ns = 0
        # (receive_ns, offset_ns), offsets increasing: the front is the window minimum.
        self._window: deque[tuple[int, int]] = deque()

    def stamp(self, timestamp_us: int, receive_ns: int) -> int:
        """Return the ROS time (ns) of a frame stamped `timestamp_us`, received `receive_ns`."""
        raw = timestamp_us & (_WRAP - 1)
        if self._last_raw is not None:
            step_ns = ((raw - self._last_raw) % _WRAP) * _NS_PER_US
            received_ns = receive_ns - self._last_receive_ns
            if abs(step_ns - received_ns) > self._tolerance_ns:
                self.restarts += 1
                self._reset_state()
            else:
                self._teensy_ns += step_ns
        self._last_raw = raw
        self._last_receive_ns = receive_ns

        offset = receive_ns - self._teensy_ns
        while self._window and self._window[-1][1] >= offset:
            self._window.pop()
        self._window.append((receive_ns, offset))
        while self._window[0][0] < receive_ns - self._window_ns:
            self._window.popleft()

        stamp = self._teensy_ns + self._window[0][1]
        stamp = max(stamp, self._last_stamp_ns)
        self._last_stamp_ns = stamp
        return stamp
