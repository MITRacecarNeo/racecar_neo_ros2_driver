"""
Stall protection state from the Teensy firmware, decoded by pit_node.

Firmware with stall protection (info key fw.stall_protect=1) reports in the
telemetry EKF slots:

- EKF[0]: state, 0 normal, 1 tripped (output at neutral until the command or
  stick returns to neutral), 2 cooldown, 3 lockout (after lockout_trips trips
  in a window). The car drives again only from 0.
- EKF[1]: seconds left in the cooldown or lockout, else 0.
- EKF[2]: trips since the Teensy booted.

Older firmware sends zeros, which read as normal with no trips. pit_node
publishes a change on /pit/stall (latched DiagnosticStatus) and the state on
/diagnostics once a second.

No ROS dependency, so it is unit-testable.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Any

STATUS_NAME = 'pit: stall'
NAMES = {0: 'normal', 1: 'tripped', 2: 'cooldown', 3: 'lockout'}
OK, WARN, ERROR = 0, 1, 2
LEVELS = {0: OK, 1: WARN, 2: WARN, 3: ERROR}
# Seconds left are published in these steps, so a countdown does not publish
# at the 200 Hz telemetry rate.
SECONDS_STEP = 0.1


@dataclass(frozen=True)
class StallState:
    state: int = 0
    seconds_left: float = 0.0
    trips: int = 0

    @property
    def name(self) -> str:
        return NAMES.get(self.state, f'state {self.state}')

    def values(self) -> dict[str, str]:
        return {
            'state': str(self.state),
            'name': self.name,
            'seconds_left': f'{self.seconds_left:.1f}',
            'trips': str(self.trips),
        }


def decode(ekf: Sequence[float]) -> StallState:
    """Read the EKF slots; anything not finite or out of range reads as 0."""

    def whole(x: float) -> int:
        return int(round(x)) if math.isfinite(x) and 0 <= x < 1e6 else 0

    seconds = ekf[1] if len(ekf) > 1 and math.isfinite(ekf[1]) and 0 < ekf[1] < 1e6 else 0.0
    return StallState(
        state=whole(ekf[0]) if ekf else 0,
        seconds_left=round(seconds / SECONDS_STEP) * SECONDS_STEP,
        trips=whole(ekf[2]) if len(ekf) > 2 else 0,
    )


class StallWatch:
    """
    The last decoded state; `update()` says when it changed.

    The first frame always counts as a change, so the latched topic holds a
    state from the start.
    """

    def __init__(self) -> None:
        self._seen: StallState | None = None

    @property
    def last(self) -> StallState:
        return self._seen if self._seen is not None else StallState()

    def update(self, ekf: Sequence[float]) -> StallState | None:
        """Decode a frame; return the new state when it differs from the last."""
        state = decode(ekf)
        if state == self._seen:
            return None
        self._seen = state
        return state

    def report(self, supported: bool | None) -> dict[str, Any]:
        """
        Return the status as a DiagnosticStatus in dictionary form.

        `supported` is the firmware's fw.stall_protect: True, False (firmware
        without it), or None (firmware info not read yet).
        """
        s = self.last
        values = {'supported': {True: '1', False: '0', None: ''}[supported], **s.values()}
        if supported is False:
            return {'level': OK, 'message': 'Not in this firmware', 'values': values}
        message = {
            0: 'Normal',
            1: 'Tripped: return the throttle to neutral',
            2: f'Cooldown, {s.seconds_left:.0f} s',
            3: f'Lockout, {s.seconds_left:.0f} s',
        }.get(s.state, s.name)
        if s.trips:
            message += f'; {s.trips} trip{"s" if s.trips != 1 else ""} since boot'
        return {'level': LEVELS.get(s.state, WARN), 'message': message, 'values': values}
