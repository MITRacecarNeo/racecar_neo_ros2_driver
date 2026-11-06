"""
Firmware info reply, requested and collected by pit_node.

pit_node sets the info_request bit in its command frames from the moment the
serial port opens until the whole payload is in or REPLY_TIMEOUT_SEC passes.
The Teensy answers with info chunks between telemetry frames (pit_protocol
InfoChunk); telemetry is unaffected. A Teensy reset or a serial reconnect
starts a new request, since a reflash changes the firmware. Firmware before
0.9.0 never answers, so its info stays unknown.

The payload is `key=value` lines (racecar-pit-firmware docs/protocol.md,
"Info keys"). Values are kept as text; unknown keys are kept and ignored.

No ROS dependency, so it is unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
from typing import Any

from .pit_protocol import InfoChunk

REPLY_TIMEOUT_SEC = 2.0
STATUS_NAME = 'pit: firmware'

OK, WARN = 0, 1
PENDING, KNOWN, UNKNOWN = 'pending', 'known', 'unknown'

CLOSED_LOOP = 'closed_loop'
# Test builds, where /drive no longer means a speed setpoint.
TEST_BUILDS = {
    'raw_pwm': 'open-loop PWM test build',
    'esc_cal': 'ESC calibration build',
}

# Keys copied onto /diagnostics; the whole payload goes on the firmware topic.
SHOWN_KEYS = (
    'fw.version',
    'fw.label',
    'fw.git',
    'fw.git_dirty',
    'fw.speed_control',
    'fw.stall_protect',
    'fw.profile',
    'esc.pwm_min_us',
    'esc.pwm_neutral_us',
    'esc.pwm_max_us',
    'power.max_current_a',
)


def parse_info(payload: bytes) -> dict[str, str]:
    """Return the payload's `key=value` lines as a dict; lines without `=` are skipped."""
    info: dict[str, str] = {}
    for line in payload.decode('ascii', errors='replace').splitlines():
        key, sep, value = line.partition('=')
        if sep and key:
            info[key] = value
    return info


@dataclass
class InfoAssembler:
    """
    Collect chunks until one revision of the payload is complete.

    A chunk of another revision, chunk count or payload size restarts the
    collection, so a payload never mixes two revisions.
    """

    _key: tuple[int, int, int] | None = None
    _parts: dict[int, bytes] = field(default_factory=dict)

    def add(self, chunk: InfoChunk) -> bytes | None:
        """Add a chunk; return the payload once every part of it is in."""
        key = (chunk.revision, chunk.count, chunk.payload_bytes)
        if key != self._key:
            self._key, self._parts = key, {}
        self._parts[chunk.index] = chunk.data
        if len(self._parts) < chunk.count:
            return None
        payload = b''.join(self._parts[i] for i in range(chunk.count))
        if len(payload) != chunk.payload_bytes:
            self._key, self._parts = None, {}
            return None
        return payload


class FirmwareInfo:
    """
    Request state and the last payload; every method takes the time, so it is testable.

    The command timer asks `requesting()` and the serial reader calls `add()`
    and `restart()`, so the state sits behind a lock.
    """

    def __init__(self, now: float, timeout_sec: float = REPLY_TIMEOUT_SEC) -> None:
        self.timeout = timeout_sec
        self._lock = threading.Lock()
        self.restart(now)

    def restart(self, now: float) -> None:
        """Forget the current info and request it again."""
        with self._lock:
            self.state = PENDING
            self.values: dict[str, str] = {}
            self.payload = b''
            self._since = now
            self._assembler = InfoAssembler()

    def requesting(self, now: float) -> bool:
        """Return True while the info_request bit belongs in the command frames."""
        with self._lock:
            if self.state == PENDING and now - self._since > self.timeout:
                self.state = UNKNOWN
            return self.state == PENDING

    def add(self, chunk: InfoChunk) -> bool:
        """Add a chunk; return True when it completed the payload."""
        with self._lock:
            if self.state != PENDING:
                return False
            payload = self._assembler.add(chunk)
            if payload is None:
                return False
            self.payload = payload
            self.values = parse_info(payload)
            self.state = KNOWN
            return True

    def summary(self) -> str:
        """One line for the log: version, label, commit and speed control."""
        v = self.values
        parts = [v.get(k) for k in ('fw.label', 'fw.git', 'fw.speed_control')]
        return f'{v.get("fw.version", "?")} ({", ".join(p for p in parts if p)})'

    def report(self) -> dict[str, Any]:
        """Return the status as a DiagnosticStatus in dictionary form."""
        with self._lock:
            state, v = self.state, dict(self.values)
        values = {'state': state, **{k: v[k] for k in SHOWN_KEYS if k in v}}
        if state == PENDING:
            return {'level': OK, 'message': 'Reading firmware info', 'values': values}
        if state == UNKNOWN:
            return {
                'level': WARN,
                'message': 'No info reply (firmware before 0.9.0)',
                'values': values,
            }
        version = v.get('fw.version', '?')
        control = v.get('fw.speed_control', CLOSED_LOOP)
        if control != CLOSED_LOOP:
            build = TEST_BUILDS.get(control, f'speed control {control}')
            return {'level': WARN, 'message': f'{version}: {build}', 'values': values}
        if v.get('fw.git_dirty') == '1':
            return {'level': WARN, 'message': f'{version}: uncommitted build', 'values': values}
        return {'level': OK, 'message': version, 'values': values}
