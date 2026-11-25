"""
One mission's bag, recorded through `racecar log` for the Planner.

Only a recording this object started is ever stopped: one the user started
with `racecar log start` is left alone, and then the mission is not recorded.
`racecar log` runs as a child, so the recorder itself belongs to init and a
finished one does not linger as a zombie that still looks alive. No ROS here.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any

import racecar_log

SCRIPT = Path(__file__).resolve().parent / 'racecar_log.py'
READY_SEC = 3.0
START_TIMEOUT_SEC = 15.0
STOP_TIMEOUT_SEC = 8.0
ANALYZE_TIMEOUT_SEC = 60.0
DAY_SEC = 86400.0


class Recorder:
    def __init__(self, run: Callable[..., Any] = subprocess.run) -> None:
        self._run = run
        self.folder: Path | None = None
        self._log: Path | None = None
        self._topics: list[str] = []

    def _cli(self, *args: str, timeout: float) -> bool:
        try:
            done = self._run([sys.executable, str(SCRIPT), *args], capture_output=True,
                             text=True, timeout=timeout)  # fmt: skip
        except (OSError, subprocess.TimeoutExpired):
            return False
        return done.returncode == 0

    def start(self, name: str, topics: list[str], min_free: float) -> bool:
        """Start recording; False when it cannot (no space, one running, no drive)."""
        self.folder = self._log = None
        if not topics or racecar_log.active_state() is not None:
            return False
        if not self._cli('start', '--name', name, '--topics', *topics, timeout=START_TIMEOUT_SEC):
            return False
        state = racecar_log.active_state()
        if state is None or Path(state['bag']).name != name:
            return False
        self.folder, self._log = Path(state['bag']), Path(state['stdout'])
        self._topics = list(topics)
        if self.free_bytes() < min_free:
            self.stop()
            return False
        return True

    def free_bytes(self) -> float:
        if self.folder is None:
            return 0.0
        try:
            return float(shutil.disk_usage(self.folder.parent).free)
        except OSError:
            return 0.0

    def running(self) -> bool:
        """Return True while the recording this object started still runs."""
        state = racecar_log.active_state() if self.folder is not None else None
        return state is not None and Path(state['bag']) == self.folder

    def status(self, min_free: float) -> dict[str, Any] | None:
        """
        Return the recording's progress for the page, or None when it does not run.

        left is the time until free space reaches `min_free`, under a day only;
        topics is (subscribed, asked for).
        """
        state = racecar_log.active_state() if self.running() else None
        if state is None:
            return None
        now = racecar_log.progress(state)
        rate = now['rate_bytes_per_sec']
        left = max(0.0, now['free_bytes'] - min_free) / rate if rate > 0 else None
        have = self._subscribed(self._topics)
        return {
            'time': racecar_log.format_duration(now['elapsed_sec']),
            'size': racecar_log.format_size(now['size_bytes']),
            'rate': f'{racecar_log.format_size(rate)}/s',
            'free': racecar_log.format_size(now['free_bytes']),
            'left': None if left is None or left >= DAY_SEC else racecar_log.format_duration(left),
            'topics': [have, len(self._topics)],
        }

    def analyze(self, folder: Path) -> str:
        """Return what `racecar log analyze` prints for a bag."""
        try:
            done = self._run(
                [sys.executable, str(SCRIPT), 'analyze', str(folder)],
                capture_output=True,
                text=True,
                timeout=ANALYZE_TIMEOUT_SEC,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ''
        return (done.stdout or done.stderr or '').strip('\n')

    def _subscribed(self, topics: list[str]) -> int:
        try:
            text = self._log.read_text(errors='replace') if self._log else ''
        except OSError:
            return 0
        return sum(f"Subscribed to topic '{t}'" in text for t in topics)

    def subscribed(self, topics: list[str]) -> bool:
        return self._subscribed(topics) == len(topics)

    def wait_ready(
        self, topics: list[str], give_up: Callable[[], bool], limit: float = READY_SEC
    ) -> bool:
        """Wait until the recorder has subscribed to the topics, at most `limit` seconds."""
        end = time.monotonic() + limit
        while time.monotonic() < end and not give_up():
            if self.subscribed(topics):
                return True
            time.sleep(0.05)
        return self.subscribed(topics)

    def stop(self) -> None:
        """Finish the bag; nothing when this object's recording is not the one running."""
        if self.running():
            self._cli('stop', '--timeout', str(STOP_TIMEOUT_SEC), timeout=STOP_TIMEOUT_SEC + 6)
