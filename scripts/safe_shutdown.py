"""
Safe shutdown of the car.

Stops recording, driving, autonomy, and teleop, then powers off.

Used by the battery guard (battery_guard.py) and by the Turn Off tile on the
8080 home page (dashboard.py). Both run as the racecar user;
polkit/49-racecar-power.rules grants the teleop stop and the power off, and
the other rules the lab apps and autonomy.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import subprocess
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent

DRIVING_UNITS = (
    'racecar-webteleop.service',
    'racecar-linefollow.service',
    'racecar-wallfollow.service',
)
AUTONOMY_UNIT = 'racecar-autonomy.service'
TELEOP_UNIT = 'racecar-teleop.service'
STEP_TIMEOUT_SEC = 30.0


def shutdown_sequence(
    run: Callable[..., Any] = subprocess.run,
    dry_run: bool = False,
    report: Callable[[str], None] = lambda line: None,
) -> list[str]:
    """
    Stop recording, driving, autonomy, and teleop, then power off.

    Each step is bounded and its failure does not stop the next; the car is
    turned off whatever happened. Each step's line goes to `report` as soon as
    it finishes, and all of them are returned.
    """
    steps: list[tuple[str, list[str]]] = [
        ('stop recording', [sys.executable, str(SCRIPT_DIR / 'racecar_log.py'), 'stop']),
        *[
            (f'stop {u}', ['systemctl', '--no-ask-password', 'stop', u])
            for u in (*DRIVING_UNITS, AUTONOMY_UNIT, TELEOP_UNIT)
        ],
        ('power off', ['systemctl', '--no-ask-password', '--check-inhibitors=no', 'poweroff']),
    ]
    lines = []
    for name, argv in steps:
        if dry_run:
            line = f'{name}: dry run, not run'
        else:
            try:
                r = run(argv, capture_output=True, text=True, timeout=STEP_TIMEOUT_SEC)
                ok = r.returncode == 0
                detail = '' if ok else f' ({(r.stderr or r.stdout or "").strip()[:200]})'
                line = f'{name}: {"ok" if ok else "failed"}{detail}'
            except (OSError, subprocess.TimeoutExpired) as exc:
                line = f'{name}: failed ({exc})'
        report(line)
        lines.append(line)
    return lines
