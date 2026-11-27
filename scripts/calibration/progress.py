"""
Progress events of a calibration run, for the Car Profile Calibration page.

When RACECAR_CAL_PROGRESS names a file, `emit` appends one JSON object per
line to it: the stage and its instruction, collection progress with a
stillness reading, plot data, and the results. Without it nothing is
written, so a run from the terminal looks the same as before.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any

PROGRESS_FILE = 'RACECAR_CAL_PROGRESS'
# Points kept per plot series; longer series are thinned evenly.
MAX_POINTS = 400


def enabled() -> bool:
    return bool(os.environ.get(PROGRESS_FILE))


def emit(kind: str, **data: Any) -> None:
    path = os.environ.get(PROGRESS_FILE)
    if not path:
        return
    line = json.dumps({'t': round(time.time(), 3), 'kind': kind, **data}, default=float)
    with open(path, 'a') as f:
        f.write(line + '\n')


def thin(points: list[Any], limit: int = MAX_POINTS) -> list[Any]:
    """Return at most `limit` points, evenly spaced, keeping the last."""
    if len(points) <= limit:
        return list(points)
    step = (len(points) - 1) / (limit - 1)
    return [points[round(i * step)] for i in range(limit)]


def spread(samples: list[list[float]]) -> list[float]:
    """Return the standard deviation of each column."""
    if len(samples) < 2:
        return [0.0] * (len(samples[0]) if samples else 0)
    columns = list(zip(*samples, strict=True))
    out = []
    for col in columns:
        mean = sum(col) / len(col)
        out.append(math.sqrt(sum((v - mean) ** 2 for v in col) / (len(col) - 1)))
    return out


def plot(
    title: str, series: list[dict[str, Any]], x: str, y: str, kind: str = 'line', **extra: Any
) -> None:
    """
    Emit one plot: series of {name, points: [[x, y], ...]}, axis labels.

    kind is line or scatter; extra keys (equal_axes, reference lines) pass
    through to the page.
    """
    emit('plot', title=title, plot=kind, x=x, y=y,
         series=[{**s, 'points': thin(s['points'])} for s in series], **extra)  # fmt: skip
