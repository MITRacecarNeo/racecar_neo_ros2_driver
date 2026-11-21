"""
Calibration values against their expected ranges (config/calibration_expected.yaml).

`verdicts(values)` rates each value OK, WARN or FAIL; `require(values)` raises
NotSaved when any is FAIL, so the writer stores nothing.
"""

from __future__ import annotations

from functools import cache
from pathlib import Path
from typing import Any

import yaml

EXPECTED = Path(__file__).resolve().parents[2] / 'config' / 'calibration_expected.yaml'
OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'


class NotSaved(ValueError):
    """A calibration value is outside its expected range; nothing was written."""


@cache
def ranges(path: Path = EXPECTED) -> dict[str, dict[str, Any]]:
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except OSError:
        return {}
    return {str(k): dict(v) for k, v in data.items() if isinstance(v, dict)}


def _inside(value: float, bounds: list[float] | None) -> bool:
    return bounds is None or bounds[0] <= value <= bounds[1]


def rate(key: str, value: Any) -> dict[str, Any] | None:
    """Return {key, value, verdict, fail, warn, spread} for a value with a range, else None."""
    rule = ranges().get(key)
    if rule is None:
        return None
    items = value if isinstance(value, list) else [value]
    try:
        numbers = [float(v) for v in items]
    except (TypeError, ValueError):
        return None
    verdict = OK
    if not all(_inside(v, rule.get('warn')) for v in numbers):
        verdict = WARN
    if not all(_inside(v, rule.get('fail')) for v in numbers):
        verdict = FAIL
    return {
        'key': key,
        'value': value,
        'verdict': verdict,
        'fail': rule.get('fail'),
        'warn': rule.get('warn'),
        'spread': rule.get('spread'),
    }


def verdicts(values: dict[str, Any]) -> list[dict[str, Any]]:
    return [r for k, v in values.items() if (r := rate(k, v)) is not None]


def require(values: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the verdicts; raise NotSaved naming each value outside its range."""
    rated = verdicts(values)
    bad = [r for r in rated if r['verdict'] == FAIL]
    if bad:
        names = ', '.join(
            f"{r['key']} {r['value']} (expected {r['fail'][0]} to {r['fail'][1]})" for r in bad
        )
        raise NotSaved(f'outside the expected range, not saved: {names}')
    return rated
