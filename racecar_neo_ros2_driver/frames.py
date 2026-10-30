"""
Frame offsets: load config/frames.yaml with the car profile's override, convert units.

frames.yaml keys carry their unit (`_mm` or `_deg`). The URDF takes meters and
radians, under the key name with the unit suffix removed.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from .car_profile import param_override

FRAMES_YAML = 'frames.yaml'
_UNITS = {'_mm': 1e-3, '_deg': math.pi / 180.0}


def _read(path: Path) -> dict[str, float]:
    data: Any = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping of key: value')
    for key, value in data.items():
        if not any(key.endswith(suffix) for suffix in _UNITS):
            raise ValueError(f'{path}: {key} has no unit suffix (_mm or _deg)')
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ValueError(f'{path}: {key} is not a number: {value!r}')
    return {k: float(v) for k, v in data.items()}


def load_frames(config_dir: str | Path) -> dict[str, float]:
    """
    Return frames.yaml merged with the profile's params/frames.yaml, in file units.

    An override key absent from frames.yaml is an error, so a typo cannot pass
    silently as an unused override.
    """
    config_dir = Path(config_dir)
    values = _read(config_dir / FRAMES_YAML)
    local = param_override('frames')
    if local is not None:
        overrides = _read(local)
        unknown = sorted(set(overrides) - set(values))
        if unknown:
            raise ValueError(f'{local}: keys not in {FRAMES_YAML}: {", ".join(unknown)}')
        values.update(overrides)
    return values


def xacro_mappings(values: dict[str, float]) -> dict[str, str]:
    """Return xacro arguments in meters and radians, keyed without the unit suffix."""
    mappings = {}
    for key, value in values.items():
        suffix = next(s for s in _UNITS if key.endswith(s))
        mappings[key[: -len(suffix)]] = format(value * _UNITS[suffix], '.10g')
    return mappings


# -- the lidar mount, for anything that turns scan angles into the car's --


def current() -> dict[str, float] | None:
    """Return the frames the running stack loads, or None when they cannot be read."""
    from .paths import config_dir

    try:
        return load_frames(config_dir())
    except (OSError, ValueError):
        return None


def laser_mount(values: dict[str, float] | None = None) -> tuple[float, float, float]:
    """
    Return the lidar's (x m, y m, yaw rad) in base_link.

    A ray at raw scan angle r points at r + yaw in base_link (counterclockwise
    from the nose). With no frames, the shipped mount: facing aft at the origin.
    """
    v = values if values is not None else (current() or {})
    return (
        v.get('laser_x_mm', 0.0) / 1000.0,
        v.get('laser_y_mm', 0.0) / 1000.0,
        math.radians(v.get('laser_yaw_deg', 180.0)),
    )


def student_deg(raw_rad: float, yaw_rad: float) -> float:
    """Return the lab angle of a ray: 0 at the nose, positive to the car's right, -180..180."""
    deg = -math.degrees(raw_rad + yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0


def raw_rad(student: float, yaw_rad: float) -> float:
    """Return the raw scan angle, -pi to pi, that points at lab angle `student` (degrees)."""
    raw = -math.radians(student) - yaw_rad
    return math.atan2(math.sin(raw), math.cos(raw))


def student_scan(msg: Any, yaw_rad: float, step: int = 3) -> list[list[float]]:
    """Return every `step`-th in-range ray as [lab angle deg, range m], for a top-down view."""
    return [
        [round(student_deg(msg.angle_min + i * msg.angle_increment, yaw_rad), 1), round(r, 3)]
        for i in range(0, len(msg.ranges), step)
        if msg.range_min < (r := msg.ranges[i]) < msg.range_max
    ]
