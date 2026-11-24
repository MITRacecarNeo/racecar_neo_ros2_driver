"""
Planner missions: waypoints on a saved map, kept beside the map.

A mission is `<map folder>/missions/<name>.yaml` (maps.map_stem gives the
folder), so it lists only with its map and moves with it when the map is
replaced or extended in place. Saving over a mission moves the old file to
`missions/previous/` first; nothing is deleted.

    version: 1
    loops: 1            # 1 to 99; 0 runs until Stop
    waypoints:
      - x: 1.2          # m, map frame
        y: 3.4
        heading_deg: 90 # null: any heading
        radius_m: 0.25  # reached within this distance
        speed_mps: null # null: full speed; else 0.5 to 1.0
        dwell_sec: 0    # wait here before the next waypoint

Used by scripts/planner_app.py (the Planner) and scripts/racecar_nav.py.
"""

from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
from typing import Any

import yaml

from .maps import map_root, map_stem, valid_name

VERSION = 1
FOLDER = 'missions'
PREVIOUS = 'previous'
LOOPS_MAX = 99
MAX_WAYPOINTS = 100
RADIUS_M = (0.25, 3.0)  # 0.25 is Nav2's goal tolerance (config/nav2.yaml)
SPEED_MPS = (0.5, 1.0)  # the firmware's speed band
DWELL_SEC = (0.0, 600.0)
DEFAULT_RADIUS_M = 0.25


def _number(value: Any, low: float, high: float, what: str) -> float:
    number = float(value)
    if not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f'{what} must be {low:g} to {high:g}')
    return number


def _coordinate(value: Any, what: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{what} must be a number')
    return number


def parse_waypoint(raw: Any) -> dict[str, Any]:
    """Return a waypoint with every field, checked against the limits; ValueError if not."""
    if not isinstance(raw, dict):
        raise ValueError('a waypoint is a mapping')
    heading = raw.get('heading_deg')
    if heading is not None:
        heading = round(_coordinate(heading, 'heading') % 360.0, 1)
    speed = raw.get('speed_mps')
    return {
        'x': round(_coordinate(raw['x'], 'x'), 3),
        'y': round(_coordinate(raw['y'], 'y'), 3),
        'heading_deg': heading,
        'radius_m': round(_number(raw.get('radius_m', DEFAULT_RADIUS_M), *RADIUS_M, 'radius'), 2),
        'speed_mps': None if speed is None else round(_number(speed, *SPEED_MPS, 'speed'), 2),
        'dwell_sec': round(_number(raw.get('dwell_sec', 0), *DWELL_SEC, 'wait'), 1),
    }


def parse(raw: Any) -> dict[str, Any]:
    """Return a mission with every field, checked against the limits; ValueError if not."""
    if not isinstance(raw, dict):
        raise ValueError('a mission is a mapping')
    loops = raw.get('loops', 1)
    if isinstance(loops, bool) or not isinstance(loops, int) or not 0 <= loops <= LOOPS_MAX:
        raise ValueError(f'loops must be 0 to {LOOPS_MAX}')
    points = raw.get('waypoints')
    if not isinstance(points, list) or not 1 <= len(points) <= MAX_WAYPOINTS:
        raise ValueError(f'a mission has 1 to {MAX_WAYPOINTS} waypoints')
    return {'version': VERSION, 'loops': loops, 'waypoints': [parse_waypoint(p) for p in points]}


def folder(map_name: str, root: Path | None = None) -> Path:
    if not valid_name(map_name):
        raise ValueError('not a map name')
    return map_stem(map_name, root or map_root()).parent / FOLDER


def _file(map_name: str, name: str, root: Path | None) -> Path:
    if not valid_name(name):
        raise ValueError('not a mission name')
    return folder(map_name, root) / f'{name}.yaml'


def listing(map_name: str, root: Path | None = None) -> list[dict[str, Any]]:
    """Return the map's missions, newest first: name, waypoints, loops, saved (local time)."""
    out = []
    where = folder(map_name, root)
    for path in where.glob('*.yaml') if where.is_dir() else []:
        try:
            mission = parse(yaml.safe_load(path.read_text()))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
            continue
        stamp = path.stat().st_mtime
        out.append({
            'name': path.stem,
            'waypoints': len(mission['waypoints']),
            'loops': mission['loops'],
            'saved': datetime.fromtimestamp(stamp).strftime('%Y-%m-%d %H:%M'),
            'stamp': stamp,
        })
    out.sort(key=lambda m: -m['stamp'])
    for m in out:
        del m['stamp']
    return out


def load(map_name: str, name: str, root: Path | None = None) -> dict[str, Any]:
    """Read a saved mission; FileNotFoundError if there is none."""
    return parse(yaml.safe_load(_file(map_name, name, root).read_text()))


def exists(map_name: str, name: str, root: Path | None = None) -> bool:
    return _file(map_name, name, root).is_file()


def save(map_name: str, name: str, mission: dict[str, Any], root: Path | None = None) -> Path:
    """Write a mission; a mission already saved under the name moves to previous/ first."""
    path = _file(map_name, name, root)
    text = yaml.safe_dump(parse(mission), sort_keys=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file():
        older = path.parent / PREVIOUS
        older.mkdir(exist_ok=True)
        path.replace(older / f'{name}-{datetime.now().strftime("%Y%m%d-%H%M%S")}.yaml')
    part = path.with_suffix('.part')
    part.write_text(text)
    part.replace(path)
    return path
