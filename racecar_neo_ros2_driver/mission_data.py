"""
What a Planner mission records: topic groups, the event log, and its summary.

A mission records only while its Record toggle is on (missions.py `record`).
The bag is a `racecar log` recording; beside it in the bag folder are
`mission.yaml`, the mission as it ran, and `events.jsonl`, one event a line:

    {"t": 1795550000.12, "event": "reached", "loop": 1, "waypoint": 2,
     "pose": [x, y, yaw], "dist_m": 3.4}

Events: recording (turned on during a mission), start, sent (x, y, yaw,
resend), planned (path), reached, skipped, paused (reason), resumed,
relocalizing, localized, edited (mission), arrived, stopped. `t` is the epoch
time, as in the bag; waypoint counts from 1; dist_m is the distance driven
since recording began.

Groups: config/mission_data.yaml and the car profile's params/mission_data.yaml.
"""

from __future__ import annotations

from collections.abc import Iterable
import json
from pathlib import Path
import time
from typing import Any

import yaml

from . import car_profile
from .paths import config_dir

CONFIG = 'mission_data'
EVENTS_FILE = 'events.jsonl'
MISSION_FILE = 'mission.yaml'
MIN_FREE_GB = 2.0
ENDINGS = ('arrived', 'stopped')


def load_config(folder: Path | None = None) -> dict[str, Any]:
    """Return the settings: the shipped file, then the car profile's groups by id."""
    path = (folder or config_dir()) / f'{CONFIG}.yaml'
    values = dict((car_profile.read_yaml(path) or {}).get(CONFIG) or {})
    groups = dict(values.get('groups') or {})
    local = car_profile.param_override(CONFIG)
    if local is not None:
        own = (car_profile.read_yaml(local) or {}).get(CONFIG) or {}
        groups.update(own.get('groups') or {})
        values.update({k: v for k, v in own.items() if k != 'groups'})
    values['groups'] = groups
    return values


def groups_view(config: dict[str, Any]) -> list[dict[str, str]]:
    """Return the groups for the page: id and name, in file order."""
    return [{'id': k, 'name': str(g.get('name') or k)} for k, g in config['groups'].items()]


def topics(config: dict[str, Any], ids: Iterable[str]) -> list[str]:
    """Return the topics of the groups, each once; an unknown group adds none."""
    out: list[str] = []
    for key in ids:
        for topic in (config['groups'].get(key) or {}).get('topics') or []:
            if topic not in out:
                out.append(str(topic))
    return out


def min_free_bytes(config: dict[str, Any]) -> float:
    return float(config.get('min_free_gb', MIN_FREE_GB)) * 1e9


def bag_name(label: str, now: float) -> str:
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now))
    return f'{stamp}_mission_{label}' if label else f'{stamp}_mission'


class EventLog:
    """
    events.jsonl in a bag folder.

    rosbag2 makes the folder itself and refuses one that exists, so events
    wait in memory until it does.
    """

    def __init__(self, folder: Path, mission: dict[str, Any]) -> None:
        self.folder = folder
        self._mission: dict[str, Any] | None = mission
        self._waiting: list[str] = []

    def add(self, event: str, now: float, **fields: Any) -> None:
        self._waiting.append(json.dumps({'t': round(now, 3), 'event': event, **fields}))
        self.flush()

    def flush(self) -> None:
        if not self.folder.is_dir():
            return
        if self._mission is not None:
            text = yaml.safe_dump(self._mission, sort_keys=False)
            (self.folder / MISSION_FILE).write_text(text)
            self._mission = None
        if self._waiting:
            with (self.folder / EVENTS_FILE).open('a') as f:
                f.write(''.join(line + '\n' for line in self._waiting))
            self._waiting = []


def read_events(folder: Path) -> list[dict[str, Any]]:
    """Return a bag's mission events, [] without a log; an unreadable line is left out."""
    out = []
    try:
        lines = (folder / EVENTS_FILE).read_text().splitlines()
    except OSError:
        return []
    for line in lines:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if isinstance(event, dict) and 't' in event and 'event' in event:
            out.append(event)
    return out


def summarize(events: list[dict[str, Any]]) -> dict[str, Any]:
    """Return the legs, loops, pauses and result of a mission's events."""
    legs: list[dict[str, Any]] = []
    pauses: list[dict[str, Any]] = []
    leg: dict[str, Any] | None = None
    pause: dict[str, Any] | None = None
    loops: dict[int, list[float]] = {}
    reached: set[int] = set()
    count, result, edits = 0, 'recording ended', 0

    def end_pause(t: float) -> None:
        nonlocal pause
        if pause is not None:
            pause['sec'] = round(t - pause.pop('t'), 1)
            pauses.append(pause)
            pause = None

    def end_leg(t: float, dist: float, how: str) -> None:
        nonlocal leg
        if leg is not None:
            leg['sec'] = round(t - leg.pop('t'), 1)
            leg['dist_m'] = round(dist - leg.pop('d'), 2)
            leg['result'] = how
            legs.append(leg)
            leg = None

    for e in events:
        t, kind, dist = e['t'], e['event'], float(e.get('dist_m') or 0.0)
        waypoint, loop = e.get('waypoint'), e.get('loop')
        if loop is not None:
            loops.setdefault(loop, [t, t])[1] = t
        if kind in ('start', 'recording', 'edited'):
            count = len((e.get('mission') or {}).get('waypoints') or []) or count
            edits += kind == 'edited'
        elif kind == 'sent':
            if leg is None or (leg['loop'], leg['waypoint']) != (loop, waypoint):
                end_leg(t, dist, 'left')
                leg = {'loop': loop, 'waypoint': waypoint, 'resends': 0, 't': t, 'd': dist}
            elif e.get('resend'):
                leg['resends'] += 1
        elif kind == 'reached':
            reached.add(waypoint)
            end_leg(t, dist, 'reached')
        elif kind == 'skipped':
            end_pause(t)
            end_leg(t, dist, 'skipped')
        elif kind == 'paused':
            end_pause(t)
            pause = {'cause': e.get('reason') or '', 'waypoint': waypoint, 't': t}
        elif kind == 'relocalizing':
            end_pause(t)
            pause = {'cause': 'localization lost', 'waypoint': waypoint, 't': t}
        elif kind in ('resumed', 'localized'):
            end_pause(t)
        elif kind in ENDINGS:
            end_pause(t)
            end_leg(t, dist, kind)
            result = kind
    if events:
        last = events[-1]
        end_pause(last['t'])
        end_leg(last['t'], float(last.get('dist_m') or 0.0), 'recording ended')
    return {
        'result': result,
        'duration_sec': round(events[-1]['t'] - events[0]['t'], 1) if events else 0.0,
        'distance_m': round(max((float(e.get('dist_m') or 0.0) for e in events), default=0.0), 2),
        'legs': legs,
        'loops': [{'loop': k, 'sec': round(b - a, 1)} for k, (a, b) in sorted(loops.items())],
        'pauses': pauses,
        'not_reached': [n for n in range(1, count + 1) if n not in reached],
        'edits': edits,
    }


CAUSES = {'user': 'paused', 'failed': 'could not reach', 'stalled': 'car stalled'}


def summary_lines(s: dict[str, Any]) -> list[str]:
    """Return the summary as `racecar log analyze` prints it."""
    out = [
        f"  mission   {s['result']}, {s['duration_sec']:.1f} s, {s['distance_m']:.2f} m",
    ]
    if s['legs']:
        out += ['', f"  {'loop':>4} {'waypoint':>9} {'sec':>7} {'m':>7} {'resends':>8}  result"]
        out += [
            f"  {g['loop'] or '':>4} {g['waypoint'] or '':>9} {g['sec']:>7.1f} "
            f"{g['dist_m']:>7.2f} {g['resends']:>8}  {g['result']}"
            for g in s['legs']
        ]
    if len(s['loops']) > 1:
        out += ['', *(f"  loop {g['loop']}: {g['sec']:.1f} s" for g in s['loops'])]
    if s['pauses']:
        out.append('')
        for p in s['pauses']:
            cause = CAUSES.get(p['cause'], p['cause'])
            out.append(f"  waypoint {p['waypoint']}: {cause}, {p['sec']:.1f} s")
    if s['not_reached']:
        out += ['', '  not reached: ' + ', '.join(str(n) for n in s['not_reached'])]
    return out
