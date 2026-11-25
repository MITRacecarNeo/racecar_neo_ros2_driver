"""
The navigator without ROS: one mission at a time for every front end.

`Run` steps through a mission (missions.py). `Commander` takes the commands of
the Planner, `racecar nav` and student programs, applies the reject rule and
the drive lease (drive_lease.py), and returns what to do. nav_server_node.py
carries the actions out: ('send', number, x, y, yaw), ('cancel',),
('limit', mps or None).

A command is a mapping: verb (start, check, pause, resume, skip, stop, edit,
log), owner, name (who is driving, for the messages), and for start and edit a
mission. A start while a mission runs, or while something else drives, is
refused; check asks without starting. The other verbs are always taken. `log`
(a key, '' for off) keeps the mission's events for the front end that sent it.
"""

from __future__ import annotations

from collections import Counter
import math
import threading
from typing import Any

from . import missions
from .drive_lease import DriveLease

YAW_TOLERANCE = 0.5  # rad, Nav2's yaw_goal_tolerance (config/nav2.yaml)
# Nav2 can report a new goal reached for the path to the one before it. Its word
# counts only with the car this close; else the waypoint is sent again, so often.
ARRIVE_MARGIN_M = 0.15
ARRIVE_MARGIN_RAD = 0.15
RETRY_LIMIT = 10
ACTIVE = ('running', 'dwelling', 'relocalizing')
# The car is held for the operator: a Drive panel may move it meanwhile.
SHARED = ('paused', 'relocalizing')
OWNER = 'nav'
NO_MAP = 'Load a map first.'
NOT_LOCALIZED = 'Wait until the car is localized.'
STALLED = 'Car stalled.'
UNKNOWN = 'Unknown command.'

Pose = tuple[float, float, float]
Action = tuple[Any, ...]


def angle_between(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def goal_yaw(wp: dict[str, Any], previous: tuple[float, float] | None, fallback: float) -> float:
    """Return the waypoint's heading, or for any heading the direction it is approached from."""
    if wp['heading_deg'] is not None:
        return math.radians(wp['heading_deg'])
    if previous is not None:
        dx, dy = wp['x'] - previous[0], wp['y'] - previous[1]
        if math.hypot(dx, dy) > 1e-3:
            return math.atan2(dy, dx)
    return fallback


def legs(mission: dict[str, Any], pose: Pose | None) -> list[tuple[Pose | None, Pose]]:
    """
    Return (start, goal) per leg for the preview.

    The first leg starts at the car (None). A mission that loops adds the leg
    from the last waypoint back to the first.
    """
    wps = mission['waypoints']
    out: list[tuple[Pose | None, Pose]] = []
    previous: Pose | None = None
    for i, wp in enumerate(wps):
        before = pose[:2] if i == 0 and pose else (previous[:2] if previous else None)
        goal = (wp['x'], wp['y'], goal_yaw(wp, before, pose[2] if pose else 0.0))
        out.append((previous, goal))
        previous = goal
    if mission['loops'] != 1 and len(wps) > 1:
        first = wps[0]
        out.append((previous, (first['x'], first['y'], goal_yaw(first, previous[:2], 0.0))))
    return out


class Run:
    """
    The mission in progress, without ROS.

    Each call returns the actions for the Navigator to carry out:
    ('send', number, x, y, yaw), ('cancel',), ('limit', mps or None).

    States: idle, running, dwelling (waiting at a waypoint), relocalizing,
    paused (reason 'user', 'failed' or 'stalled'), arrived, stopped. `number` counts the
    goals sent to Nav2, so a late reply about an earlier one is ignored.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state = 'idle'
        self.reason = ''
        self.mission: dict[str, Any] | None = None
        self.version = 0
        self.index = 0
        self.loop = 1
        self.left: float | None = None
        self.recoveries = 0
        self.number = 0
        self.dwell_until = 0.0
        self.retries = 0  # resends of this waypoint after a success away from it
        self.stalled = False  # the motor's stall protection holds the car
        self.visited: set[int] = set()  # waypoints reached since the mission started
        self.log_on = False  # events are kept only while a recording runs
        self.events: list[dict[str, Any]] = []

    # -- helpers; the caller holds self.lock --

    def _note(self, event: str, **fields: Any) -> None:
        if self.log_on:
            self.events.append(
                {'event': event, 'loop': self.loop, 'waypoint': self.index + 1, **fields}
            )

    def _pause(self, reason: str) -> None:
        self.number += 1
        self.state, self.reason, self.left = 'paused', reason, None
        self._note('paused', reason=reason)

    def _waypoints(self) -> list[dict[str, Any]]:
        assert self.mission is not None
        return self.mission['waypoints']

    def _previous(self, pose: Pose | None) -> tuple[float, float] | None:
        wps = self._waypoints()
        if self.index > 0:
            return wps[self.index - 1]['x'], wps[self.index - 1]['y']
        if self.loop > 1:
            return wps[-1]['x'], wps[-1]['y']
        return pose[:2] if pose else None

    def _leg(self, pose: Pose | None, again: bool = False) -> list[Action]:
        wp = self._waypoints()[self.index]
        yaw = goal_yaw(wp, self._previous(pose), pose[2] if pose else 0.0)
        self.number += 1
        self.state, self.reason, self.left, self.recoveries = 'running', '', None, 0
        self._note('sent', x=wp['x'], y=wp['y'], yaw=round(yaw, 4), resend=again)
        return [('limit', wp['speed_mps']), ('send', self.number, wp['x'], wp['y'], yaw)]

    def _end(self, state: str) -> list[Action]:
        self.number += 1
        self.state, self.reason, self.left = state, '', None
        self._note(state)
        return [('cancel',), ('limit', None)]

    def _next(self, pose: Pose | None) -> list[Action]:
        assert self.mission is not None
        self.retries = 0
        self.index += 1
        if self.index >= len(self._waypoints()):
            loops = self.mission['loops']
            if loops != 0 and self.loop >= loops:
                self.index = len(self._waypoints()) - 1
                return self._end('arrived')
            self.loop += 1
            self.index = 0
        return self._leg(pose)

    def _reached(self, pose: Pose | None, extra_m: float = 0.0, extra_rad: float = 0.0) -> bool:
        if pose is None:
            return False
        wp = self._waypoints()[self.index]
        if math.hypot(pose[0] - wp['x'], pose[1] - wp['y']) > wp['radius_m'] + extra_m:
            return False
        h = wp['heading_deg']
        return h is None or angle_between(pose[2], math.radians(h)) <= YAW_TOLERANCE + extra_rad

    def _at_waypoint(self, now: float, pose: Pose | None) -> list[Action]:
        self.visited.add(self.index)
        self._note('reached')
        dwell = self._waypoints()[self.index]['dwell_sec']
        if dwell > 0:
            self.number += 1
            self.state, self.left, self.dwell_until = 'dwelling', None, now + dwell
            return [('cancel',)]
        return self._next(pose)

    # -- events --

    def start(self, mission: dict[str, Any], pose: Pose | None) -> list[Action]:
        with self.lock:
            self.mission, self.version = mission, self.version + 1
            self.index, self.loop, self.retries, self.visited = 0, 1, 0, set()
            self._note('start', mission=mission)
            return self._leg(pose)

    def tick(
        self, now: float, pose: Pose | None, localized: bool | None, stalled: bool = False
    ) -> list[Action]:
        """Advance on the car's pose, localization (None: not running) and stall protection."""
        with self.lock:
            self.stalled = stalled
            if localized is None:
                if self.state in ACTIVE or self.state == 'paused':
                    return self._end('stopped')
                return []
            if self.state == 'running':
                if stalled:
                    self._pause('stalled')
                    return [('cancel',)]
                if not localized:
                    self.number += 1
                    self.state = 'relocalizing'
                    self._note('relocalizing')
                    return [('cancel',)]
                if self._reached(pose):
                    return self._at_waypoint(now, pose)
            elif self.state == 'relocalizing':
                if localized:
                    self._note('localized')
                    return self._leg(pose)
            elif self.state == 'dwelling' and now >= self.dwell_until:
                return self._next(pose)
            return []

    def refused(self, number: int) -> None:
        with self.lock:
            if number == self.number and self.state == 'running':
                self._pause('failed')

    def result(self, number: int, status: str, now: float, pose: Pose | None) -> list[Action]:
        """Nav2's result for a goal: 'succeeded', 'canceled' or 'failed'."""
        with self.lock:
            if number != self.number or self.state != 'running':
                return []
            if status == 'succeeded':
                if self._reached(pose, ARRIVE_MARGIN_M, ARRIVE_MARGIN_RAD):
                    return self._at_waypoint(now, pose)
                if self.retries < RETRY_LIMIT:
                    self.retries += 1
                    return self._leg(pose, again=True)
            if status in ('succeeded', 'failed'):
                self._pause('failed')
            return []

    def current(self, number: int) -> bool:
        with self.lock:
            return number == self.number

    def feedback(self, number: int, left: float, recoveries: int) -> None:
        with self.lock:
            if number == self.number:
                self.left, self.recoveries = left, recoveries

    def pause(self) -> list[Action]:
        with self.lock:
            if self.state not in ACTIVE:
                return []
            self._pause('user')
            return [('cancel',)]

    def resume(self, pose: Pose | None) -> list[Action]:
        with self.lock:
            if self.state != 'paused' or self.stalled:
                return []
            self.retries = 0
            self._note('resumed')
            return self._leg(pose)

    def skip(self, pose: Pose | None) -> list[Action]:
        with self.lock:
            if (self.state in ACTIVE or self.state == 'paused') and not self.stalled:
                self._note('skipped')
                return self._next(pose)
            return []

    def stop(self) -> list[Action]:
        with self.lock:
            if self.state in ACTIVE or self.state == 'paused':
                return self._end('stopped')
            return []

    def edit(self, mission: dict[str, Any], pose: Pose | None) -> list[Action]:
        """Replace the waypoints; a change to the one being driven to sends it again."""
        with self.lock:
            busy = self.mission is not None and (self.state in ACTIVE or self.state == 'paused')
            before = self._waypoints()[self.index] if busy else None
            # A waypoint keeps its mark while it keeps its place on the map.
            reached = Counter(
                (w['x'], w['y'])
                for i, w in enumerate(self.mission['waypoints'] if self.mission else [])
                if i in self.visited
            )
            self.visited = set()
            for i, w in enumerate(mission['waypoints']):
                if reached[w['x'], w['y']] > 0:
                    reached[w['x'], w['y']] -= 1
                    self.visited.add(i)
            self.mission, self.version = mission, self.version + 1
            if not busy:
                return []
            self._note('edited', mission=mission)
            self.index = min(self.index, len(mission['waypoints']) - 1)
            if self.state == 'running' and mission['waypoints'][self.index] != before:
                self.retries = 0
                return self._leg(pose)
            return []

    def underway(self) -> bool:
        with self.lock:
            return self.state in ACTIVE or self.state == 'paused'

    def record_events(self, on: bool, underway: bool = False) -> None:
        """Keep events from here on, or drop them; `underway` notes where the mission is."""
        with self.lock:
            self.log_on, self.events = on, []
            if on and underway:
                self._note('recording', mission=self.mission)

    def take_events(self) -> list[dict[str, Any]]:
        with self.lock:
            out, self.events = self.events, []
            return out

    def _heading_for(self) -> int | None:
        """Return the waypoint the car goes to next, or None."""
        if self.state in ('running', 'relocalizing', 'paused'):
            return self.index
        if self.state != 'dwelling' or self.mission is None:
            return None
        if self.index + 1 < len(self._waypoints()):
            return self.index + 1
        loops = self.mission['loops']
        return 0 if loops == 0 or self.loop < loops else None

    def view(self, now: float) -> dict[str, Any]:
        with self.lock:
            dwelling = self.state == 'dwelling'
            return {
                'state': self.state,
                'reason': self.reason,
                'index': self.index,
                'loop': self.loop,
                'loops': self.mission['loops'] if self.mission else 1,
                'count': len(self.mission['waypoints']) if self.mission else 0,
                'left': None if self.left is None else round(self.left, 2),
                'recoveries': self.recoveries,
                'stalled': self.stalled,
                'next': self._heading_for(),
                'visited': sorted(self.visited),
                'dwell_left': round(max(0.0, self.dwell_until - now), 1) if dwelling else None,
                'mission': self.mission,
                'version': self.version,
            }


class Commander:
    def __init__(self, lease: DriveLease | None = None) -> None:
        self.run = Run()
        self.lease = lease or DriveLease(OWNER, 'Navigation')
        self.owner = ''
        self.name = ''
        self.message = ''  # why the last mission ended by itself, if it did
        self.log = ''

    def command(
        self,
        raw: dict[str, Any],
        pose: Pose | None,
        localized: bool | None,
        stalled: bool,
        ready: bool,
    ) -> tuple[bool, str, list[Action]]:
        """Return (accepted, message, actions)."""
        verb = raw.get('verb')
        if verb == 'start':
            return self._start(raw, pose, localized, stalled, ready)
        if verb == 'check':
            message = self._problem(localized, stalled, ready)
            return not message, message, []
        if verb == 'edit':
            return True, '', self.run.edit(missions.parse(raw.get('mission')), pose)
        if verb == 'log':
            self.log = str(raw.get('log') or '')
            self.run.record_events(bool(self.log), self.run.underway())
            return True, '', []
        actions = {
            'pause': self.run.pause,
            'resume': lambda: self.run.resume(pose),
            'skip': lambda: self.run.skip(pose),
            'stop': self.run.stop,
        }.get(str(verb))
        return (True, '', actions()) if actions else (False, UNKNOWN, [])

    def _start(
        self,
        raw: dict[str, Any],
        pose: Pose | None,
        localized: bool | None,
        stalled: bool,
        ready: bool,
    ) -> tuple[bool, str, list[Action]]:
        mission = missions.parse(raw.get('mission'))
        message = self._problem(localized, stalled, ready)
        if message:
            return False, message, []
        self.lease.name, self.lease.share = str(raw.get('name') or 'Navigation'), False
        if not self.lease.take():
            return False, self.lease.refused(), []
        self.owner, self.name, self.message = str(raw.get('owner') or ''), self.lease.name, ''
        self.log = str(raw.get('log') or '')
        self.run.record_events(bool(self.log))
        return True, '', self.run.start(mission, pose)

    def _problem(self, localized: bool | None, stalled: bool, ready: bool) -> str:
        """Return why a mission cannot start, or ''."""
        if not ready or localized is None:
            return NO_MAP
        if self.run.underway():
            return f'{self.name} is driving.'
        if not localized:
            return NOT_LOCALIZED
        if stalled:
            return STALLED
        return self.lease.refused() if self.lease.holder() else ''

    def tick(
        self, now: float, pose: Pose | None, localized: bool | None, stalled: bool
    ) -> list[Action]:
        """Advance the mission, and keep the lease from its start to its end."""
        actions = self.run.tick(now, pose, localized, stalled)
        if self.run.underway():
            self.lease.share = self.run.state in SHARED
            if not self.lease.hold():
                self.message = self.lease.refused()
                actions += self.run.stop()
        elif self.lease.held and self.lease.enforced:
            self.lease.release()
        return actions

    def view(self, now: float) -> dict[str, Any]:
        out = self.run.view(now)
        del out['mission']
        return {**out, 'owner': self.owner, 'name': self.name, 'message': self.message}
