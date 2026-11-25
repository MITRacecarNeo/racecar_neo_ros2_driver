#!/usr/bin/env python3
"""
Planner app on port 8090 (/planner/): drive the car through waypoints on a map with Nav2.

The page (scripts/planner.html) shows the map, the car and the lidar as the
Mapper does, the waypoints, and the paths Nav2 plans. Loading a map, Set pose
and the Drive panel work as in the Mapper: this server builds on
scripts/mapview.py (Viewer, Job, the map and thumbnail routes) under its own
node name.

A mission (racecar_neo_ros2_driver/missions.py) is a list of waypoints, each
with a heading or any, a radius, a speed limit and a wait, and a loop count;
missions are saved beside their map. `Run` steps through one: a
navigate_to_pose goal per waypoint, the next one preempting the last so the
car does not stop between them. A waypoint is reached once the car is within
its radius (and facing its heading, if it has one), or Nav2 reports it
reached with the car near it; a report with the car elsewhere sends the
waypoint again. A waypoint Nav2 cannot reach, or the motor's stall protection
(/pit/stall), pauses the mission. Lost localization pauses it too: the Nav2
goal is canceled (the bridge has already stopped the car), and the waypoint
is sent again once the car is localized. The speed limit goes to Nav2 on
/speed_limit.

A mission records only while its Record toggle is on (mission_data.py,
mission_recorder.py): a bag through `racecar log` and an event log beside it,
from Go, or from the moment the toggle goes on, until the mission ends or the
toggle goes off. With it off nothing is started, subscribed or written. A
recording problem never stops the mission.

The preview plans every leg with compute_path_to_pose before Go. The costmap
view subscribes to /global_costmap/costmap only while the page has it on.

The car moves only while RB is held or the safety override is on; the page
keeps the override alive while it is open (scripts/web/override.js).
"""

from __future__ import annotations

from collections import Counter
import gzip
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse

import mapview
import mission_recorder
import numpy as np
from racecar_neo_ros2_driver import apps, mission_data, missions
from racecar_neo_ros2_driver.maps import valid_name
import webui

HERE = Path(__file__).resolve().parent
PORT = apps.port('planner')
PAGE = HERE / 'planner.html'
NODE = 'planner'
ACTION = 'navigate_to_pose'
PREVIEW_ACTION = 'compute_path_to_pose'
STALL_TOPIC = '/pit/stall'
SPEED_LIMIT_TOPIC = '/speed_limit'
# Map actions the Planner offers: load a map, and save or stop a mapping run it finds.
PLANNER_ACTIONS = ('load', 'save', 'stop')
PATH_POINTS = 400
GOAL_WAIT_SEC = 2.0
PREVIEW_WAIT_SEC = 5.0
TICK_SEC = 0.1
YAW_TOLERANCE = 0.5  # rad, Nav2's yaw_goal_tolerance (config/nav2.yaml)
# Nav2 can report a new goal reached for the path to the one before it. Its word
# counts only with the car this close; else the waypoint is sent again, so often.
ARRIVE_MARGIN_M = 0.15
ARRIVE_MARGIN_RAD = 0.15
RETRY_LIMIT = 10
NO_MAP = 'Load a map first.'
NO_SUMMARY = 'Could not read the recording.'
NOT_LOCALIZED = 'Wait until the car is localized.'
STALLED = 'Car stalled.'
BUSY = 'Another change is running.'
WATCH_SEC = 0.5
STATUS_SEC = 1.0
FREE_CHECK_SEC = 5.0
STEP_M = (0.005, 0.5)  # pose steps that count as driving, not noise or a new pose
PLAN_POINTS = 100
PLAN_END_M = 0.5
MISSION_NAME_RULE = 'Mission names use letters, digits, - and _.'
RELOAD_TEXT = mapview.RELOAD
ACTIVE = ('running', 'dwelling', 'relocalizing')

Pose = tuple[float, float, float]
Action = tuple[Any, ...]


def thin(points: list[list[float]], limit: int = PATH_POINTS) -> list[list[float]]:
    """Return at most `limit` points of a path, always keeping both ends."""
    if len(points) <= limit:
        return points
    step = (len(points) - 1) / (limit - 1)
    return [points[round(i * step)] for i in range(limit)]


def path_points(msg: Any) -> list[list[float]]:
    """Return a nav_msgs/Path as [x, y] pairs, rounded to the millimeter and thinned."""
    return thin([[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses])


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


class Navigator(mapview.Viewer):
    """The Mapper's viewer, plus Nav2: missions, the path preview, the costmap on request."""

    def __init__(self) -> None:
        super().__init__(NODE)
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import ComputePathToPose, NavigateToPose
        from nav2_msgs.msg import SpeedLimit
        from racecar_neo_ros2_driver.qos import LATCHED
        from rclpy.action import ActionClient

        self.run = Run()
        self.recorder = mission_recorder.Recorder()
        self.recording = False
        self.not_recording = False  # wanted, but it could not start or ended early
        self._events: mission_data.EventLog | None = None
        self.rec_status: dict[str, Any] | None = None  # once a second while recording
        self.rec_summary: dict[str, Any] = {'id': 0, 'text': ''}  # the last one recorded
        self._label = ''
        self._dist = 0.0
        self._plan_sub: Any = None
        self._want_plan: tuple[float, float, int, int] | None = None  # x, y, loop, waypoint
        self._rec_lock = threading.Lock()  # one recording change at a time
        self._start_lock = threading.Lock()
        self._abort = False
        self._events_lock = threading.Lock()
        self.path: list[list[float]] = []
        self.path_version = 0
        self._goal_handle: Any = None
        self._action = ActionClient(self.node, NavigateToPose, ACTION)
        self._preview = ActionClient(self.node, ComputePathToPose, PREVIEW_ACTION)
        self._speed = self.node.create_publisher(SpeedLimit, SPEED_LIMIT_TOPIC, 1)
        self._speed_msg = SpeedLimit
        self._costmap_sub: Any = None
        self.costmap_info: dict[str, Any] | None = None
        self.costmap_gz = b''
        self.costmap_version = 0
        # Held for the whole run, not only while a page polls: a mission keeps going
        # with the page closed (RB held), and must pause if localization is lost.
        self._localized: bool | None = None
        self._stalled = False
        self.map_name = ''
        self.node.create_subscription(DiagnosticStatus, STALL_TOPIC, self._on_stall, LATCHED)
        self._pose: Pose | None = None
        self.node.create_subscription(
            DiagnosticStatus, '/autonomy/status', self._on_mission_status, LATCHED
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/autonomy/pose', self._on_mission_pose, 10
        )
        threading.Thread(target=self._mission_loop, daemon=True).start()

    # _open and _close run under self._lock (webui.IdleGate, from touch and the watch loop).
    def _open(self) -> None:
        super()._open()
        from nav_msgs.msg import Path as PathMsg

        self._handles.append(self.node.create_subscription(PathMsg, '/plan', self._on_path, 10))

    def _close(self) -> None:
        super()._close()
        self._drop_costmap()
        self.path = []

    def _on_path(self, msg: Any) -> None:
        points = path_points(msg)
        with self._lock:
            self.path = points
            self.path_version += 1

    # -- costmap view --

    def set_costmap(self, on: bool) -> None:
        with self._lock:
            if not on:
                self._drop_costmap()
            elif self._costmap_sub is None:
                from nav_msgs.msg import OccupancyGrid
                from rclpy.qos import DurabilityPolicy, QoSProfile

                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self._costmap_sub = self.node.create_subscription(
                    OccupancyGrid, '/global_costmap/costmap', self._on_costmap, qos
                )

    def _drop_costmap(self) -> None:
        """Unsubscribe from the costmap; the caller holds self._lock."""
        if self._costmap_sub is not None:
            self.node.destroy_subscription(self._costmap_sub)
            self._costmap_sub = None
        self.costmap_info, self.costmap_gz = None, b''

    def _on_costmap(self, msg: Any) -> None:
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int8).tobytes()
        with self._lock:
            self.costmap_info = {
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin': [info.origin.position.x, info.origin.position.y],
            }
            self.costmap_gz = gzip.compress(data, compresslevel=5)
            self.costmap_version += 1

    def costmap_payload(self) -> tuple[dict[str, Any], bytes] | None:
        with self._lock:
            if self.costmap_info is None:
                return None
            return {**self.costmap_info, 'version': self.costmap_version}, self.costmap_gz

    # -- path preview --

    def _pose_stamped(self, pose: Pose) -> Any:
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.orientation.w = math.cos(pose[2] / 2.0)
        return msg

    def _plan(self, start: Pose | None, goal: Pose) -> list[list[float]] | None:
        """Plan one leg without driving; start None plans from the car."""
        from nav2_msgs.action import ComputePathToPose

        request = ComputePathToPose.Goal()
        request.goal = self._pose_stamped(goal)
        request.use_start = start is not None
        if start is not None:
            request.start = self._pose_stamped(start)
        done = threading.Event()
        out: dict[str, Any] = {}

        def finished(future: Any) -> None:
            out['result'] = future.result()
            done.set()

        def accepted(future: Any) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(finished)

        self._preview.send_goal_async(request).add_done_callback(accepted)
        if not done.wait(PREVIEW_WAIT_SEC) or out.get('result') is None:
            return None
        result = out['result'].result
        points = path_points(result.path)
        return points if result.error_code == 0 and len(points) >= 2 else None

    def preview(self, mission: dict[str, Any]) -> tuple[bool, Any]:
        """Plan every leg: (True, [points or None per leg]) or (False, message)."""
        if not self._preview.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, NO_MAP
        return True, [self._plan(start, goal) for start, goal in legs(mission, self._pose)]

    # -- missions --

    def _apply(self, actions: list[Action]) -> None:
        for action in actions:
            kind = action[0]
            if kind == 'send':
                self._send(*action[1:])
            elif kind == 'cancel':
                self._cancel()
            elif kind == 'limit':
                msg = self._speed_msg()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.percentage = False
                msg.speed_limit = float(action[1] or 0.0)  # 0: no limit
                self._speed.publish(msg)

    def _send(self, number: int, x: float, y: float, yaw: float) -> None:
        """Send a waypoint to Nav2; it preempts the goal before it."""
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped((x, y, yaw))

        def feedback(msg: Any) -> None:
            fb = msg.feedback
            self.run.feedback(number, float(fb.distance_remaining), int(fb.number_of_recoveries))

        def finished(future: Any) -> None:
            result = future.result()
            status = {
                GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                GoalStatus.STATUS_CANCELED: 'canceled',
            }.get(result.status if result is not None else -1, 'failed')
            self._apply(self.run.result(number, status, time.monotonic(), self._pose))

        def accepted(future: Any) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.run.refused(number)
                return
            if not self.run.current(number):
                handle.cancel_goal_async()  # paused or stopped while Nav2 was accepting it
                return
            with self._lock:
                self._goal_handle = handle
            handle.get_result_async().add_done_callback(finished)

        self._action.send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)

    def _cancel(self) -> None:
        with self._lock:
            handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def _cannot_drive(self) -> str:
        if not self._localized:
            return NOT_LOCALIZED
        return STALLED if self._stalled else ''

    def start(self, mission: dict[str, Any], label: str = '') -> tuple[bool, str]:
        """Run a mission; with its Record toggle on, once the recorder is ready."""
        if not self._action.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, NO_MAP
        if not self._start_lock.acquire(blocking=False):
            return False, BUSY
        try:
            self._abort, self.not_recording = False, False
            problem = self._cannot_drive()
            if problem:
                return False, problem
            self._label = label or self.map_name
            self._end_recording()
            if mission['record']['on']:
                self._begin_recording(mission)
                # Up to a few seconds have passed.
                problem = self._cannot_drive()
                if problem or self._abort:
                    self._end_recording(summary=False)
                    return False, problem or 'Stopped.'
            self._apply(self.run.start(mission, self._pose))
            if self.recording:
                threading.Thread(target=self._watch_recording, daemon=True).start()
            return True, ''
        finally:
            self._start_lock.release()

    # -- recording: only while a mission's Record toggle is on --

    def _begin_recording(self, mission: dict[str, Any], underway: bool = False) -> None:
        with self._rec_lock:
            if self.recording:
                return
            config = mission_data.load_config()
            wanted = mission_data.topics(config, mission['record']['groups'])
            name = mission_data.bag_name(self._label, time.time())
            if not self.recorder.start(name, wanted, mission_data.min_free_bytes(config)):
                self.not_recording = True
                return
            folder = self.recorder.folder
            assert folder is not None
            from nav_msgs.msg import Path as PathMsg

            self._dist, self._want_plan = 0.0, None
            with self._events_lock:
                self._events = mission_data.EventLog(folder, mission)
            self.run.record_events(True, underway)
            with self._lock:
                self._plan_sub = self.node.create_subscription(
                    PathMsg, '/plan', self._on_leg_plan, 10
                )
            self.recording = True
            if not underway:
                live = {t for t, _ in self.node.get_topic_names_and_types()}
                ready = [t for t in wanted if t in live]
                self.recorder.wait_ready(ready, lambda: self._abort)

    def _end_recording(self, summary: bool = True) -> None:
        with self._rec_lock:
            if not self.recording:
                return
            self._write_events()
            self.run.record_events(False)
            with self._lock:
                if self._plan_sub is not None:
                    self.node.destroy_subscription(self._plan_sub)
                    self._plan_sub = None
            self.recorder.stop()
            with self._events_lock:
                if self._events is not None:
                    self._events.flush()
                    self._events = None
            self.recording, self.rec_status = False, None
            folder = self.recorder.folder
        if folder is not None and summary:
            threading.Thread(target=self._summarize, args=(folder,), daemon=True).start()

    def _summarize(self, folder: Path) -> None:
        text = self.recorder.analyze(folder) or NO_SUMMARY
        self.rec_summary = {'id': self.rec_summary['id'] + 1, 'text': text}

    def _watch_recording(self) -> None:
        """End the recording with the mission, with its toggle, or on a problem."""
        config = mission_data.load_config()
        checked = shown = time.monotonic()
        while self.recording:
            time.sleep(WATCH_SEC)
            if time.monotonic() - shown >= STATUS_SEC:
                shown = time.monotonic()
                self.rec_status = self.recorder.status(mission_data.min_free_bytes(config))
            mission = self.run.mission
            wanted = mission is not None and mission['record']['on']
            if not self.run.underway() or not wanted:
                self._end_recording()
                return
            if time.monotonic() - checked >= FREE_CHECK_SEC:
                checked = time.monotonic()
                if (
                    not self.recorder.running()
                    or self.recorder.free_bytes() < mission_data.min_free_bytes(config)
                ):
                    self.not_recording = True
                    self._end_recording()
                    return

    def _write_events(self) -> None:
        events = self.run.take_events()
        with self._events_lock:
            if self._events is None:
                return
            pose = None if self._pose is None else [round(v, 3) for v in self._pose]
            for e in events:
                if e['event'] == 'sent':
                    self._want_plan = (e['x'], e['y'], e['loop'], e['waypoint'])
                fields = {k: v for k, v in e.items() if k != 'event'}
                self._events.add(
                    e['event'], time.time(), **fields, pose=pose, dist_m=round(self._dist, 2)
                )
            self._events.flush()

    def _on_leg_plan(self, msg: Any) -> None:
        """Log the first path Nav2 plans to the waypoint just sent."""
        want, points = self._want_plan, path_points(msg)
        if want is None or not points:
            return
        if math.hypot(points[-1][0] - want[0], points[-1][1] - want[1]) > PLAN_END_M:
            return  # still the path to the waypoint before
        self._want_plan = None
        with self._events_lock:
            if self._events is not None:
                self._events.add(
                    'planned', time.time(), loop=want[2], waypoint=want[3],
                    path=thin(points, PLAN_POINTS), dist_m=round(self._dist, 2),
                )  # fmt: skip

    def shutdown(self) -> None:
        """Stop the car's goal and finish the bag before the node goes."""
        if self.run.underway():
            self.control('stop')
            time.sleep(0.3)  # the cancel goes out on the executor
        self._end_recording(summary=False)
        super().shutdown()

    def control(self, verb: str) -> None:
        """Pause, resume, skip or stop the mission."""
        pose = self._pose
        if verb == 'stop':
            self._abort = True  # also a start still waiting for the recorder
        actions = {
            'pause': self.run.pause,
            'resume': lambda: self.run.resume(pose),
            'skip': lambda: self.run.skip(pose),
            'stop': self.run.stop,
        }[verb]()
        self._apply(actions)

    def edit(self, mission: dict[str, Any]) -> None:
        self._apply(self.run.edit(mission, self._pose))
        if mission['record']['on'] and self.run.underway() and not self.recording:
            threading.Thread(target=self._record_from_here, args=(mission,), daemon=True).start()

    def _record_from_here(self, mission: dict[str, Any]) -> None:
        self.not_recording = False
        self._begin_recording(mission, underway=True)
        self._watch_recording()

    def _on_mission_status(self, msg: Any) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        localizing = values.get('mode') == 'localization'
        self._localized = values.get('localized') == 'true' if localizing else None
        self.map_name = values.get('map', '') if localizing else ''

    def _on_stall(self, msg: Any) -> None:
        # pit_stall.py: 0 normal; the car drives again only from it.
        self._stalled = {kv.key: kv.value for kv in msg.values}.get('state', '0') != '0'

    def _on_mission_pose(self, msg: Any) -> None:
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        before, self._pose = self._pose, (p.position.x, p.position.y, yaw)
        if self.recording and before is not None:
            step = math.hypot(self._pose[0] - before[0], self._pose[1] - before[1])
            if STEP_M[0] <= step <= STEP_M[1]:
                self._dist += step

    def _mission_loop(self) -> None:
        while True:
            time.sleep(TICK_SEC)
            now = time.monotonic()
            self._apply(self.run.tick(now, self._pose, self._localized, self._stalled))
            if self.recording:
                self._write_events()

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        out = super().snapshot(trail_from, trail_id)
        del out['trail']  # the Planner page draws the planned path, not the trail
        with self._lock:
            out['path'] = self.path
            out['path_version'] = self.path_version
            out['costmap_version'] = self.costmap_version if self.costmap_info else 0
        out['mission'] = {
            **self.run.view(time.monotonic()),
            'recording': self.recording,
            'not_recording': self.not_recording,
            'rec_status': self.rec_status,
            'rec_summary': self.rec_summary['id'],
        }
        return out


def mission_name(body: dict[str, Any]) -> str:
    name = str(body.get('name') or '')
    if not valid_name(name):
        raise ValueError(MISSION_NAME_RULE)
    return name


class Handler(mapview.Handler):
    viewer: Navigator

    POSTS = ('/api/run', '/api/run_saved', '/api/edit', '/api/control', '/api/stop',
             '/api/preview', '/api/missions/save', '/api/costmap', '/api/action')  # fmt: skip

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        q = {k: v[-1] for k, v in parse_qs(url.query).items()}
        if url.path in ('/', '/index.html'):
            self.page(PAGE)
        elif url.path == '/api/costmap':
            self.viewer.touch()
            payload = self.viewer.costmap_payload()
            if payload is None:
                self.send(204, b'', 'application/octet-stream')
            else:
                self._grid(*payload)
        elif url.path == '/api/missions':
            name = self.viewer.map_name
            found = missions.listing(name) if name else []
            groups = mission_data.groups_view(mission_data.load_config())
            self.send_json({'map': name, 'missions': found, 'groups': groups})
        elif url.path == '/api/recording':
            self.send_json(self.viewer.rec_summary)
        elif url.path == '/api/missions/one':
            try:
                mission = missions.load(self.viewer.map_name, q['name'])
                self.send_json({'ok': True, 'mission': mission})
            except (OSError, ValueError, KeyError):
                self.send_json({'ok': False, 'message': RELOAD_TEXT}, 404)
        else:
            super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path not in self.POSTS:
            super().do_POST()
            return
        if self.refuse_cross_site():
            return
        v = self.viewer
        try:
            body = self.read_json()
            if path in ('/api/run', '/api/run_saved'):
                v.touch()
                label = str(body.get('name') or '')
                if path == '/api/run':
                    mission = missions.parse(body.get('mission'))
                    label = label if valid_name(label) else ''
                elif not v.map_name:
                    self.send_json({'ok': False, 'message': NO_MAP}, 409)
                    return
                else:
                    try:
                        mission = missions.load(v.map_name, mission_name(body))
                    except FileNotFoundError:
                        self.send_json({'ok': False, 'message': 'No such mission.'}, 404)
                        return
                ok, message = v.start(mission, label)
                self.send_json({'ok': ok, 'message': message}, 200 if ok else 409)
            elif path == '/api/edit':
                v.edit(missions.parse(body.get('mission')))
                self.send_json({'ok': True})
            elif path == '/api/control':
                verb = body.get('verb')
                if verb not in ('pause', 'resume', 'skip', 'stop'):
                    raise ValueError('not a mission control')
                v.control(verb)
                self.send_json({'ok': True})
            elif path == '/api/stop':
                v.control('stop')
                self.send_json({'ok': True})
            elif path == '/api/preview':
                v.touch()
                ok, found = v.preview(missions.parse(body.get('mission')))
                if ok:
                    self.send_json({'ok': True, 'legs': found})
                else:
                    self.send_json({'ok': False, 'message': found}, 409)
            elif path == '/api/missions/save':
                name, mission = mission_name(body), missions.parse(body.get('mission'))
                if not v.map_name:
                    self.send_json({'ok': False, 'message': NO_MAP}, 409)
                elif missions.exists(v.map_name, name) and not body.get('replace'):
                    self.send_json({'ok': False, 'exists': True}, 409)
                else:
                    missions.save(v.map_name, name, mission)
                    self.send_json({'ok': True})
            elif path == '/api/costmap':
                v.touch()
                v.set_costmap(bool(body.get('on')))
                self.send_json({'ok': True})
            else:
                if body.get('action') not in PLANNER_ACTIONS:
                    raise ValueError('not a Planner action')
                argv = mapview.action_argv(body)
                if not self.job.start(argv):
                    self.send_json({'ok': False, 'message': BUSY}, 409)
                    return
                self.send_json({'ok': True, 'argv': argv})
        except (ValueError, KeyError, TypeError) as exc:
            known = str(exc) in (mapview.NAME_RULE, MISSION_NAME_RULE)
            self.send_json({'ok': False, 'message': str(exc) if known else RELOAD_TEXT}, 400)


def main() -> int:
    import rclpy

    rclpy.init()
    Handler.viewer = Navigator()
    Handler.job = mapview.Job([sys.executable, str(mapview.RACECAR_MAP)])
    Handler.checks = mapview.Job([sys.executable, str(mapview.RACECAR_CHECK)], 'racecar check')
    print(f'planner: http://0.0.0.0:{PORT}', flush=True)
    webui.serve(PORT, Handler, cleanup=Handler.viewer.shutdown)
    return 0


if __name__ == '__main__':
    sys.exit(main())
