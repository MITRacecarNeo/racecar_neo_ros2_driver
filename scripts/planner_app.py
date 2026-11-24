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
import numpy as np
from racecar_neo_ros2_driver import apps, missions
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
NOT_LOCALIZED = 'Wait until the car is localized.'
STALLED = 'Car stalled.'
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

    # -- helpers; the caller holds self.lock --

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

    def _leg(self, pose: Pose | None) -> list[Action]:
        wp = self._waypoints()[self.index]
        yaw = goal_yaw(wp, self._previous(pose), pose[2] if pose else 0.0)
        self.number += 1
        self.state, self.reason, self.left, self.recoveries = 'running', '', None, 0
        return [('limit', wp['speed_mps']), ('send', self.number, wp['x'], wp['y'], yaw)]

    def _end(self, state: str) -> list[Action]:
        self.number += 1
        self.state, self.reason, self.left = state, '', None
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
                    self.number += 1
                    self.state, self.reason, self.left = 'paused', 'stalled', None
                    return [('cancel',)]
                if not localized:
                    self.number += 1
                    self.state = 'relocalizing'
                    return [('cancel',)]
                if self._reached(pose):
                    return self._at_waypoint(now, pose)
            elif self.state == 'relocalizing':
                if localized:
                    return self._leg(pose)
            elif self.state == 'dwelling' and now >= self.dwell_until:
                return self._next(pose)
            return []

    def refused(self, number: int) -> None:
        with self.lock:
            if number == self.number and self.state == 'running':
                self.number += 1
                self.state, self.reason = 'paused', 'failed'

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
                    return self._leg(pose)
            if status in ('succeeded', 'failed'):
                self.number += 1
                self.state, self.reason = 'paused', 'failed'
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
            self.number += 1
            self.state, self.reason, self.left = 'paused', 'user', None
            return [('cancel',)]

    def resume(self, pose: Pose | None) -> list[Action]:
        with self.lock:
            if self.state != 'paused' or self.stalled:
                return []
            self.retries = 0
            return self._leg(pose)

    def skip(self, pose: Pose | None) -> list[Action]:
        with self.lock:
            if (self.state in ACTIVE or self.state == 'paused') and not self.stalled:
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
            self.index = min(self.index, len(mission['waypoints']) - 1)
            if self.state == 'running' and mission['waypoints'][self.index] != before:
                self.retries = 0
                return self._leg(pose)
            return []

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

    def start(self, mission: dict[str, Any]) -> tuple[bool, str]:
        if not self._action.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, NO_MAP
        if not self._localized:
            return False, NOT_LOCALIZED
        if self._stalled:
            return False, STALLED
        self._apply(self.run.start(mission, self._pose))
        return True, ''

    def control(self, verb: str) -> None:
        """Pause, resume, skip or stop the mission."""
        pose = self._pose
        actions = {
            'pause': self.run.pause,
            'resume': lambda: self.run.resume(pose),
            'skip': lambda: self.run.skip(pose),
            'stop': self.run.stop,
        }[verb]()
        self._apply(actions)

    def edit(self, mission: dict[str, Any]) -> None:
        self._apply(self.run.edit(mission, self._pose))

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
        self._pose = (p.position.x, p.position.y, yaw)

    def _mission_loop(self) -> None:
        while True:
            time.sleep(TICK_SEC)
            now = time.monotonic()
            self._apply(self.run.tick(now, self._pose, self._localized, self._stalled))

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        out = super().snapshot(trail_from, trail_id)
        del out['trail']  # the Planner page draws the planned path, not the trail
        with self._lock:
            out['path'] = self.path
            out['path_version'] = self.path_version
            out['costmap_version'] = self.costmap_version if self.costmap_info else 0
        out['mission'] = self.run.view(time.monotonic())
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
            self.send_json({'map': name, 'missions': found})
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
                if path == '/api/run':
                    mission = missions.parse(body.get('mission'))
                elif not v.map_name:
                    self.send_json({'ok': False, 'message': NO_MAP}, 409)
                    return
                else:
                    try:
                        mission = missions.load(v.map_name, mission_name(body))
                    except FileNotFoundError:
                        self.send_json({'ok': False, 'message': 'No such mission.'}, 404)
                        return
                ok, message = v.start(mission)
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
                    self.send_json({'ok': False, 'message': 'Another change is running.'}, 409)
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
