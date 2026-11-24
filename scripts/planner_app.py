#!/usr/bin/env python3
"""
Planner app on port 8090 (/planner/): drive the car to a point on a map with Nav2.

The page (scripts/planner.html) shows the map, the car and the lidar as the
Mapper does, and the path Nav2 plans; a drag on the map sets a goal (position
and heading) and shows the path to it, Go sends it, Stop cancels it. Loading a
map, Set pose and the Drive panel work as in the Mapper: this server builds on
scripts/mapview.py (Viewer, Job, the map and thumbnail routes) under its own
node name.

Nav2 runs with localization in the autonomy service (launch/autonomy.launch.py).
The goal preview comes from its compute_path_to_pose action, goals go to
navigate_to_pose, and the path being driven comes from /plan. The costmap view
subscribes to /global_costmap/costmap only while the page has it on.

When localization is lost during a goal, the goal pauses: the Nav2 goal is
canceled (the bridge has already stopped the car), AMCL is asked for an
update without motion twice a second, and the same goal is sent again once
the car is localized. Stop ends the wait.

The car moves only while RB is held or the safety override is on; the page
keeps the override alive while it is open (scripts/web/override.js).
"""

from __future__ import annotations

import gzip
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import urlparse

import mapview
import numpy as np
from racecar_neo_ros2_driver import apps
import webui

HERE = Path(__file__).resolve().parent
PORT = apps.port('planner')
PAGE = HERE / 'planner.html'
NODE = 'planner'
ACTION = 'navigate_to_pose'
PREVIEW_ACTION = 'compute_path_to_pose'
NOMOTION_SERVICE = '/request_nomotion_update'
# Map actions the Planner offers: load a map, and save or stop a mapping run it finds.
PLANNER_ACTIONS = ('load', 'save', 'stop')
PATH_POINTS = 400
GOAL_WAIT_SEC = 2.0
PREVIEW_WAIT_SEC = 5.0
MISSION_PERIOD_SEC = 0.5
NO_MAP = 'Load a map first.'
NO_PATH = 'No path to this goal.'
ACTIVE = ('sending', 'driving', 'relocalizing')


def thin(points: list[list[float]], limit: int = PATH_POINTS) -> list[list[float]]:
    """Return at most `limit` points of a path, always keeping both ends."""
    if len(points) <= limit:
        return points
    step = (len(points) - 1) / (limit - 1)
    return [points[round(i * step)] for i in range(limit)]


def path_points(msg: Any) -> list[list[float]]:
    """Return a nav_msgs/Path as [x, y] pairs, rounded to the millimeter and thinned."""
    return thin([[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses])


def pose_stamped(node: Any, x: float, y: float, yaw: float) -> Any:
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class Mission:
    """
    The goal in progress and how it went: what the page shows under the map.

    States: idle, sending, driving, relocalizing (paused while localization
    is lost), arrived, failed, stopped. `number` counts goals sent to Nav2,
    so a late reply about an earlier one is ignored.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state = 'idle'
        self.goal: list[float] | None = None
        self.left: float | None = None
        self.recoveries = 0
        self.number = 0

    def view(self) -> dict[str, Any]:
        with self.lock:
            return {
                'state': self.state,
                'goal': self.goal,
                'left': None if self.left is None else round(self.left, 2),
                'recoveries': self.recoveries,
            }

    def begin(self, goal: list[float]) -> int:
        with self.lock:
            self.number += 1
            self.state, self.goal, self.left, self.recoveries = 'sending', goal, None, 0
            return self.number

    def current(self, number: int) -> bool:
        with self.lock:
            return number == self.number

    def update(self, number: int, **values: Any) -> None:
        with self.lock:
            if number == self.number:
                for key, value in values.items():
                    setattr(self, key, value)

    def stop(self) -> None:
        """End the goal in progress (sent or paused); replies about it are ignored."""
        with self.lock:
            if self.state in ACTIVE:
                self.number += 1
                self.state = 'stopped'

    def step(self, localized: bool | None) -> str | None:
        """
        Advance on the car's localization; return what to do.

        localized is None when localization is not running (no map, or
        autonomy stopped): a goal in progress then stops. Returns 'pause'
        (cancel the Nav2 goal), 'nudge' (ask AMCL for an update), 'resume'
        (send the goal again), or None.
        """
        with self.lock:
            if localized is None:
                if self.state in ACTIVE:
                    self.number += 1
                    self.state = 'stopped'
                    return 'pause'
                return None
            if self.state in ('sending', 'driving') and not localized:
                self.number += 1
                self.state = 'relocalizing'
                return 'pause'
            if self.state == 'relocalizing':
                return 'resume' if localized else 'nudge'
            return None


class Navigator(mapview.Viewer):
    """The Mapper's viewer, plus Nav2: goals, the path preview, the costmap on request."""

    def __init__(self) -> None:
        super().__init__(NODE)
        from diagnostic_msgs.msg import DiagnosticStatus
        from nav2_msgs.action import ComputePathToPose, NavigateToPose
        from racecar_neo_ros2_driver.qos import LATCHED
        from rclpy.action import ActionClient
        from std_srvs.srv import Empty

        self.mission = Mission()
        self.path: list[list[float]] = []
        self.path_version = 0
        self._goal_handle: Any = None
        self._action = ActionClient(self.node, NavigateToPose, ACTION)
        self._preview = ActionClient(self.node, ComputePathToPose, PREVIEW_ACTION)
        self._nomotion = self.node.create_client(Empty, NOMOTION_SERVICE)
        self._empty = Empty.Request
        self._costmap_sub: Any = None
        self.costmap_info: dict[str, Any] | None = None
        self.costmap_gz = b''
        self.costmap_version = 0
        # Held for the whole run, not only while a page polls: a goal keeps going
        # with the page closed (RB held), and must pause if localization is lost.
        self._localized: bool | None = None
        self.node.create_subscription(
            DiagnosticStatus, '/autonomy/status', self._on_mission_status, LATCHED
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

    def preview(self, x: float, y: float, yaw: float) -> tuple[bool, Any]:
        """Plan from the car to a goal without driving: (True, points) or (False, message)."""
        from nav2_msgs.action import ComputePathToPose

        if not self._preview.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, NO_MAP
        goal = ComputePathToPose.Goal()
        goal.goal = pose_stamped(self.node, x, y, yaw)
        goal.use_start = False  # from the car's pose
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

        self._preview.send_goal_async(goal).add_done_callback(accepted)
        if not done.wait(PREVIEW_WAIT_SEC) or out.get('result') is None:
            return False, NO_PATH
        result = out['result'].result
        points = path_points(result.path)
        if result.error_code != 0 or len(points) < 2:
            return False, NO_PATH
        return True, points

    # -- goals --

    def go(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        """Send a goal to Nav2; the result arrives in self.mission."""
        from nav2_msgs.action import NavigateToPose

        if not self._action.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, NO_MAP
        self.stop()
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped(self.node, x, y, yaw)
        number = self.mission.begin([round(x, 3), round(y, 3), round(yaw, 4)])

        def feedback(msg: Any) -> None:
            fb = msg.feedback
            self.mission.update(number, left=float(fb.distance_remaining),
                                recoveries=int(fb.number_of_recoveries))  # fmt: skip

        def accepted(future: Any) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.mission.update(number, state='failed')
                return
            if not self.mission.current(number):
                # Paused or stopped while Nav2 was accepting it.
                handle.cancel_goal_async()
                return
            with self._lock:
                self._goal_handle = handle
            self.mission.update(number, state='driving')
            handle.get_result_async().add_done_callback(lambda f: self._finished(number, f))

        self._action.send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)
        return True, ''

    def _finished(self, number: int, future: Any) -> None:
        from action_msgs.msg import GoalStatus

        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_ABORTED
        state = {
            GoalStatus.STATUS_SUCCEEDED: 'arrived',
            GoalStatus.STATUS_CANCELED: 'stopped',
        }.get(status, 'failed')
        self.mission.update(number, state=state)

    def _cancel(self) -> None:
        with self._lock:
            handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def stop(self) -> None:
        """End the goal in progress, sent or paused."""
        self.mission.stop()
        self._cancel()

    # -- localization lost during a goal --

    def _on_mission_status(self, msg: Any) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        self._localized = (
            values.get('localized') == 'true' if values.get('mode') == 'localization' else None
        )

    def _mission_loop(self) -> None:
        while True:
            time.sleep(MISSION_PERIOD_SEC)
            action = self.mission.step(self._localized)
            if action == 'pause':
                self._cancel()
            elif action == 'nudge' and self._nomotion.service_is_ready():
                self._nomotion.call_async(self._empty())
            elif action == 'resume':
                goal = self.mission.view()['goal']
                if goal:
                    self.go(*goal)

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        out = super().snapshot(trail_from, trail_id)
        del out['trail']  # the Planner page draws the planned path, not the trail
        with self._lock:
            out['path'] = self.path
            out['path_version'] = self.path_version
            out['costmap_version'] = self.costmap_version if self.costmap_info else 0
        out['mission'] = self.mission.view()
        return out


def read_pose(body: dict[str, Any]) -> tuple[float, float, float]:
    x, y, yaw = (float(body[k]) for k in ('x', 'y', 'yaw'))
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError('not a pose')
    return x, y, yaw


class Handler(mapview.Handler):
    viewer: Navigator

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        if url.path in ('/', '/index.html'):
            self.page(PAGE)
        elif url.path == '/api/costmap':
            self.viewer.touch()
            payload = self.viewer.costmap_payload()
            if payload is None:
                self.send(204, b'', 'application/octet-stream')
            else:
                self._grid(*payload)
        else:
            super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path not in ('/api/goal', '/api/preview', '/api/stop', '/api/costmap', '/api/action'):
            super().do_POST()
            return
        if self.refuse_cross_site():
            return
        try:
            body = self.read_json()
            if path == '/api/goal':
                self.viewer.touch()
                ok, message = self.viewer.go(*read_pose(body))
                self.send_json({'ok': ok, 'message': message}, 200 if ok else 409)
            elif path == '/api/preview':
                self.viewer.touch()
                ok, found = self.viewer.preview(*read_pose(body))
                if ok:
                    self.send_json({'ok': True, 'path': found})
                else:
                    self.send_json({'ok': False, 'message': found}, 409)
            elif path == '/api/stop':
                self.viewer.stop()
                self.send_json({'ok': True})
            elif path == '/api/costmap':
                self.viewer.touch()
                self.viewer.set_costmap(bool(body.get('on')))
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
            known = str(exc) == mapview.NAME_RULE
            self.send_json({'ok': False, 'message': str(exc) if known else mapview.RELOAD}, 400)


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
