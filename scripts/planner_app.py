#!/usr/bin/env python3
"""
Planner app on port 8090 (/planner/): drive the car to a point on a map with Nav2.

The page (scripts/planner.html) shows the map, the car and the lidar as the
Mapper does, and the path Nav2 plans; a drag on the map sets a goal (position
and heading), Go sends it, Stop cancels it. Loading a map, Set pose and the
Drive panel work as in the Mapper: this server builds on scripts/mapview.py
(Viewer, Job, the map and thumbnail routes) under its own node name.

Nav2 runs with localization in the autonomy service (launch/autonomy.launch.py);
goals go to its navigate_to_pose action, and the planned path comes from
/plan. The costmap view subscribes to /global_costmap/costmap only while the
page has it on.

The car moves only while RB is held or the safety override is on; the page
keeps the override alive while it is open (scripts/web/override.js).
"""

from __future__ import annotations

import gzip
import math
from pathlib import Path
import sys
import threading
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
# Map actions the Planner offers: load a map, and save or stop a mapping run it finds.
PLANNER_ACTIONS = ('load', 'save', 'stop')
PATH_POINTS = 400
GOAL_WAIT_SEC = 2.0


def thin(points: list[list[float]], limit: int = PATH_POINTS) -> list[list[float]]:
    """Return at most `limit` points of a path, always keeping both ends."""
    if len(points) <= limit:
        return points
    step = (len(points) - 1) / (limit - 1)
    return [points[round(i * step)] for i in range(limit)]


class Mission:
    """The goal in progress and how it went: what the page shows under the map."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state = 'idle'  # idle, sending, driving, arrived, failed, stopped
        self.goal: list[float] | None = None
        self.left: float | None = None
        self.recoveries = 0
        self.number = 0  # counts goals, so a late result from an old goal is ignored

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

    def update(self, number: int, **values: Any) -> None:
        with self.lock:
            if number == self.number:
                for key, value in values.items():
                    setattr(self, key, value)


class Navigator(mapview.Viewer):
    """The Mapper's viewer, plus Nav2: goals, the planned path, the costmap on request."""

    def __init__(self) -> None:
        super().__init__(NODE)
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self.mission = Mission()
        self.path: list[list[float]] = []
        self.path_version = 0
        self._goal_handle: Any = None
        self._action = ActionClient(self.node, NavigateToPose, ACTION)
        self._costmap_sub: Any = None
        self.costmap_info: dict[str, Any] | None = None
        self.costmap_gz = b''
        self.costmap_version = 0

    def _open(self) -> None:
        super()._open()
        from nav_msgs.msg import Path as PathMsg

        self._handles.append(self.node.create_subscription(PathMsg, '/plan', self._on_path, 10))

    # _open and _close run under self._lock (webui.IdleGate, from touch and the watch loop).
    def _close(self) -> None:
        super()._close()
        self._drop_costmap()
        self.path = []

    def _on_path(self, msg: Any) -> None:
        points = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        with self._lock:
            self.path = thin(points)
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

    # -- goals --

    def go(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        """Send a goal to Nav2; the result arrives in self.mission."""
        from nav2_msgs.action import NavigateToPose

        if not self._action.wait_for_server(timeout_sec=GOAL_WAIT_SEC):
            return False, 'Load a map first.'
        self.stop()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
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

    def stop(self) -> None:
        """Cancel the goal in progress, if any."""
        with self._lock:
            handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        out = super().snapshot(trail_from, trail_id)
        del out['trail']  # the Planner page draws the planned path, not the trail
        with self._lock:
            out['path'] = self.path
            out['path_version'] = self.path_version
            out['costmap_version'] = self.costmap_version if self.costmap_info else 0
        out['mission'] = self.mission.view()
        return out


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
        if path not in ('/api/goal', '/api/stop', '/api/costmap', '/api/action'):
            super().do_POST()
            return
        if self.refuse_cross_site():
            return
        try:
            body = self.read_json()
            if path == '/api/goal':
                self.viewer.touch()
                x, y, yaw = (float(body[k]) for k in ('x', 'y', 'yaw'))
                if not all(math.isfinite(v) for v in (x, y, yaw)):
                    raise ValueError('not a pose')
                ok, message = self.viewer.go(x, y, yaw)
                self.send_json({'ok': ok, 'message': message}, 200 if ok else 409)
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
