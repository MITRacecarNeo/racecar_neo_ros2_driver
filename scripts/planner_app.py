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
missions are saved beside their map. The navigator (nav_server_node.py) drives
it; this server sends it commands and shows its state (nav_link.py). Closing
the app stops a mission started from it.

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

import gzip
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse
import uuid

import mapview
import mission_recorder
import numpy as np
from racecar_neo_ros2_driver import apps, mission_data, missions, nav_link
from racecar_neo_ros2_driver.geometry import yaw_of
from racecar_neo_ros2_driver.maps import valid_name
from racecar_neo_ros2_driver.nav_runner import legs, NO_MAP, Pose
import webui

HERE = Path(__file__).resolve().parent
PORT = apps.port('planner')
PAGE = HERE / 'planner.html'
NODE = 'planner'
PREVIEW_ACTION = 'compute_path_to_pose'
# Map actions the Planner offers: load a map, and save or stop a mapping run it finds.
PLANNER_ACTIONS = ('load', 'save', 'stop')
PATH_POINTS = 400
SERVER_WAIT_SEC = 2.0
PREVIEW_WAIT_SEC = 5.0
NO_SUMMARY = 'Could not read the recording.'
BUSY = 'Another change is running.'
WATCH_SEC = 0.5
LAST_EVENT_SEC = 0.5  # the navigator's last events, after the state that ends a mission
STATUS_SEC = 1.0
FREE_CHECK_SEC = 5.0
STEP_M = (0.005, 0.5)  # pose steps that count as driving, not noise or a new pose
PLAN_POINTS = 100
PLAN_END_M = 0.5
MISSION_NAME_RULE = 'Mission names use letters, digits, - and _.'
RELOAD_TEXT = mapview.RELOAD


def thin(points: list[list[float]], limit: int = PATH_POINTS) -> list[list[float]]:
    """Return at most `limit` points of a path, always keeping both ends."""
    if len(points) <= limit:
        return points
    step = (len(points) - 1) / (limit - 1)
    return [points[round(i * step)] for i in range(limit)]


def path_points(msg: Any) -> list[list[float]]:
    """Return a nav_msgs/Path as [x, y] pairs, rounded to the millimeter and thinned."""
    return thin([[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses])


class Navigator(mapview.Viewer):
    """The Mapper's viewer, plus Nav2: missions, the path preview, the costmap on request."""

    def __init__(self) -> None:
        super().__init__(NODE)
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import ComputePathToPose
        from racecar_neo_ros2_driver.qos import LATCHED
        from rclpy.action import ActionClient

        self.link = nav_link.NavLink(self.node)
        self.recorder = mission_recorder.Recorder()
        self.recording = False
        self.not_recording = False  # wanted, but it could not start or ended early
        self._events: mission_data.EventLog | None = None
        self._log_key = ''  # marks this recording's events
        self._event_sub: Any = None
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
        self._preview = ActionClient(self.node, ComputePathToPose, PREVIEW_ACTION)
        self._costmap_sub: Any = None
        self.costmap_info: dict[str, Any] | None = None
        self.costmap_gz = b''
        self.costmap_version = 0
        # Held for the whole run, not only while a page polls: missions are saved
        # under the map's name, and a recording measures the distance.
        self.map_name = ''
        self._pose: Pose | None = None
        self.node.create_subscription(
            DiagnosticStatus, '/autonomy/status', self._on_mission_status, LATCHED
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/autonomy/pose', self._on_mission_pose, 10
        )

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
        if not self._preview.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            return False, NO_MAP
        return True, [self._plan(start, goal) for start, goal in legs(mission, self._pose)]

    # -- missions --

    def start(self, mission: dict[str, Any], label: str = '') -> tuple[bool, str]:
        """Run a mission; with its Record toggle on, once the recorder is ready."""
        if not self._start_lock.acquire(blocking=False):
            return False, BUSY
        try:
            self._abort, self.not_recording = False, False
            ok, problem = self.link.command('check')
            if not ok:
                return False, problem
            self._label = label or self.map_name
            self._end_recording()
            if mission['record']['on']:
                self._begin_recording(mission)
                if self._abort:  # up to a few seconds have passed
                    self._end_recording(summary=False)
                    return False, 'Stopped.'
            ok, problem = self.link.command(
                'start', owner=NODE, name=apps.app(NODE)['name'], mission=mission,
                resumable=True, log=self._log_key if self.recording else '',
            )  # fmt: skip
            if not ok:
                self._end_recording(summary=False)
                return False, problem
            if self.recording:
                threading.Thread(target=self._watch_recording, daemon=True).start()
            if self._abort:  # Stop went out just before the start
                self.control('stop')
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
            from std_msgs.msg import String

            self._dist, self._want_plan = 0.0, None
            with self._events_lock:
                self._events = mission_data.EventLog(folder, mission)
            self._log_key = uuid.uuid4().hex
            with self._lock:
                self._plan_sub = self.node.create_subscription(
                    PathMsg, '/plan', self._on_leg_plan, 10
                )
                self._event_sub = self.node.create_subscription(
                    String, nav_link.EVENT_TOPIC, self._on_event, nav_link.event_qos()
                )
            self.recording = True
            if underway:
                self.link.command('log', log=self._log_key)
            else:
                live = {t for t, _ in self.node.get_topic_names_and_types()}
                ready = [t for t in wanted if t in live]
                self.recorder.wait_ready(ready, lambda: self._abort)

    def _end_recording(self, summary: bool = True) -> None:
        with self._rec_lock:
            if not self.recording:
                return
            if self.link.underway():
                self.link.command('log', log='')
            self._log_key = ''
            with self._lock:
                for sub in (self._plan_sub, self._event_sub):
                    if sub is not None:
                        self.node.destroy_subscription(sub)
                self._plan_sub = self._event_sub = None
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
        key = self._log_key
        while self.recording and self._log_key == key:
            time.sleep(WATCH_SEC)
            if time.monotonic() - shown >= STATUS_SEC:
                shown = time.monotonic()
                self.rec_status = self.recorder.status(mission_data.min_free_bytes(config))
            view = self.link.view()
            wanted = view['mission'] is not None and view['mission']['record']['on']
            if not self.link.underway() or not wanted:
                time.sleep(LAST_EVENT_SEC)
                if self._log_key == key:
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

    def _on_event(self, msg: Any) -> None:
        """Log an event of the mission, if it is of this recording."""
        e = nav_link.decode(msg.data)
        if e is None or not self._log_key or e.get('log') != self._log_key:
            return
        if e['event'] == 'sent':
            self._want_plan = (e['x'], e['y'], e['loop'], e['waypoint'])
        fields = {k: v for k, v in e.items() if k not in ('event', 't', 'log', 'pose')}
        with self._events_lock:
            if self._events is not None:
                self._events.add(
                    e['event'], e['t'], **fields, pose=e.get('pose'), dist_m=round(self._dist, 2)
                )

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
        """Stop the mission started here and finish the bag before the node goes."""
        if self.link.underway() and self.link.view()['owner'] == NODE:
            self.control('stop')
        self._end_recording(summary=False)
        super().shutdown()

    def control(self, verb: str) -> tuple[bool, str]:
        """Pause, resume, skip or stop the mission."""
        if verb == 'stop':
            self._abort = True  # also a start still waiting for the recorder
        if not self.link.alive:
            return True, ''  # no navigator, no mission
        return self.link.command(verb)

    def edit(self, mission: dict[str, Any]) -> tuple[bool, str]:
        ok, message = self.link.command('edit', mission=mission)
        if ok and mission['record']['on'] and self.link.underway() and not self.recording:
            threading.Thread(target=self._record_from_here, args=(mission,), daemon=True).start()
        return ok, message

    def _record_from_here(self, mission: dict[str, Any]) -> None:
        self.not_recording = False
        self._begin_recording(mission, underway=True)
        self._watch_recording()

    def _on_mission_status(self, msg: Any) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        localizing = values.get('mode') == 'localization'
        self.map_name = values.get('map', '') if localizing else ''

    def _on_mission_pose(self, msg: Any) -> None:
        p = msg.pose.pose
        before, self._pose = self._pose, (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.recording and before is not None:
            step = math.hypot(self._pose[0] - before[0], self._pose[1] - before[1])
            if STEP_M[0] <= step <= STEP_M[1]:
                self._dist += step

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        out = super().snapshot(trail_from, trail_id)
        del out['trail']  # the Planner page draws the planned path, not the trail
        with self._lock:
            out['path'] = self.path
            out['path_version'] = self.path_version
            out['costmap_version'] = self.costmap_version if self.costmap_info else 0
        out['mission'] = {
            **self.link.view(),
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

    def _answer(self, ok: bool, message: str) -> None:
        self.send_json({'ok': ok, 'message': message}, 200 if ok else 409)

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
                self._answer(*v.start(mission, label))
            elif path == '/api/edit':
                self._answer(*v.edit(missions.parse(body.get('mission'))))
            elif path == '/api/control':
                verb = body.get('verb')
                if verb not in ('pause', 'resume', 'skip', 'stop'):
                    raise ValueError('not a mission control')
                self._answer(*v.control(verb))
            elif path == '/api/stop':
                self._answer(*v.control('stop'))
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
