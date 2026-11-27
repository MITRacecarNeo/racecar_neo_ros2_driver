"""
Live sensor feeds for the web dashboard's Overview section.

Subscriptions open on the first request and close IDLE_SEC after the last, so
the dashboard costs nothing extra while nobody watches. Each camera feed opens
by itself, only while a page asks for its frames, and closes CAMERA_IDLE_SEC
after the last one. Callbacks keep only the latest serialized message;
decoding and JPEG encoding happen when a page asks, at the page's rate rather
than the sensor's. The color frame also goes to the camera panel of the Mapper
and the Planner (web/camera_panel.js), so they need no subscription of their own.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from webui import IdleGate  # noqa: E402

IDLE_SEC = 10.0
CAMERA_IDLE_SEC = 3.0
CAMERAS = ('color', 'depth')
GRAPH_SEC = 30.0
GRAPH_HZ = 10.0
PREVIEW_WIDTH = 320
PANEL_WIDTH = 480
PANEL_MAX_AGE_SEC = 0.1
# A frame older than this is not served: the camera has stopped. With STALE_MS in
# web/camera_panel.js the panel grays out within a second of the last frame.
STALE_SEC = 0.3
JPEG_QUALITY = 60
SCAN_MAX_M = 8.0
SCAN_STRIDE = 3
DEPTH_FAR_M = 4.0

# name: (topic, message type path)
FEEDS = {
    'color': ('/camera/color', 'sensor_msgs/msg/Image'),
    'depth': ('/camera/depth', 'sensor_msgs/msg/Image'),
    'scan': ('/scan', 'sensor_msgs/msg/LaserScan'),
    'imu': ('/imu/fused', 'sensor_msgs/msg/Imu'),
    'encoder': ('/encoder/speed', 'std_msgs/msg/Float32'),
}


def _msg_class(path: str) -> Any:
    pkg, _, name = path.split('/')
    return getattr(__import__(f'{pkg}.msg', fromlist=[name]), name)


def scan_points(
    ranges: list[float], angle_min: float, angle_inc: float, mount: tuple[float, float, float]
) -> list[list[float]]:
    """Return lidar returns as [x, y] in base_link (x forward, y left), thinned."""
    lx, ly, yaw = mount
    out = []
    for i in range(0, len(ranges), SCAN_STRIDE):
        r = ranges[i]
        if not math.isfinite(r) or r <= 0.0 or r > SCAN_MAX_M:
            continue
        a = angle_min + i * angle_inc + yaw
        out.append([round(lx + r * math.cos(a), 2), round(ly + r * math.sin(a), 2)])
    return out


class Overview:
    """On-demand feeds: latest frames, a thinned scan, and 30 s IMU and speed graphs."""

    def __init__(
        self,
        node: Any,
        deserialize: Callable[[bytes, Any], Any] | None = None,
        config_dir: Path | None = None,
    ) -> None:
        if deserialize is None:
            from rclpy.serialization import deserialize_message

            deserialize = deserialize_message
        self.node = node
        self._deserialize = deserialize
        self._lock = threading.Lock()
        self._handles: dict[str, Any] = {}
        self._latest: dict[str, tuple[bytes, float]] = {}
        self._jpeg: dict[tuple[str, int], tuple[bytes, float]] = {}  # by feed and width
        self.imu: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.speed: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        others = [n for n in FEEDS if n not in CAMERAS]
        self.gate = IdleGate(IDLE_SEC, lambda: self._open(others), self._close)
        self.cameras = {name: self._camera_gate(name) for name in CAMERAS}
        from racecar_neo_ros2_driver import frames

        values = frames.current() if config_dir is None else frames.load_frames(config_dir)
        self._mount = frames.laser_mount(values)
        threading.Thread(target=self._sample, daemon=True).start()

    # -- subscriptions --

    def _camera_gate(self, name: str) -> IdleGate:
        return IdleGate(CAMERA_IDLE_SEC, lambda: self._open([name]), lambda: self._drop([name]))

    @property
    def active(self) -> bool:
        return self.gate.open

    def touch(self) -> None:
        with self._lock:
            self.gate.touch()

    def touch_camera(self, name: str) -> None:
        with self._lock:
            self.cameras[name].touch()

    def _open(self, names: list[str]) -> None:
        from rclpy.qos import qos_profile_sensor_data

        for name in names:
            topic, type_path = FEEDS[name]
            self._handles[name] = self.node.create_subscription(
                _msg_class(type_path),
                topic,
                self._store(name),
                qos_profile_sensor_data,
                raw=True,
            )

    def _drop(self, names: list[str]) -> None:
        for name in names:
            h = self._handles.pop(name, None)
            if h is not None:
                self.node.destroy_subscription(h)
            self._latest.pop(name, None)
            for key in [k for k in self._jpeg if k[0] == name]:
                del self._jpeg[key]

    def _close(self) -> None:
        self._drop([n for n in self._handles if n not in CAMERAS])
        self.imu.clear()
        self.speed.clear()

    def _store(self, name: str) -> Callable[[bytes], None]:
        def cb(data: bytes) -> None:
            self._latest[name] = (data, time.monotonic())

        return cb

    def _decode(self, name: str) -> tuple[Any, float] | None:
        item = self._latest.get(name)
        if item is None:
            return None
        return self._deserialize(item[0], _msg_class(FEEDS[name][1])), item[1]

    # -- graph sampling --

    def _sample(self) -> None:
        while True:
            time.sleep(1.0 / GRAPH_HZ)
            with self._lock:
                for gate in self.cameras.values():
                    gate.check()
                if not self.gate.check():
                    continue
                now = time.time()
                imu = self._decode('imu')
                if imu is not None:
                    g = imu[0].angular_velocity
                    rates = [round(math.degrees(v), 1) for v in (g.x, g.y, g.z)]
                    self.imu.append([round(now, 2), *rates])
                enc = self._decode('encoder')
                if enc is not None:
                    self.speed.append([round(now, 2), round(float(enc[0].data), 3)])

    # -- page data --

    def state(self) -> dict[str, Any]:
        """Scan points, graph histories, and the age of each feed."""
        with self._lock:
            now = time.monotonic()
            ages = {k: round(now - v[1], 1) for k, v in self._latest.items()}
            scan = self._decode('scan')
            points = (
                scan_points(
                    list(scan[0].ranges), scan[0].angle_min, scan[0].angle_increment, self._mount
                )
                if scan
                else []
            )
            return {
                'now': time.time(),
                'ages': ages,
                'scan': points,
                'imu': list(self.imu),
                'speed': list(self.speed),
                'graph_sec': GRAPH_SEC,
            }

    def jpeg(
        self, name: str, max_age: float = 0.25, width: int = PREVIEW_WIDTH
    ) -> bytes | None:
        """
        Return a JPEG of the latest color or depth frame; cached for max_age s.

        None without a frame, or when the last one is over STALE_SEC old.
        """
        from racecar_neo_ros2_driver import imaging

        with self._lock:
            item = self._latest.get(name)
            if item is None or time.monotonic() - item[1] > STALE_SEC:
                return None
            cached = self._jpeg.get((name, width))
            if cached and time.monotonic() - cached[1] < max_age:
                return cached[0]
            decoded = self._decode(name)
        if decoded is None:
            return None
        msg = decoded[0]
        if name == 'depth':
            data = imaging.depth_preview(msg, width, DEPTH_FAR_M, JPEG_QUALITY)
        else:
            img = imaging.bgr(msg)
            data = (
                None
                if img is None
                else imaging.jpeg(imaging.fit_width(img, width), JPEG_QUALITY)
            )
        if data is None:
            return None
        with self._lock:
            self._jpeg[name, width] = (data, time.monotonic())
        return data

    def panel_jpeg(self) -> bytes | None:
        """Return the color frame for the camera panel (web/camera_panel.js)."""
        return self.jpeg('color', PANEL_MAX_AGE_SEC, PANEL_WIDTH)
