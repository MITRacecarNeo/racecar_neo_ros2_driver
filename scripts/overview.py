"""
Live sensor feeds for the web dashboard's Overview section.

Subscriptions open on the first request and close IDLE_SEC after the last, so
the dashboard costs nothing extra while nobody watches. The two camera feeds,
two thirds of the cost, open only while the page asks for frames and close
CAMERA_IDLE_SEC after the last one. Callbacks keep only
the latest serialized message; decoding and JPEG encoding happen when the
page asks, at the page's rate rather than the sensor's.
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
        self._jpeg: dict[str, tuple[bytes, float]] = {}
        self.imu: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.speed: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        others = [n for n in FEEDS if n not in CAMERAS]
        cameras = [n for n in CAMERAS if n in FEEDS]
        self.gate = IdleGate(IDLE_SEC, lambda: self._open(others), self._close)
        self.cameras = IdleGate(
            CAMERA_IDLE_SEC, lambda: self._open(cameras), lambda: self._drop(cameras)
        )
        from racecar_neo_ros2_driver import frames

        values = frames.current() if config_dir is None else frames.load_frames(config_dir)
        self._mount = frames.laser_mount(values)
        threading.Thread(target=self._sample, daemon=True).start()

    # -- subscriptions --

    @property
    def active(self) -> bool:
        return self.gate.open

    def touch(self) -> None:
        with self._lock:
            self.gate.touch()

    def touch_cameras(self) -> None:
        with self._lock:
            self.gate.touch()
            self.cameras.touch()

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
            self._jpeg.pop(name, None)

    def _close(self) -> None:
        self._drop(list(self._handles))
        self._latest.clear()
        self._jpeg.clear()
        self.imu.clear()
        self.speed.clear()
        self.cameras.open = False

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
                if not self.gate.check():
                    continue
                self.cameras.check()
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

    def jpeg(self, name: str, max_age: float = 0.25) -> bytes | None:
        """Return a small JPEG of the latest color or depth frame; cached for max_age s."""
        from racecar_neo_ros2_driver import imaging

        with self._lock:
            cached = self._jpeg.get(name)
            if cached and time.monotonic() - cached[1] < max_age:
                return cached[0]
            decoded = self._decode(name)
        if decoded is None:
            return None
        msg = decoded[0]
        if name == 'depth':
            data = imaging.depth_preview(msg, PREVIEW_WIDTH, DEPTH_FAR_M, JPEG_QUALITY)
        else:
            img = imaging.bgr(msg)
            data = (
                None
                if img is None
                else imaging.jpeg(imaging.fit_width(img, PREVIEW_WIDTH), JPEG_QUALITY)
            )
        if data is None:
            return None
        with self._lock:
            self._jpeg[name] = (data, time.monotonic())
        return data
