"""
Live sensor feeds for the web dashboard's Overview section.

Subscriptions open on the first request and close IDLE_SEC after the last, so
the dashboard costs nothing extra while nobody watches. Callbacks keep only
the latest serialized message; decoding and JPEG encoding happen when the
page asks, at the page's rate rather than the sensor's.
"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable

import numpy as np

IDLE_SEC = 10.0
GRAPH_SEC = 30.0
GRAPH_HZ = 10.0
PREVIEW_WIDTH = 320
JPEG_QUALITY = 60
SCAN_MAX_M = 8.0
SCAN_STRIDE = 3
DEPTH_NEAR_M = 0.2
DEPTH_FAR_M = 4.0
# Pixels with no depth: flat gray, so the stereo blind strip on the left and
# floor closer than the minimum range do not read as a dark object.
NO_DEPTH_BGR = (102, 81, 80)

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


def image_array(msg: Any) -> np.ndarray | None:
    """Return a sensor_msgs/Image as an HxWxC (color) or HxW (depth) array."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint16 if msg.encoding == '16UC1' else np.uint8)
    if msg.encoding == '16UC1':
        return buf.reshape(h, msg.step // 2)[:, :w]
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        return buf.reshape(h, msg.step)[:, :w]
    return None


def colorize_depth(depth_mm: np.ndarray, far_m: float = DEPTH_FAR_M) -> np.ndarray:
    """Return a BGR preview of depth: bright is near, dark is far, gray is no reading."""
    import cv2

    metres = depth_mm.astype(np.float32) / 1000.0
    valid = metres > 0
    norm = np.clip((metres - DEPTH_NEAR_M) / max(far_m - DEPTH_NEAR_M, 1e-3), 0.0, 1.0)
    ramp = (40 + (1.0 - norm) * 215).astype(np.uint8)
    out = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    out[~valid] = NO_DEPTH_BGR
    return out


def scan_points(ranges: list[float], angle_min: float, angle_inc: float,
                mount: tuple[float, float, float]) -> list[list[float]]:
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


def laser_mount_from_config(config_dir: Path | None) -> tuple[float, float, float]:
    """Return the lidar's (x, y, yaw) in base_link from frames.yaml, or the origin."""
    try:
        from racecar_neo_ros2_driver.frames import load_frames

        if config_dir is None:
            from ament_index_python.packages import get_package_share_directory

            config_dir = Path(get_package_share_directory('racecar_neo_ros2_driver')) / 'config'
        f = load_frames(config_dir)
        return (f.get('laser_x_mm', 0.0) / 1000.0, f.get('laser_y_mm', 0.0) / 1000.0,
                math.radians(f.get('laser_yaw_deg', 0.0)))
    except Exception:  # noqa: BLE001  the view still draws, unrotated
        return (0.0, 0.0, 0.0)


class Overview:
    """On-demand feeds: latest frames, a thinned scan, and 30 s IMU and speed graphs."""

    def __init__(self, node: Any, deserialize: Callable[[bytes, Any], Any] | None = None,
                 config_dir: Path | None = None) -> None:
        if deserialize is None:
            from rclpy.serialization import deserialize_message as deserialize
        self.node = node
        self._deserialize = deserialize
        self._lock = threading.Lock()
        self._handles: list[Any] = []
        self._latest: dict[str, tuple[bytes, float]] = {}
        self._jpeg: dict[str, tuple[bytes, float]] = {}
        self.imu: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.speed: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.active = False
        self.last_request = 0.0
        self._mount = laser_mount_from_config(config_dir)
        threading.Thread(target=self._sample, daemon=True).start()

    # -- subscriptions --

    def touch(self) -> None:
        with self._lock:
            self.last_request = time.monotonic()
            if not self.active:
                self._open()

    def _open(self) -> None:
        from rclpy.qos import qos_profile_sensor_data

        for name, (topic, type_path) in FEEDS.items():
            self._handles.append(self.node.create_subscription(
                _msg_class(type_path), topic, self._store(name), qos_profile_sensor_data, raw=True,
            ))
        self.active = True

    def _close(self) -> None:
        for h in self._handles:
            self.node.destroy_subscription(h)
        self._handles = []
        self._latest.clear()
        self._jpeg.clear()
        self.imu.clear()
        self.speed.clear()
        self.active = False

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
                if self.active and time.monotonic() - self.last_request > IDLE_SEC:
                    self._close()
                if not self.active:
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
            points = scan_points(list(scan[0].ranges), scan[0].angle_min, scan[0].angle_increment,
                                 self._mount) if scan else []
            return {'now': time.time(), 'ages': ages, 'scan': points,
                    'imu': list(self.imu), 'speed': list(self.speed), 'graph_sec': GRAPH_SEC}

    def jpeg(self, name: str, max_age: float = 0.25) -> bytes | None:
        """Return a small JPEG of the latest color or depth frame; cached for max_age s."""
        import cv2

        with self._lock:
            cached = self._jpeg.get(name)
            if cached and time.monotonic() - cached[1] < max_age:
                return cached[0]
            decoded = self._decode(name)
        if decoded is None:
            return None
        img = image_array(decoded[0])
        if img is None:
            return None
        if name == 'depth':
            img = colorize_depth(img)
        scale = PREVIEW_WIDTH / img.shape[1]
        small = cv2.resize(img, (PREVIEW_WIDTH, max(1, int(img.shape[0] * scale))),
                           interpolation=cv2.INTER_AREA)
        data = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[1].tobytes()
        with self._lock:
            self._jpeg[name] = (data, time.monotonic())
        return data
