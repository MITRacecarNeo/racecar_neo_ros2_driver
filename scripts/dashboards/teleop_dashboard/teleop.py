#!/usr/bin/env python3
"""
RACECAR Neo web teleop service: drive the car from a browser.

Buttons or the keyboard drive, with every sensor in view: camera, depth,
lidar, encoder, IMU, battery, and compass, plus editors for the dot matrix
and the LED strip.

The camera card carries both D435i streams: the color frame over a colorized
depth frame. Depth is optional; a car whose camera has no depth stream leaves
depth_topic empty and the card shows the color frame alone.

A Detections toggle overlays the object detector's boxes on the color frame.
Boxes are published in source-image pixels and leave here normalized, so the
browser scales them to whatever size it is drawing the preview at.

Same pattern as the other lab dashboards (stdlib HTTP + rclpy) on port
8081. The browser holds a command of two axes, forward/back and left/right,
each -1, 0, or +1, from whichever buttons or keys are down at once, and
posts it on every change and every 100 ms while anything is held:

  throttle = fwd  * speed      (fwd  = +1 forward, -1 back)
  steering = turn * angle      (turn = +1 right,  -1 left; student convention)

/drive is published at a fixed rate from a timer. A command older than
cmd_timeout is treated as released, so a closed tab, a dropped link, or a
page that lost focus with a key down all coast to zero rather than driving
the last command forever. That is the one thing this dashboard adds over
the student library's rc.drive.set_speed_angle.

CAUTION: the RACECAR Neo mux gates /drive on the RB bumper and zeroes
output when /joy or the active source goes stale. The shipped yaml has
speed at 0.0 so the buttons do nothing until the slider is raised.

/drive steering is negated on publish: positive on the wire turns this car
left (physically verified on the sibling dashboards). Speed feedback comes
from /odom.

The IMU, magnetometer, dot matrix frame, and LED strip are read from the same
topics as the student library (rc.physics, rc.display, rc.led). Their
callbacks keep only the latest serialized message; the timer decodes it at
the publish rate. Dot matrix and LED frames sent from the page are re-sent
while the page is open, because the driver blanks both shortly after the last
message; they stop DISPLAY_HOLD_S after the page stops polling.
"""

from collections import deque
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time
from typing import Any

from ackermann_msgs.msg import AckermannDriveStamped
import cv2
from nav_msgs.msg import Odometry
import numpy as np
from racecar_neo_ros2_driver import car_profile, data_root
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import BatteryState, Image, Imu, LaserScan, MagneticField
from std_msgs.msg import String, UInt8MultiArray

try:
    from vision_msgs.msg import Detection2DArray
except ImportError:
    # A car without vision_msgs still gets every other view; the Detections
    # toggle reports itself unavailable rather than taking the page down.
    Detection2DArray = None
import yaml

PORT = 8081
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import webui  # noqa: E402, I100

# Shipped defaults stay untouched; saved tuning goes to the per-car file,
# which loads on top of them, in the car profile (dashboards/<name>.yaml).
DEFAULTS_PATH = BASE / 'teleop.yaml'
YAML_PATH = car_profile.dashboard_settings('teleop')
LOG_DIR = data_root.current()[0] / 'dashboards' / 'teleop'

MAX_SPEED = 1.0  # hard throttle cap either way, as drive_real.set_max_speed
PUBLISH_RATE_HZ = 15.0  # /drive rate, independent of the browser
PREVIEW_RATE_HZ = 15.0
# The live chart is 420 px wide; send an even sample of the history rather
# than every row. The log csv keeps all of them.
HIST_POINTS = 300

TUNE_KEYS = ('speed', 'angle')

# Raw scan angle that points at the car's nose, in degrees.
#
# The RACECAR Neo mounts its RPLIDAR facing aft over 360 degrees at 1080
# points, so the nose sits at the raw +/-180 edge of the sweep rather than at
# 0. Measured on hardware: an object in front reports at raw 180, and one off
# the car's right at raw +90.
#
#     student_deg = LIDAR_MOUNT_YAW_DEG - raw_deg
#
# The view read 180 degrees out before this constant existed, which is the
# v0.8.0 known limitation: the plain negation below it assumed the neoracer's
# forward-facing 270 degree mount. Set this to 0.0 for such a lidar. Nothing
# here assumes a sweep width; the angle comes from the message's own
# angle_min and angle_increment. Same convention and constant as
# wallfollow.py, which steers on it.
LIDAR_MOUNT_YAW_DEG = 180.0

# Depth coloring. Near reads bright and far reads dark, on the perceptually
# uniform inferno ramp: a single ordered scale, so a step in color is a step
# in distance, and it survives being read by a color blind student. The ramp
# starts at DEPTH_FLOOR rather than 0 so the darkest real reading is still
# above black. No return is flat gray, so the stereo blind strip on the left
# and floor closer than the minimum range do not read as a dark object.
DEPTH_FLOOR = 24
DEPTH_NEAR_M = 0.15  # below the D435i minimum range; nothing valid is nearer
NO_RETURN_BGR = (102, 81, 80)

# Detections older than this are dropped rather than left frozen on the
# frame. The overlay is only honest if a stale box disappears.
DET_TIMEOUT_S = 1.5

# Sensor charts: the last SENSE_S seconds at the publish rate.
SENSE_S = 10.0
MATRIX_ROWS, MATRIX_COLS = 8, 24
NUM_LEDS = 84
# Re-send periods inside the driver's timeouts (dot matrix pixels 5 s, text
# 6 s, LED frame 0.5 s), and how long a frame is held after the page stops.
MATRIX_RESEND_S = 1.0
LED_RESEND_S = 0.2
DISPLAY_HOLD_S = 3.0

_lock = threading.Lock()
_params: dict = {}
_state: dict = {
    'fwd': 0,
    'turn': 0,
    'held': False,
    'timed_out': False,
    'speed_cmd': 0.0,
    'steer': 0.0,
    'enc_speed': 0.0,
    'scan': [],
    'res': [0, 0],
    'cam_fps': 0.0,
    'depth_res': [0, 0],
    'depth_fps': 0.0,
    'dets': [],
    'det_fps': 0.0,
    'det_ok': False,
    'hist': [],
    'marks': [],
}
_preview = b''
_depth_preview = b''
# The browser's command and when it last arrived. Read by the timer.
_cmd: dict = {'fwd': 0, 'turn': 0, 'stamp': 0.0}
_hist: deque = deque(maxlen=1200)
# Full rows behind the live chart. POST /logs/save snapshots this to a csv,
# so a saved log is exactly what was on the screen.
_rows: deque = deque(maxlen=1200)
# Parameter-change markers [t, label] shown on the charts. Saved into logs
# as "# mark" lines.
_marks: deque = deque(maxlen=40)
_sense: deque = deque(maxlen=int(SENSE_S * PUBLISH_RATE_HZ))
# Latest serialized messages, decoded by the timer: {name: (bytes, stamp)}.
_raw: dict = {}
# What the page asked the dot matrix and LED strip to show. kind is
# 'pixels' or 'text' for the matrix; leds is 252 bytes or None.
_display: dict = {'kind': None, 'pixels': [], 'text': '', 'leds': None}
_last_poll = 0.0
_T0 = time.monotonic()


def _now() -> float:
    return round(time.monotonic() - _T0, 2)


def _add_mark(label: str) -> None:
    # Coalesce rapid updates: the sliders fire while they are dragged.
    if (
        _marks
        and _now() - _marks[-1][0] < 1.0
        and _marks[-1][1].split(' ')[0] == label.split(' ')[0]
    ):
        _marks[-1] = [_now(), label]
    else:
        _marks.append([_now(), label])


def clamp_speed(speed: float) -> float:
    """
    Clamp throttle to [-MAX_SPEED, MAX_SPEED].

    Every speed that leaves this program passes through here, so /drive can
    never carry more than 1.0 in either direction.
    """
    return max(-MAX_SPEED, min(MAX_SPEED, float(speed)))


def _clean(p: dict[str, Any]) -> None:
    """Clamp every tunable into its range. Caller holds _lock."""
    p['speed'] = max(0.0, clamp_speed(p['speed']))
    p['angle'] = max(0.0, min(1.0, float(p['angle'])))
    p['cmd_timeout'] = max(0.1, float(p.get('cmd_timeout', 0.5)))
    p['depth_max_m'] = max(0.5, float(p.get('depth_max_m', 4.0)))


def load_params() -> None:
    with _lock:
        before = json.dumps(_params, sort_keys=True)
        _params.update(yaml.safe_load(DEFAULTS_PATH.read_text()))
        if YAML_PATH.exists():
            _params.update(yaml.safe_load(YAML_PATH.read_text()) or {})
        _clean(_params)
        if before != '{}' and before != json.dumps(_params, sort_keys=True):
            _add_mark('yaml load')


def save_params() -> None:
    """Rewrite the numeric values in place so the yaml comments survive."""
    if not YAML_PATH.exists():
        car_profile.write_text(YAML_PATH, DEFAULTS_PATH.read_text())
    scalar = re.compile(r'^([A-Za-z_]\w*):([ \t]*)[-\d.eE+]+(.*)$')
    out = []
    with _lock:
        for raw in YAML_PATH.read_text().splitlines(keepends=True):
            m = scalar.match(raw.rstrip('\n'))
            if m:
                key, gap, tail = m.groups()
                v = _params.get(key)
                if isinstance(v, (int, float)):
                    out.append(f'{key}:{gap}{v}{tail}\n')
                    continue
            out.append(raw)
    car_profile.write_text(YAML_PATH, ''.join(out))


def update_params(data: dict[str, Any]) -> list[str]:
    """Apply a posted subset of TUNE_KEYS. Returns the change labels."""
    with _lock:
        before = dict(_params)
        for k in TUNE_KEYS:
            if k in data:
                _params[k] = float(data[k])
        _clean(_params)
        changed = [f'{k} {_params[k]:g}' for k in TUNE_KEYS if before.get(k) != _params[k]]
        if changed:
            _add_mark(', '.join(changed))
    return changed


def _axis(v: Any) -> int:
    """Squash whatever the browser sent to -1, 0, or +1."""
    v = int(v)
    return (v > 0) - (v < 0)


def heading_deg(mag: list[float]) -> float:
    """
    Return the compass heading from the magnetic field in body axes.

    0 faces magnetic north, increasing counterclockwise, as the simulator's
    dashboard.
    """
    return math.degrees(math.atan2(-mag[1], mag[0]))


def matrix_pixels(values: Any) -> list[int]:
    """192 values (0/1, row-major) from whatever the page sent."""
    flat = [1 if v else 0 for v in list(values)[: MATRIX_ROWS * MATRIX_COLS]]
    return flat + [0] * (MATRIX_ROWS * MATRIX_COLS - len(flat))


def led_bytes(colors: Any) -> bytes:
    """NUM_LEDS x RGB as the 252 bytes pit_node expects, clamped to 0..255."""
    out: list[int] = []
    for c in list(colors)[:NUM_LEDS]:
        out.extend(max(0, min(255, int(v))) for v in (list(c) + [0, 0, 0])[:3])
    return bytes(out + [0] * (NUM_LEDS * 3 - len(out)))


def set_display(kind: str, value: Any) -> None:
    """Record a dot matrix or LED request from the page; the timer sends it."""
    with _lock:
        if kind == 'pixels':
            _display.update(kind='pixels', pixels=matrix_pixels(value))
        elif kind == 'text':
            _display.update(kind='text', text=str(value)[:255])
        elif kind == 'leds':
            _display['leds'] = led_bytes(value)
        elif kind == 'release':
            _display.update(kind=None, leds=None)


def set_cmd(data: dict[str, Any], now: float | None = None) -> None:
    """
    Take the browser's command: {'fwd': -1|0|1, 'turn': -1|0|1}.

    Both axes together are one command, so forward and left arrive as one
    post and are applied at the same instant.
    """
    with _lock:
        _cmd['fwd'] = _axis(data.get('fwd', 0))
        _cmd['turn'] = _axis(data.get('turn', 0))
        _cmd['stamp'] = time.monotonic() if now is None else now


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('webteleop')
        with _lock:
            cam_topic = _params['camera_topic']
            depth_topic = _params.get('depth_topic') or ''
            det_topic = _params.get('detections_topic') or ''
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(Image, cam_topic, self._image_cb, qos_profile_sensor_data)
        # Depth is a separate stream on the same camera, and only the D435i
        # has one. An empty depth_topic is a car whose camera is color only;
        # subscribing to nothing would sit at 0 fps and read as a fault.
        if depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, qos_profile_sensor_data)
        # Detections are a bonus view, not a dependency: a car with no
        # detector, or no vision_msgs, simply never enables the toggle.
        self._det_available = bool(det_topic) and Detection2DArray is not None
        if self._det_available:
            self.create_subscription(
                Detection2DArray, det_topic, self._det_cb, qos_profile_sensor_data
            )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        # The library's topics, raw: the timer decodes only the newest message.
        for name, kind, topic in (
            ('imu', Imu, '/imu/fused'),
            ('mag', MagneticField, '/mag'),
            ('frame', UInt8MultiArray, '/dotmatrix/frame'),
            ('leds', UInt8MultiArray, '/led/pixels'),
        ):
            self.create_subscription(
                kind, topic, self._raw_cb(name), qos_profile_sensor_data, raw=True
            )
        self.create_subscription(BatteryState, '/battery/state', self._battery_cb, 10)
        self._pub_pixels = self.create_publisher(UInt8MultiArray, '/dotmatrix/pixels', 1)
        self._pub_text = self.create_publisher(String, '/dotmatrix/text', 1)
        self._pub_leds = self.create_publisher(UInt8MultiArray, '/led/pixels', 1)
        self._matrix_sent = 0.0
        self._leds_sent = 0.0
        self._battery: tuple[float, float] | None = None
        # /drive at a fixed rate whether or not the browser is talking, so
        # the mux never sees a gap and a silent browser reads as released.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._actuate)

        self._enc = 0.0
        self._speed_cmd = 0.0
        self._steer_cmd = 0.0
        self._res = (0, 0)
        self._last_preview = 0.0
        self._last_depth_preview = 0.0
        self._frame_stamps: list[float] = []
        self._depth_stamps: list[float] = []
        self._det_stamps: list[float] = []
        self._last_det = 0.0

    def _odom_cb(self, msg: Odometry) -> None:
        self._enc = msg.twist.twist.linear.x

    @staticmethod
    def _raw_cb(name: str) -> Callable[[bytes], None]:
        def cb(data: bytes) -> None:
            _raw[name] = (data, time.monotonic())

        return cb

    def _battery_cb(self, msg: BatteryState) -> None:
        # BatteryState counts discharge as negative; the page shows draw as positive.
        self._battery = (round(float(msg.voltage), 3), round(-float(msg.current), 3))

    def _decode(self, name: str, kind: Any) -> Any:
        item = _raw.get(name)
        return deserialize_message(item[0], kind) if item else None

    def _sample_sensors(self, t: float) -> dict[str, Any]:
        """Decode the newest IMU and magnetometer messages into one chart row."""
        imu, mag = self._decode('imu', Imu), self._decode('mag', MagneticField)
        state: dict[str, Any] = {}
        if imu is not None:
            a, g = imu.linear_acceleration, imu.angular_velocity
            state['accel'] = [round(a.x, 3), round(a.y, 3), round(a.z, 3)]
            state['gyro'] = [round(g.x, 4), round(g.y, 4), round(g.z, 4)]
        if mag is not None:
            f = mag.magnetic_field
            ut = [round(v * 1e6, 2) for v in (f.x, f.y, f.z)]
            state['mag'] = ut
            state['heading'] = round(heading_deg(ut), 1)
        if self._battery is not None:
            state['volts'], state['amps'] = self._battery
        if imu is not None or mag is not None:
            _sense.append(
                [
                    t,
                    *state.get('accel', [0.0] * 3),
                    *state.get('gyro', [0.0] * 3),
                    *state.get('mag', [0.0] * 3),
                ]
            )
        frame = self._decode('frame', UInt8MultiArray)
        if frame is not None:
            state['matrix'] = matrix_pixels(frame.data)
        leds = _raw.get('leds')
        if leds is not None and time.monotonic() - leds[1] < 1.0:
            data = bytes(self._decode('leds', UInt8MultiArray).data)
            state['leds'] = [list(data[i:][:3]) for i in range(0, min(len(data), NUM_LEDS * 3), 3)]
        else:
            state['leds'] = []
        return state

    def _send_display(self, now: float) -> None:
        """Re-send the page's dot matrix and LED frames while the page is open."""
        with _lock:
            d = dict(_display)
            alive = now - _last_poll < DISPLAY_HOLD_S
        if not alive:
            return
        if d['kind'] and now - self._matrix_sent >= MATRIX_RESEND_S:
            self._matrix_sent = now
            if d['kind'] == 'pixels':
                self._pub_pixels.publish(UInt8MultiArray(data=d['pixels']))
            else:
                self._pub_text.publish(String(data=d['text']))
        if d['leds'] is not None and now - self._leds_sent >= LED_RESEND_S:
            self._leds_sent = now
            self._pub_leds.publish(UInt8MultiArray(data=d['leds']))

    @staticmethod
    def _student_deg(msg: LaserScan, i: int) -> float:
        """Student angle of ray `i`: 0 the nose, positive the car's right."""
        raw = math.degrees(msg.angle_min + i * msg.angle_increment)
        return (LIDAR_MOUNT_YAW_DEG - raw + 180.0) % 360.0 - 180.0

    def _scan_cb(self, msg: LaserScan) -> None:
        # Every third ray is plenty for a 420 px view.
        scan = [
            [round(self._student_deg(msg, i), 1), round(msg.ranges[i], 3)]
            for i in range(0, len(msg.ranges), 3)
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]
        with _lock:
            _state['scan'] = scan

    @staticmethod
    def _tick_fps(stamps: list[float], now: float, key: str) -> list[float]:
        """
        Record a frame arrival and publish the stream's rate.

        The rate is over the last two seconds. Returns the trimmed stamp list.
        """
        stamps = [t for t in stamps if now - t < 2.0] + [now]
        with _lock:
            _state[key] = round(len(stamps) / 2.0, 1)
        return stamps

    def _image_cb(self, msg: Image) -> None:
        global _preview
        now = time.monotonic()
        self._frame_stamps = self._tick_fps(self._frame_stamps, now, 'cam_fps')
        if now - self._last_preview < 1.0 / PREVIEW_RATE_HZ:
            return
        self._last_preview = now
        if msg.encoding == 'jpeg':
            arr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr is None:
            return
        h, w = arr.shape[:2]
        self._res = (w, h)
        with _lock:
            p = dict(_params)
        pw = int(p['preview_width'])
        small = cv2.resize(arr, (pw, max(1, round(pw * h / w))))
        quality = [cv2.IMWRITE_JPEG_QUALITY, int(p['preview_quality'])]
        with _lock:
            _preview = cv2.imencode('.jpg', small, quality)[1].tobytes()
            _state['res'] = [w, h]

    def _depth_cb(self, msg: Image) -> None:
        """
        Colorize the depth frame for the browser.

        16UC1 is millimeters, the D435i default; 32FC1 is meters, which some
        drivers publish instead. Both reach the ramp as meters.
        """
        global _depth_preview
        now = time.monotonic()
        self._depth_stamps = self._tick_fps(self._depth_stamps, now, 'depth_fps')
        if now - self._last_depth_preview < 1.0 / PREVIEW_RATE_HZ:
            return
        self._last_depth_preview = now
        if msg.encoding in ('16UC1', 'mono16'):
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            meters = raw.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            meters = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            return
        h, w = meters.shape
        with _lock:
            p = dict(_params)
        pw = int(p['preview_width'])
        # Nearest neighbour, not area: averaging a valid reading against a
        # zero would invent a surface halfway to a hole in the depth image.
        small = cv2.resize(
            meters, (pw, max(1, round(pw * h / w))), interpolation=cv2.INTER_NEAREST
        )
        with _lock:
            _depth_preview = cv2.imencode(
                '.jpg',
                _colorize_depth(small, p['depth_max_m']),
                [cv2.IMWRITE_JPEG_QUALITY, int(p['preview_quality'])],
            )[1].tobytes()
            _state['depth_res'] = [w, h]

    def _det_cb(self, msg: Any) -> None:
        """
        Normalize the detector's boxes against the color frame.

        vision_msgs carries a center and a size in source-image pixels. The
        browser draws the preview at whatever width fits its column, so the
        boxes leave here as fractions of the frame and are scaled there.
        """
        now = time.monotonic()
        self._det_stamps = self._tick_fps(self._det_stamps, now, 'det_fps')
        self._last_det = now
        with _lock:
            w, h = _state['res']
        if not w or not h:
            return
        dets = []
        for d in msg.detections:
            cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
            bw, bh = d.bbox.size_x, d.bbox.size_y
            label, score = '', 0.0
            if d.results:
                label = d.results[0].hypothesis.class_id
                score = round(float(d.results[0].hypothesis.score), 3)
            dets.append(
                [
                    round((cx - bw / 2) / w, 4),
                    round((cy - bh / 2) / h, 4),
                    round(bw / w, 4),
                    round(bh / h, 4),
                    label,
                    score,
                ]
            )
        with _lock:
            _state['dets'] = dets

    def _expire_detections(self, now: float) -> None:
        """Clear the overlay when the detector stops publishing."""
        fresh = self._det_available and (now - self._last_det) < DET_TIMEOUT_S
        with _lock:
            _state['det_ok'] = fresh
            if not fresh:
                _state['dets'] = []
                _state['det_fps'] = 0.0

    def _actuate(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        self._expire_detections(now)
        with _lock:
            p = dict(_params)
            fwd, turn, stamp = _cmd['fwd'], _cmd['turn'], _cmd['stamp']
        held = fwd != 0 or turn != 0
        timed_out = held and now - stamp > p['cmd_timeout']
        if timed_out:
            fwd = turn = 0
        self._speed_cmd = clamp_speed(fwd * p['speed'])
        self._steer_cmd = max(-1.0, min(1.0, turn * p['angle']))

        out = AckermannDriveStamped()
        out.drive.speed = self._speed_cmd
        out.drive.steering_angle = float(-self._steer_cmd)  # wire positive = left on this car
        self._pub.publish(out)

        t = _now()
        sensors = self._sample_sensors(t)
        self._send_display(now)
        _rows.append(
            f'{t},{fwd},{turn},{self._speed_cmd:.3f},{self._steer_cmd:.3f},{self._enc:.3f}\n'
        )
        _hist.append([t, round(self._enc, 3), round(self._speed_cmd, 3)])
        with _lock:
            _state.update(
                {
                    'fwd': fwd,
                    'turn': turn,
                    'held': held and not timed_out,
                    'timed_out': timed_out,
                    'speed_cmd': round(self._speed_cmd, 3),
                    'steer': round(self._steer_cmd, 3),
                    'enc_speed': round(self._enc, 3),
                    'hist': _thin(_hist),
                    'marks': list(_marks),
                    'sense': list(_sense),
                    **sensors,
                }
            )


def _colorize_depth(meters: np.ndarray, far_m: float) -> np.ndarray:
    """
    Turn depth in meters into a BGR preview.

    Bright is near, dark is far, pure black is no return. A reading beyond
    far_m clamps to the dark end rather than dropping out, so a far wall
    still reads as a wall.
    """
    valid = meters > 0
    near = min(DEPTH_NEAR_M, far_m)
    span = max(far_m - near, 1e-3)
    norm = np.clip((meters - near) / span, 0.0, 1.0)
    ramp = (DEPTH_FLOOR + (1.0 - norm) * (255 - DEPTH_FLOOR)).astype(np.uint8)
    out = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    out[~valid] = NO_RETURN_BGR
    return out


def _thin(hist: deque) -> list:
    """Even sample of `hist` down to HIST_POINTS, newest row always kept."""
    rows = list(hist)
    stride = max(1, len(rows) // HIST_POINTS)
    if stride == 1:
        return rows
    sent = rows[::stride]
    if sent[-1] is not rows[-1]:
        sent.append(rows[-1])
    return sent


def _log_number(name: str) -> int:
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = re.match(r'Log(\d+)\.csv$', name)
    return int(m.group(1)) if m else 0


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self) -> None:
        global _last_poll
        asset = webui.static_file(self.path)
        if asset is not None:
            self.send_response(200)
            self.send_header('Content-Type', asset[1])
            self.send_header('Content-Length', str(len(asset[0])))
            self.send_header('Cache-Control', webui.CACHE_CONTROL)
            self.end_headers()
            self.wfile.write(asset[0])
            return
        if self.path == '/':
            self._send((BASE / 'teleop.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path.startswith('/depth'):
            with _lock:
                data = _depth_preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/state':
            with _lock:
                _last_poll = time.monotonic()
                body = json.dumps(
                    {
                        **_state,
                        'params': _params,
                        'marks': list(_marks),
                        'display': {'kind': _display['kind'], 'text': _display['text']},
                    }
                )
            self._send(body.encode(), 'application/json')
        elif self.path == '/logs':
            names = sorted((f.name for f in LOG_DIR.glob('*.csv')), key=_log_number)
            self._send(json.dumps(names).encode(), 'application/json')
        elif self.path.startswith('/logs/'):
            f = LOG_DIR / Path(self.path).name
            if f.is_file():
                self._send(f.read_bytes(), 'text/csv')
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        if self.path == '/cmd':
            set_cmd(self._body())
        elif self.path == '/params':
            update_params(self._body())
        elif self.path == '/logs/save':
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            latest = max((_log_number(f.name) for f in LOG_DIR.glob('Log*.csv')), default=0)
            name = f'Log{latest + 1}.csv'
            rows = list(_rows)
            tmin = float(rows[0].split(',')[0]) if rows else 0.0
            with open(LOG_DIR / name, 'w') as f:
                f.write('t,fwd,turn,speed_cmd,steer,speed_ms\n')
                f.writelines(f'# mark,{t},{label}\n' for t, label in list(_marks) if t >= tmin)
                f.writelines(rows)
            self._send(name.encode(), 'text/plain')
            return
        elif self.path == '/matrix':
            set_display('pixels', self._body().get('pixels', []))
        elif self.path == '/text':
            set_display('text', self._body().get('text', ''))
        elif self.path == '/leds':
            set_display('leds', self._body().get('colors', []))
        elif self.path == '/display/release':
            set_display('release', None)
        elif self.path == '/save':
            save_params()
        elif self.path == '/load':
            load_params()
        else:
            self.send_error(404)
            return
        self._send(b'ok', 'text/plain')

    def _body(self) -> Any:
        return json.loads(self.rfile.read(int(self.headers['Content-Length'])))

    def _send(self, body: bytes, ctype: str, cache: str | None = None) -> None:
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        pass


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main() -> None:
    load_params()
    rclpy.init()
    node = TeleopNode()
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)

    # rclpy.init() installs SIGINT/SIGTERM handlers that shut the ROS context
    # down but leave serve_forever() blocked, so the process outlives the
    # signal until systemd's TimeoutStopSec expires and SIGKILLs it. Take the
    # signals back. shutdown() has to run off the serving thread or it
    # deadlocks waiting for the loop it is called from.
    def stop(*_: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f'Web teleop dashboard on http://0.0.0.0:{PORT}')
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
