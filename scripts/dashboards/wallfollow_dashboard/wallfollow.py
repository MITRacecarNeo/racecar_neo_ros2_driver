#!/usr/bin/env python3
"""
RACECAR Neo wall-follow service: /scan -> PD steering -> /drive + web dashboard.

Same pattern as the driver's dashboard.py (stdlib HTTP + rclpy) on port 8083.
CAUTION: the RACECAR Neo mux gates /drive on the RB bumper and
zeroes output when /joy or the active source goes stale. The shipped yaml has
speed 0.0 so the car cannot drive until the slider is raised.

Two forms, selected by `mode` in wallfollow.yaml (file edit only, no UI switch):
  static:  mirrored side windows at look_angle. Error = right - left meters.
           Setpoint 0 keeps the car centered between the walls.
  dynamic: steer toward the most open heading in a front arc, with obstacles
           inflated by the car's half-width. side_weight biases one side.

Angles use the lab convention (0 = nose, positive = right), turned from raw
scan angles by the car's calibrated lidar yaw; see LASER_YAW. /drive steering
is negated on publish: positive on the wire turns this car left (physically
verified; the car steered mirrored without the negation). Speed feedback comes
from /odom.
"""

import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racecar_neo_ros2_driver import apps, frames
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PORT = apps.port('wallfollow')
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import labkit  # noqa: E402, I100
import webui  # noqa: E402

SEARCH_STEP = 2  # degrees between candidate headings (dynamic)
HALF_CLEAR = 0.21  # meters, car half-width plus margin, inflates obstacles
CENTER_BIAS = 0.006  # score penalty per degree off-center, stops ping-pong

# The lidar's yaw in base_link from the car's frames (config/frames.yaml, with
# the car profile's calibrated value on top): about 180 degrees on the RACECAR
# Neo, whose RPLIDAR faces aft. Lab angles are 0 at the nose and positive to
# the car's right (frames.student_deg); nothing here assumes a sweep width.
LASER_YAW = frames.laser_mount()[2]

_lock = threading.Lock()
_marks = labkit.Marks()
_state: dict = {'scan': [], 'hist': [], 'overlay': {}, 'enc_speed': 0.0}


TUNE_KEYS = (
    'speed',
    'kp',
    'kd',
    'speed_kp',
    'speed_kd',
    'window',
    'lookahead',
    'look_angle',
    'width',
    'side_weight',
)


def _clean(p: dict[str, Any]) -> None:
    """Clamp the speed; read the mode as 0 static or 1 dynamic. Caller holds _lock."""
    p['speed'] = labkit.clamp_speed(p['speed'])
    # Older yamls spell the mode out.
    p['mode'] = 1 if str(p['mode']).strip() in ('1', 'dynamic') else 0


# Shipped defaults in wallfollow.yaml; saved tuning in the car profile
# (dashboards/wallfollow.yaml), loaded on top.
TUNING = labkit.Tuning(
    'wallfollow', BASE / 'wallfollow.yaml', TUNE_KEYS, _lock, _marks, clean=_clean, fixed=('mode',)
)
_params = TUNING.params
LOG = labkit.RunLog('wallfollow', 't,error,steer,speed_cmd,speed_ms', _marks, maxlen=600)


class WallFollowNode(Node):
    def __init__(self) -> None:
        super().__init__('wallfollow')
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self._last_error = 0.0
        self._enc = 0.0
        self._enc_stamp = 0.0
        self._speed_cmd = 0.0
        self._trim = 0.0
        self._last_speed_error = 0.0

    def _odom_cb(self, msg: Odometry) -> None:
        self._enc = msg.twist.twist.linear.x
        self._enc_stamp = time.monotonic()

    @staticmethod
    def _index_at(msg: LaserScan, deg: float) -> int:
        """Ray index for a student angle (0 = nose, positive = right)."""
        raw = frames.raw_rad(deg, LASER_YAW)
        return round((raw - msg.angle_min) / msg.angle_increment) % len(msg.ranges)

    @staticmethod
    def _student_deg(msg: LaserScan, i: int) -> float:
        """Lab angle of ray `i`, -180 to 180."""
        return frames.student_deg(msg.angle_min + i * msg.angle_increment, LASER_YAW)

    def _mean_at(self, msg: LaserScan, deg: float, window: float) -> float:
        """Average range (m) in a `window`-deg cone at `deg` (0 = nose, + = right)."""
        n = len(msg.ranges)
        center = self._index_at(msg, deg)
        half = max(1, round(math.radians(window / 2) / msg.angle_increment))
        vals = [msg.ranges[(center + k) % n] for k in range(-half, half + 1)]
        vals = [r for r in vals if msg.range_min < r < msg.range_max]
        # No return means open space, not zero. Zero would pull toward gaps.
        return sum(vals) / len(vals) if vals else msg.range_max

    def _scan_cb(self, msg: LaserScan) -> None:
        with _lock:
            p = dict(_params)

        la = max(p['lookahead'], 0.1)
        if int(p['mode']) == 1:
            degs = list(range(-int(p['width']), int(p['width']) + 1, SEARCH_STEP))
            dists = [min(self._mean_at(msg, d, p['window']), la) for d in degs]
            # An obstacle at d blocks every heading within atan(HALF_CLEAR/d)
            # of it. The car body would sideswipe it there.
            corridor = list(dists)
            for j, d in enumerate(dists):
                if d >= la:
                    continue
                block = math.degrees(math.atan2(HALF_CLEAR, d))
                span = int(block // SEARCH_STEP) + 1
                for i in range(max(0, j - span), min(len(degs), j + span + 1)):
                    if abs(degs[i] - degs[j]) <= block and d < corridor[i]:
                        corridor[i] = d
            best = max(
                range(len(degs)),
                key=lambda i: corridor[i]
                - CENTER_BIAS * abs(degs[i])
                + p['side_weight'] * degs[i],
            )
            error = degs[best] / p['width']
            overlay = {'mode': 'dynamic', 'width': p['width'], 'target_deg': degs[best], 'la': la}
        else:
            left = min(self._mean_at(msg, -p['look_angle'], p['window']), la)
            right = min(self._mean_at(msg, p['look_angle'], p['window']), la)
            error = right - left  # >0: drifted left -> steer right
            overlay = {
                'mode': 'static',
                'look_angle': p['look_angle'],
                'window': p['window'],
                'left': round(left, 2),
                'right': round(right, 2),
                'la': la,
            }

        cmd = p['kp'] * error + p['kd'] * (error - self._last_error)
        self._last_error = error
        cmd = max(-1.0, min(1.0, cmd))

        # Speed shaping, race_fast style: the slider is the throttle cap.
        # Slow when the wall ahead is inside lookahead or when steering hard,
        # and PD-track the target so the ramp is smooth.
        front = min(self._mean_at(msg, 0, p['window']), la)
        # Target speed in m/s from how much road the car actually has:
        # the shorter of the corridor straight ahead and the corridor along
        # the chosen line. Long corridor = straight = fast; corners shorten
        # it and slow the car with no separate corner term. speed_kp/kd then
        # regulate the MEASURED speed from odometry onto that target, so
        # battery sag and surface changes are corrected instead of baked in.
        if int(p['mode']) == 1:
            road = corridor[best]  # the chosen heading's furthest distance
        else:
            road = front
        # Speed, kept simple: the slider IS the throttle. With speed_kp
        # and speed_kd at zero the car holds that constant command, exactly
        # like rc.drive.set_speed_angle. Nonzero gains bend the throttle
        # toward the road-shaped target using measured speed: slower into
        # corners, corrected for battery and surface. Stale odometry just
        # falls back to the constant command.
        target = p['speed'] * p['max_mps'] * min(road / la, 1.0)
        serr = target - self._enc
        gains_off = p['speed_kp'] == 0 and p['speed_kd'] == 0
        odom_stale = time.monotonic() - self._enc_stamp > 0.5
        if p['speed'] <= 1e-3:
            self._trim = 0.0
            self._speed_cmd = 0.0
        elif gains_off or odom_stale:
            self._trim = 0.0
            self._speed_cmd = p['speed']
        else:
            # Never integrate up while commanded but not moving (gated).
            blocked = self._enc < 0.05 and self._speed_cmd > 0.3
            if serr < 0 or not blocked:
                self._trim += p['speed_kp'] * serr + p['speed_kd'] * (
                    serr - self._last_speed_error
                )
            self._trim = max(-1.0, min(0.3, self._trim))
            self._speed_cmd = max(0.0, min(1.0, p['speed'] + self._trim))
        self._last_speed_error = serr

        self._pub.publish(labkit.drive_msg(self._speed_cmd, cmd))

        t = labkit.now()
        LOG.rows.append(f'{t},{error:.4f},{cmd:.4f},{self._speed_cmd:.3f},{self._enc:.3f}\n')
        LOG.hist.append([t, round(error, 4)])
        overlay.update(
            {
                'error': round(error, 4),
                'steer': round(cmd, 3),
                'speed_cmd': round(self._speed_cmd, 3),
            }
        )
        scan = frames.student_scan(msg, LASER_YAW)
        with _lock:
            _state.update(
                {
                    'scan': scan,
                    'overlay': overlay,
                    'hist': list(LOG.hist),
                    'marks': list(_marks),
                    'enc_speed': round(self._enc, 3),
                }
            )


class Handler(webui.AppHandler):
    def do_GET(self) -> None:
        if self.serve_asset():
            return
        if self.path == '/':
            self.page(BASE / 'wallfollow.html')
        elif self.path == '/state':
            with _lock:
                body = json.dumps({**_state, 'params': _params})
            self.send(200, body.encode(), 'application/json')
        elif not LOG.get(self):
            self.send_error(404)

    def do_POST(self) -> None:
        if self.path == '/params':
            TUNING.update(self.read_json())
        elif LOG.post(self):
            return
        elif self.path == '/save':
            TUNING.save()
        elif self.path == '/load':
            TUNING.load()
        else:
            self.send_error(404)
            return
        self.send(200, b'ok', 'text/plain')


def main() -> None:
    TUNING.load()
    rclpy.init()
    node = WallFollowNode()
    print(f'Wall-follow dashboard on http://0.0.0.0:{PORT} (mode={_params["mode"]})')
    webui.serve(PORT, Handler, spin=webui.ros_spinner(node), cleanup=webui.ros_shutdown)


if __name__ == '__main__':
    main()
