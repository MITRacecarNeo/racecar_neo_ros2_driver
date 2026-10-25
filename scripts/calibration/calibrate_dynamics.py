#!/usr/bin/env python3
"""
Vehicle dynamics for the simulator: steering angle, servo time, speed, stopping.

The `dynamics` step of `racecar calibrate`. The car drives four fixed runs by
itself while the operator holds RB; each is recorded as a bag and analyzed.
Results go to the car profile's params/dynamics.yaml, which no node reads: it records
this car's measured values for the simulator. There is no --check.

Runs (normalized commands, as on /drive; config/throttle.yaml gives m/s):

    straight025, straight05, straight10   2 s still, speed 0.25 / 0.5 / 1.0
                                           straight, then 0 and 3 s still
    circle_right, circle_left              2 s still, speed 0.25 straight for
                                           0.8 s, full lock for 13 s, then 0
                                           and 3 s still

Straight runs give the steady speed, the time to reach it, and the stop
distance and time after the command goes to 0, from /odom/integrated; the
lidar's distance between the rest scans checks the encoder over the run.
Circle runs chain lidar scan matches (each scan deskewed with the odometry)
into a trajectory and fit a circle: the rear-axle radius, the Ackermann inner
and outer wheel angles, the outer front tire track diameter, and the
encoder's over-read against the lidar path. The step from straight to full
lock gives the center-to-lock steering time from the yaw rate, an upper
bound for the servo since it includes tire lag; lock to lock is the sum of
both sides.

--analyze BAG re-analyzes recorded runs without driving.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

import numpy as np
from racecar_neo_ros2_driver.car_profile import param_path
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    node_params,
    run_sequence,
    stack_config_dir,
    update_local_values,
)
from racecar_log import build_bag_name, build_record_argv, load_config  # noqa: E402, I100
from racecar_log import resolve_log_root  # noqa: E402
from scan_match import (  # noqa: E402
    base_motion,
    compose,
    invert,
    laser_motion,
    match_scans,
    Pose,
    transform,
)

TOOL = 'calibrate_dynamics.py'
TOPICS = ['/scan', '/imu/lsm9ds1', '/odom/integrated', '/motor', '/drive']
RATE_HZ = 50.0
RB_BUTTON = 5  # config/mux.yaml autonomy_enable_button
COUNTDOWN_SEC = 3.0
# (speed, steering, seconds); steering +1 is full left.
RUNS: dict[str, list[tuple[float, float, float]]] = {
    'straight025': [(0.0, 0.0, 2.0), (0.25, 0.0, 3.0), (0.0, 0.0, 3.0)],
    'straight05': [(0.0, 0.0, 2.0), (0.5, 0.0, 3.0), (0.0, 0.0, 3.0)],
    'straight10': [(0.0, 0.0, 2.0), (1.0, 0.0, 2.5), (0.0, 0.0, 3.0)],
    # A short straight before the lock gives the center-to-lock steering step.
    'circle_right': [(0.0, 0.0, 2.0), (0.25, 0.0, 0.8), (0.25, -1.0, 13.0), (0.0, 0.0, 3.0)],
    'circle_left': [(0.0, 0.0, 2.0), (0.25, 0.0, 0.8), (0.25, 1.0, 13.0), (0.0, 0.0, 3.0)],
}
TITLES = {
    'straight025': 'straight at speed 0.25',
    'straight05': 'straight at speed 0.5',
    'straight10': 'straight at speed 1.0',
    'circle_right': 'right-turn circles at full lock, at speed 0.25',
    'circle_left': 'left-turn circles at full lock, at speed 0.25',
}
STRAIGHT_PLACE = 'Place the car at the start of the straight path, pointing along it.'
CIRCLE_PLACE = 'Place the car at one end of the clear area, pointing along it.'
# Wheelbase and track measured on a reference car, m.
# Per car: odometry.wheelbase_m in odom.yaml / the profile's params/odom.yaml (measured).
WHEELBASE_M = float(node_params('odom', 'pit_node').get('odometry.wheelbase_m', 0.230))
TRACK_M = 0.178
# Circle fits skip this long after each steering change.
SETTLE_SEC = 2.0


# ---------------------------------------------------------------------------
# Geometry and timing (pure)
# ---------------------------------------------------------------------------


def fit_circle(xy: np.ndarray) -> tuple[float, float, float, float]:
    """Return (cx, cy, radius, RMS residual) of an algebraic circle fit."""
    x, y = xy[:, 0], xy[:, 1]
    design = np.column_stack((x, y, np.ones(len(x))))
    (d, e, f), *_ = np.linalg.lstsq(design, -(x * x + y * y), rcond=None)
    cx, cy = -d / 2.0, -e / 2.0
    r = math.sqrt(cx * cx + cy * cy - f)
    resid = np.hypot(x - cx, y - cy) - r
    return float(cx), float(cy), r, float(np.sqrt(np.mean(resid**2)))


@dataclass(frozen=True)
class Steering:
    """Front wheel angles (deg) for a rear-axle-midpoint turning radius (m)."""

    radius_m: float
    inner_deg: float
    outer_deg: float
    bicycle_deg: float
    outer_front_track_diameter_m: float


def ackermann(radius_m: float, wheelbase: float = WHEELBASE_M, track: float = TRACK_M) -> Steering:
    """Return wheel angles that roll without slip about a center `radius_m` from base_link."""
    inner = math.degrees(math.atan(wheelbase / (radius_m - track / 2)))
    outer = math.degrees(math.atan(wheelbase / (radius_m + track / 2)))
    return Steering(
        radius_m,
        inner,
        outer,
        math.degrees(math.atan(wheelbase / radius_m)),
        2 * math.hypot(radius_m + track / 2, wheelbase),
    )


def transition_times(
    t: np.ndarray, kappa: np.ndarray, t_switch: float, before: float, after: float
) -> tuple[float, float]:
    """
    Return (10-90% time, command-to-95% time) of a curvature step at t_switch.

    `before` and `after` are the steady curvatures on each side.
    """
    span = after - before
    frac = (kappa - before) / span
    later = t >= t_switch

    def first(level: float) -> float:
        hit = np.nonzero(later & (frac >= level))[0]
        return float(t[hit[0]]) if hit.size else math.nan

    t10, t90, t95 = first(0.1), first(0.9), first(0.95)
    return t90 - t10, t95 - t_switch


@dataclass(frozen=True)
class StraightResult:
    steady_mps: float
    rise_90_s: float  # command on to 90% of steady
    stop_m: float  # command 0 to standstill, along the path
    stop_s: float


def straight_metrics(
    t: np.ndarray, v: np.ndarray, s: np.ndarray, t_on: float, t_off: float
) -> StraightResult:
    """Speed `v` and path length `s` (both from /odom/integrated) at times `t`."""
    hold = (t >= t_off - 1.0) & (t < t_off)
    steady = float(np.mean(v[hold]))
    rise = np.nonzero((t >= t_on) & (v >= 0.9 * steady))[0]
    after = np.nonzero(t >= t_off)[0]
    moving = after[np.abs(v[after]) > 0.0]
    i_off = after[0]
    i_stop = moving[-1] + 1 if moving.size else i_off
    i_stop = min(i_stop, len(t) - 1)
    return StraightResult(
        steady,
        float(t[rise[0]] - t_on) if rise.size else math.nan,
        float(s[i_stop] - s[i_off]),
        float(t[i_stop] - t_off),
    )


# ---------------------------------------------------------------------------
# Lidar scan chaining
# ---------------------------------------------------------------------------


def interp_pose(t: np.ndarray, poses: np.ndarray, at: np.ndarray) -> np.ndarray:
    """Interpolate (x, y, unwrapped theta) poses at times `at`."""
    return np.column_stack([np.interp(at, t, poses[:, k]) for k in range(3)])


def deskew(
    points: np.ndarray,
    beam_times: np.ndarray,
    t_ref: float,
    odom_t: np.ndarray,
    odom_poses: np.ndarray,
    laser: Pose,
) -> np.ndarray:
    """Move each point from the laser frame at its beam time to the laser frame at t_ref."""
    ref = tuple(interp_pose(odom_t, odom_poses, np.array([t_ref]))[0])
    at = interp_pose(odom_t, odom_poses, beam_times)
    to_ref = compose(invert(laser), invert(ref))
    out = np.empty_like(points)
    for i, (p, pose) in enumerate(zip(points, at, strict=True)):
        m = compose(compose(to_ref, tuple(pose)), laser)
        out[i] = transform(p[None, :], m)[0]
    return out


def chain_scans(
    scans: list[tuple[float, np.ndarray, np.ndarray]],
    odom_t: np.ndarray,
    odom_poses: np.ndarray,
    laser: Pose,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Return (times, base_link poses, mean match RMS) from consecutive scan matches.

    Each scan is (reference time, laser-frame points, beam times); poses start
    at the first scan, in its base_link frame.
    """
    times, poses, rms = [scans[0][0]], [(0.0, 0.0, 0.0)], []
    prev_t = scans[0][0]
    prev_pts = deskew(scans[0][1], scans[0][2], prev_t, odom_t, odom_poses, laser)
    pose = (0.0, 0.0, 0.0)
    for t_ref, pts, beams in scans[1:]:
        cur = deskew(pts, beams, t_ref, odom_t, odom_poses, laser)
        a, b = interp_pose(odom_t, odom_poses, np.array([prev_t, t_ref]))
        guess = laser_motion(compose(invert(tuple(a)), tuple(b)), laser)
        match = match_scans(prev_pts, cur, guess)
        rms.append(match.rms_m)
        pose = compose(pose, base_motion(match.pose, laser))
        times.append(t_ref)
        poses.append(pose)
        prev_t, prev_pts = t_ref, cur
    return np.array(times), np.array(poses), float(np.mean(rms)) if rms else math.nan


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------


@dataclass
class Bag:
    odom_t: np.ndarray  # s
    odom: np.ndarray  # x, y, unwrapped theta
    speed: np.ndarray  # m/s, /odom/integrated twist
    path_m: np.ndarray  # cumulative distance along /odom/integrated
    gyro_t: np.ndarray
    yaw_rate: np.ndarray  # rad/s about base_link z, as the odometry reads it
    motor_t: np.ndarray  # bag receive time, s
    motor: np.ndarray  # normalized speed, steering
    scans: list[tuple[float, np.ndarray, np.ndarray]]  # stamp, ranges, angles


def read_bag(path: Path, axis: int = 2, sign: float = 1.0) -> Bag:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = rosbag2_py.Info().read_metadata(str(path), '')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {n: get_message(types[n]) for n in TOPICS if n in types}
    odom, gyro, motor, scans = [], [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == '/odom/integrated':
            q = msg.pose.pose.orientation
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom.append(
                (
                    st,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    2 * math.atan2(q.z, q.w),
                    msg.twist.twist.linear.x,
                )
            )
        elif topic == '/imu/lsm9ds1':
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gyro.append((st, sign * getattr(msg.angular_velocity, 'xyz'[axis])))
        elif topic == '/motor':
            motor.append((t_ns * 1e-9, msg.drive.speed, msg.drive.steering_angle))
        elif topic == '/scan':
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ranges = np.asarray(msg.ranges, float)
            angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
            ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            beams = st + msg.time_increment * np.arange(ranges.size)
            scans.append((st + msg.scan_time, ranges[ok], angles[ok], beams[ok]))
    o = np.array(odom)
    xy = o[:, 1:3]
    path_m = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))))
    g = np.array(gyro)
    m = np.array(motor)
    pts = [(t_ref, np.column_stack((r * np.cos(a), r * np.sin(a))), b) for t_ref, r, a, b in scans]
    return Bag(
        odom_t=o[:, 0],
        odom=np.column_stack((xy, np.unwrap(o[:, 3]))),
        speed=o[:, 4],
        path_m=path_m,
        gyro_t=g[:, 0],
        yaw_rate=g[:, 1],
        motor_t=m[:, 0],
        motor=m[:, 1:3],
        scans=pts,
    )


def command_edges(bag: Bag) -> list[tuple[float, float, float]]:
    """Return (time, speed, steering) at each change of the /motor command."""
    edges = []
    last = None
    for t, (sp, st) in zip(bag.motor_t, bag.motor, strict=True):
        key = (round(float(sp), 3), round(float(st), 3))
        if key != last:
            edges.append((float(t), *key))
            last = key
    return edges


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def laser_pose() -> Pose:
    from racecar_neo_ros2_driver.frames import load_frames

    f = load_frames(stack_config_dir())
    return f['laser_x_mm'] * 1e-3, f['laser_y_mm'] * 1e-3, math.radians(f['laser_yaw_deg'])


# Scans stacked per rest: a few fill gaps between beams; more only slow the match.
REST_SCANS = 3


def rest_scan(bag: Bag, t0: float, t1: float) -> np.ndarray:
    """Return the points of the last REST_SCANS scans between t0 and t1."""
    pts = [p for t, p, _ in bag.scans if t0 <= t <= t1][-REST_SCANS:]
    return np.vstack(pts) if pts else np.empty((0, 2))


def speed_tag(speed: float) -> str:
    """Return a key suffix for a normalized speed: 0.25 -> '025', 1.0 -> '10'."""
    return f'{speed:.2f}'.replace('.', '').rstrip('0').ljust(2, '0')


def analyze_straight(bag: Bag) -> tuple[dict[str, float], list[str]]:
    """Return (values for params/dynamics.yaml, report lines) for a straight run."""
    edges = command_edges(bag)
    on = next(e for e in edges if e[1] != 0.0)
    off = next(e for e in edges if e[0] > on[0] and e[1] == 0.0)
    r = straight_metrics(bag.odom_t, bag.speed, bag.path_m, on[0], off[0])
    tag = speed_tag(on[1])
    values = {
        f'steady_speed_mps_{tag}': round(r.steady_mps, 3),
        f'rise_90_s_{tag}': round(r.rise_90_s, 2),
        f'stop_distance_mm_{tag}': round(r.stop_m * 1e3),
        f'stop_time_s_{tag}': round(r.stop_s, 2),
    }
    lines = [
        f'Speed {on[1]:g}: steady {r.steady_mps:.3f} m/s, '
        f'90% of it {r.rise_90_s:.2f} s after the command.',
        f'Stop after the command goes to 0: {r.stop_m * 1e3:.0f} mm in {r.stop_s:.2f} s.',
    ]
    laser = laser_pose()
    start = rest_scan(bag, bag.odom_t[0], on[0] - 0.2)
    end = rest_scan(bag, bag.odom_t[-1] - 2.0, bag.odom_t[-1])
    if len(start) and len(end):
        a, b = tuple(bag.odom[0]), tuple(bag.odom[-1])
        match = match_scans(start, end, laser_motion(compose(invert(a), b), laser))
        lidar_m = math.hypot(*base_motion(match.pose, laser)[:2])
        odom_m = float(bag.path_m[-1] - bag.path_m[0])
        lines.append(
            f'Run distance: odometry {odom_m:.3f} m, lidar {lidar_m:.3f} m '
            f'({odom_m / lidar_m - 1:+.2%}; scan match {match.rms_m * 1e3:.1f} mm, '
            f'{match.inliers:.0%}).'
        )
    return values, lines


def analyze_circle(bag: Bag) -> tuple[dict[str, float], list[str]]:
    """Return (values for params/dynamics.yaml, report lines) for one circle run."""
    edges = command_edges(bag)
    lead = next(e for e in edges if e[1] != 0.0)
    lock = next(e for e in edges if e[0] > lead[0] and e[2] != 0.0)
    t_end = next(e[0] for e in edges if e[0] > lock[0] and e[1] == 0.0)
    side = 'left' if lock[2] > 0 else 'right'
    laser = laser_pose()
    scans = [s for s in bag.scans if lock[0] - 0.5 <= s[0] <= t_end]
    # The driver does not document beam time order against beam index; the
    # order that matches better wins.
    flipped = [(t, p, b[::-1]) for t, p, b in scans]
    trial = [chain_scans(c[:15], bag.odom_t, bag.odom, laser)[2] for c in (scans, flipped)]
    if trial[1] < trial[0]:
        scans = flipped
    times, poses, rms = chain_scans(scans, bag.odom_t, bag.odom, laser)
    keep = (times >= lock[0] + SETTLE_SEC) & (times <= t_end)
    _, _, radius, resid = fit_circle(poses[keep, :2])
    laps = abs(poses[keep, 2][-1] - poses[keep, 2][0]) / (2 * math.pi)
    lidar_path = float(np.sum(np.hypot(*np.diff(poses[keep, :2], axis=0).T)))
    in_odom = (bag.odom_t >= times[keep][0]) & (bag.odom_t <= times[keep][-1])
    enc_path = float(bag.path_m[in_odom][-1] - bag.path_m[in_odom][0])
    sa = ackermann(radius)
    values: dict[str, float] = {
        f'turn_radius_mm_{side}': round(radius * 1e3),
        f'steer_inner_deg_{side}': round(sa.inner_deg, 1),
        f'steer_outer_deg_{side}': round(sa.outer_deg, 1),
        f'outer_front_track_diameter_mm_{side}': round(sa.outer_front_track_diameter_m * 1e3),
        f'encoder_turn_overread_{side}': round(enc_path / lidar_path - 1, 4),
    }
    lines = [
        f'Scan chain: {len(times)} scans, mean match {rms * 1e3:.1f} mm.',
        f'Full {side} lock at speed {lock[1]:g}: rear-axle radius {radius * 1e3:.0f} mm '
        f'(fit RMS {resid * 1e3:.1f} mm, {laps:.2f} laps).',
        f'Wheel angles: inner {sa.inner_deg:.1f} deg, outer {sa.outer_deg:.1f} deg.',
        f'Outer front tire track diameter: {sa.outer_front_track_diameter_m * 1e3:.0f} mm.',
        f'Path: lidar {lidar_path:.3f} m, encoder {enc_path:.3f} m '
        f'({enc_path / lidar_path - 1:+.2%}).',
    ]

    def curvature(t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
        g = (bag.gyro_t >= t0) & (bag.gyro_t <= t1)
        v = np.interp(bag.gyro_t[g], bag.odom_t, bag.speed)
        return bag.gyro_t[g], bag.yaw_rate[g] / np.where(np.abs(v) > 0.1, v, np.nan)

    straight = float(np.nanmedian(curvature(lock[0] - 0.4, lock[0])[1]))
    turning = float(np.nanmedian(curvature(lock[0] + SETTLE_SEC, t_end)[1]))
    t, kappa = curvature(lock[0] - 0.3, lock[0] + 1.5)
    rise, total = transition_times(t, kappa, lock[0], straight, turning)
    values[f'servo_center_to_lock_s_{side}'] = round(total, 3)
    lines.append(
        f'Center to full {side} lock from the yaw rate: 10-90% in {rise:.3f} s, 95% '
        f'{total:.3f} s after the command (an upper bound for the servo).'
    )
    return values, lines


def lock_to_lock(values: dict[str, float]) -> dict[str, float]:
    """Return servo_lock_to_lock_s from both center-to-lock times, when both exist."""
    right = values.get('servo_center_to_lock_s_right')
    left = values.get('servo_center_to_lock_s_left')
    if right is None or left is None:
        return {}
    return {'servo_lock_to_lock_s': round(right + left, 3)}


def analyze(bag_path: Path) -> tuple[dict[str, float], list[str]]:
    bag = read_bag(bag_path)
    return analyze_circle(bag) if '_dyn_circle_' in bag_path.name else analyze_straight(bag)


# ---------------------------------------------------------------------------
# Driving
# ---------------------------------------------------------------------------


class Driver:
    """Publish /drive while RB is held on /joy; record the topics the analysis needs."""

    def __init__(self) -> None:
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from sensor_msgs.msg import Joy

        rclpy.init()
        self.rclpy = rclpy
        self.msg_type = AckermannDriveStamped
        self.node = rclpy.create_node('dynamics_calibrator')
        self.log = self.node.get_logger()
        self.pub = self.node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.rb = False
        self.node.create_subscription(Joy, '/joy', self._joy, 10)

    def _joy(self, msg: Any) -> None:
        self.rb = len(msg.buttons) > RB_BUTTON and bool(msg.buttons[RB_BUTTON])

    def _hold(self, seconds: float, speed: float, steer: float, need_rb: bool) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = self.msg_type()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steer)
            self.pub.publish(msg)
            self.rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            if need_rb and not self.rb:
                return False
        return True

    def drive(
        self,
        run: str,
        segments: list[tuple[float, float, float]] | None = None,
        prefix: str = 'dyn',
    ) -> Path | None:
        """
        Drive one run while RB is held; return its bag, or None when RB was released.

        `segments` defaults to RUNS[run]; the bag is named <time>_<prefix>_<run>.
        """
        root = Path(resolve_log_root(load_config().get('DIR')))
        bag_dir = root / build_bag_name(f'{prefix}_{run}', time.time())
        recorder = subprocess.Popen(
            build_record_argv(bag_dir, TOPICS),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        ok = False
        try:
            self._hold(2.0, 0.0, 0.0, need_rb=False)
            self.log.info(
                'Press and hold RB. The car starts driving 3 seconds after you press it.'
            )
            self.log.info('Release RB at any time to stop the car.')
            while not self.rb:
                self._hold(0.1, 0.0, 0.0, need_rb=False)
            ok = self._hold(COUNTDOWN_SEC, 0.0, 0.0, need_rb=True)
            plan = segments or RUNS[run]
            moving = [k for k, seg in enumerate(plan) if seg[0] != 0.0]
            last_move = moving[-1] if moving else -1
            for k, (speed, steer, seconds) in enumerate(plan if ok else []):
                # RB is needed from the first move to the last, including the
                # pauses between moves; the rests at the start and end are still.
                if k == last_move + 1:
                    self.log.info('You can release RB now.')
                    self.log.info(f'Keep your hands off the car for {seconds:.0f} seconds.')
                need_rb = bool(moving) and moving[0] <= k <= last_move
                if not self._hold(seconds, speed, steer, need_rb=need_rb):
                    ok = False
                    break
        finally:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=15)
        if ok:
            self.log.info('Done.')
            return bag_dir
        self.log.error('Stopped: RB was released. This run is not measured.')
        return None

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def write_values(values: dict[str, float], log: Any) -> int:
    # The two circle runs may come from different sessions.
    local = param_path('dynamics')
    stored = (yaml.safe_load(local.read_text()) or {}) if local.exists() else {}
    values = {**values, **lock_to_lock({**stored, **values})}
    written = update_local_values('dynamics', values, TOOL)
    for key, value in sorted(values.items()):
        log.info(f'{key}: {value}')
    log.info(backup_reminder(written))
    return 0


def run_session() -> int:
    driver = Driver()
    log = driver.log
    values: dict[str, float] = {}
    runs = list(RUNS)

    def show(i: int) -> None:
        log.info(f'Run {i + 1} of {len(runs)}: {TITLES[runs[i]]}')
        log.info(CIRCLE_PLACE if runs[i].startswith('circle') else STRAIGHT_PLACE)

    def run_one(i: int) -> dict[str, float] | None:
        bag = driver.drive(runs[i])
        if bag is None:
            return None
        log.info('Analyzing the run...')
        got, lines = analyze(bag)
        for line in lines:
            log.info(line)
        return got

    try:
        for _, got in sorted(run_sequence(len(runs), show, run_one).items()):
            values.update(got)
        if not values:
            log.info('Nothing measured.')
            return 0
        return write_values(values, log)
    finally:
        driver.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--analyze', nargs='+', metavar='BAG', help='analyze recorded runs instead of driving'
    )
    args = parser.parse_args(argv)
    if not args.analyze:
        return run_session()
    import logging

    logging.basicConfig(format='%(message)s', level=logging.INFO)
    log = logging.getLogger(TOOL)
    values: dict[str, float] = {}
    for name in args.analyze:
        bag = Path(name).expanduser()
        if not bag.exists():
            bag = Path(resolve_log_root(load_config().get('DIR'))) / name
        log.info(bag.name)
        got, lines = analyze(bag)
        for line in lines:
            log.info(line)
        values.update(got)
    return write_values(values, log)


if __name__ == '__main__':
    sys.exit(main())
