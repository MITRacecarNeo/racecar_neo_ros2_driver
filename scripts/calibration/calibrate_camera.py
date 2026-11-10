#!/usr/bin/env python3
"""
RealSense mount pitch, roll and height from the floor in the depth image.

The `camera` step of `racecar calibrate`. The lens and depth calibration is
done at the factory and stored on the camera; this tool measures only how the
camera sits on the car. With the car on a flat floor and the floor clear
ahead, it fits a plane to the floor points of about 3 s of depth (RANSAC, then
a least-squares refit) and expresses it at the tripod screw, where the
URDF's mount rotation acts (camera_bottom_screw_frame). A floor plane with
upward unit normal n at distance d from the screw gives:

    camera_bottom_z_mm = d, camera_pitch_deg = -asin(n_x), camera_roll_deg = atan2(n_y, n_z)

Yaw cannot come from the floor and keeps its value. The camera's
accelerometer is a cross-check: at rest it reads gravity along the floor
normal when the floor is level. Written to the car profile's
params/frames.yaml; teleop applies it at its next start.

--check measures again and passes when pitch and roll are within
CHECK_DEG and the height within CHECK_MM of the file.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    read_line,
    run_calibrator,
    stack_config_dir,
    update_local_values,
)
from racecar_neo_ros2_driver.frames import load_frames  # noqa: E402

TOOL = 'calibrate_camera.py'
DEPTH_TOPIC = '/camera/depth'
INFO_TOPIC = '/camera/depth/camera_info'
ACCEL_TOPIC = '/camera/accel/sample'
SCREW_FRAME = 'camera_bottom_screw_frame'
BASE_FRAME = 'base_link'
COLLECT_SECONDS = 3.0
# Depth pixels used: every STEP-th row and column, within this range.
STEP = 4
MIN_RANGE_M, MAX_RANGE_M = 0.3, 2.5
# Floor candidates in base_link with the mount the stack runs now: near z = 0,
# ahead of the car.
CANDIDATE_Z_M = 0.10
CANDIDATE_X_M = (0.4, 1.8)
RANSAC_ITERATIONS = 300
INLIER_M = 0.01
MIN_INLIERS = 2000
MIN_INLIER_SHARE = 0.6
# The accelerometer and the floor normal disagree past this on a level floor.
GRAVITY_DEG = 1.0
CHECK_DEG = 0.5
CHECK_MM = 5.0

FLOOR_HELP = (
    'Set the car on its wheels on a flat floor.',
    'Clear the floor 2 m ahead of the car: no walls, boxes or feet in front of the camera.',
)


@dataclass(frozen=True)
class Plane:
    normal: np.ndarray  # unit, pointing from the floor toward the frame's origin
    distance: float  # m from the frame's origin to the floor, positive
    inliers: int
    share: float  # inliers / candidates
    rms_m: float


@dataclass(frozen=True)
class Mount:
    pitch_deg: float
    roll_deg: float
    bottom_z_mm: float


def deproject(depth_mm: np.ndarray, k: list[float], step: int = STEP) -> np.ndarray:
    """Return depth pixels as Nx3 points (m), optical frame: x right, y down, z ahead."""
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    rows = np.arange(0, depth_mm.shape[0], step)
    cols = np.arange(0, depth_mm.shape[1], step)
    v, u = np.meshgrid(rows, cols, indexing='ij')
    z = depth_mm[v, u].astype(float) / 1000.0
    ok = (z > MIN_RANGE_M) & (z < MAX_RANGE_M)
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))


def transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to Nx3 points."""
    return points @ t[:3, :3].T + t[:3, 3]


def matrix(translation: Any, rotation: Any) -> np.ndarray:
    """Return the 4x4 matrix of a geometry_msgs Transform's translation and quaternion."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = (translation.x, translation.y, translation.z)
    return out


def fit_plane(
    points: np.ndarray, rng: np.random.Generator | None = None, inlier_m: float = INLIER_M
) -> Plane | None:
    """
    Fit the dominant plane: RANSAC on three-point samples, then a least-squares refit.

    The normal is oriented toward the frame's origin (up, for the floor seen
    from above it). None with too few points.
    """
    n = len(points)
    if n < 3:
        return None
    rng = rng or np.random.default_rng(0)
    best: np.ndarray | None = None
    for _ in range(RANSAC_ITERATIONS):
        a, b, c = points[rng.choice(n, 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        mask = np.abs((points - a) @ normal) < inlier_m
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < 3:
        return None
    inl = points[best]
    centroid = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    normal = vt[2]
    # Toward the origin: the origin is on the positive side.
    if -centroid @ normal < 0:
        normal = -normal
    distance = float(-centroid @ normal)
    rms = float(np.sqrt(np.mean(((inl - centroid) @ normal) ** 2)))
    return Plane(normal, distance, int(best.sum()), float(best.mean()), rms)


def mount_from_plane(plane: Plane) -> Mount:
    """Return the mount at the tripod screw from the floor plane in the screw frame."""
    nx, ny, nz = plane.normal
    return Mount(
        math.degrees(-math.asin(max(-1.0, min(1.0, nx)))),
        math.degrees(math.atan2(ny, nz)),
        plane.distance * 1000.0,
    )


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cos = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def plane_problem(plane: Plane | None) -> str:
    """Return why the floor fit cannot be trusted, or '' when it can."""
    if plane is None or plane.inliers < MIN_INLIERS:
        return 'too little floor in view'
    if plane.share < MIN_INLIER_SHARE:
        return f'only {plane.share:.0%} of the points ahead lie on one plane'
    return ''


def check_criteria(measured: Mount, file: dict[str, float]) -> list[Criterion]:
    rows = (
        ('camera pitch', measured.pitch_deg, file.get('camera_pitch_deg', 0.0), CHECK_DEG, 'deg'),
        ('camera roll', measured.roll_deg, file.get('camera_roll_deg', 0.0), CHECK_DEG, 'deg'),
        (
            'camera height',
            measured.bottom_z_mm,
            file.get('camera_bottom_z_mm', 0.0),
            CHECK_MM,
            'mm',
        ),
    )
    return [
        Criterion(
            name,
            f'{got:+.2f} {unit} (file {want:+.2f})',
            f'within {tol:g} {unit}',
            'PASS' if abs(got - want) <= tol else 'FAIL',
            'the camera moved on its mount: racecar calibrate camera',
        )
        for name, got, want, tol, unit in rows
    ]


class CameraMountCalibrator(CalibratorNode):
    def __init__(self, check: bool, report: str | None) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, Imu
        from tf2_ros import Buffer, TransformListener

        super().__init__('calibrate_camera', Image, DEPTH_TOPIC)
        self.check = check
        self.report = report
        self.k: list[float] | None = None
        self.frame = ''
        self.accel: list[np.ndarray] = []
        self.tf = Buffer()
        self._listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._info, qos_profile_sensor_data)
        self.create_subscription(Imu, ACCEL_TOPIC, self._accel, qos_profile_sensor_data)

    def _info(self, msg: Any) -> None:
        self.k = list(msg.k)

    def _accel(self, msg: Any) -> None:
        if self._collecting:
            a = msg.linear_acceleration
            self.accel.append(np.array((a.x, a.y, a.z)))
            if not hasattr(self, 'accel_frame'):
                self.accel_frame = msg.header.frame_id

    def extract(self, msg: Any) -> Any:
        self.frame = msg.header.frame_id
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()

    def lookup(self, target: str, source: str) -> np.ndarray | None:
        from rclpy.time import Time

        try:
            t = self.tf.lookup_transform(target, source, Time())
        except Exception:  # noqa: BLE001  tf2 raises several lookup errors
            return None
        return matrix(t.transform.translation, t.transform.rotation)

    def measure(self) -> tuple[Mount, Plane, float | None] | None:
        log = self.get_logger()
        self.accel = []
        frames = self.collect(COLLECT_SECONDS, 'Depth')
        if not frames or self.k is None:
            log.error(f'No depth frames or no {INFO_TOPIC}.')
            return None
        depth = np.median(np.stack(frames), axis=0)
        depth[np.stack(frames).min(axis=0) == 0] = 0  # a pixel missing in any frame is not used
        pts = deproject(depth, self.k)
        screw_opt = self.lookup(SCREW_FRAME, self.frame)
        base_opt = self.lookup(BASE_FRAME, self.frame)
        if screw_opt is None or base_opt is None:
            log.error(f'No transform from {self.frame} to {SCREW_FRAME}: is teleop running?')
            return None
        in_base = transform(pts, base_opt)
        ahead = (
            (np.abs(in_base[:, 2]) < CANDIDATE_Z_M)
            & (in_base[:, 0] > CANDIDATE_X_M[0])
            & (in_base[:, 0] < CANDIDATE_X_M[1])
        )
        plane = fit_plane(transform(pts[ahead], screw_opt))
        problem = plane_problem(plane)
        if problem or plane is None:
            log.error(f'Floor fit: {problem}. {FLOOR_HELP[1]}')
            return None
        tilt = None
        screw_acc = self.lookup(SCREW_FRAME, getattr(self, 'accel_frame', ''))
        if self.accel and screw_acc is not None:
            g = screw_acc[:3, :3] @ np.mean(self.accel, axis=0)
            tilt = angle_deg(g, plane.normal)
        return mount_from_plane(plane), plane, tilt

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            log.error('Turn the camera on: 8080 home page, Features, Camera.')
            return 1
        for line in FLOOR_HELP:
            log.info(line)
        read_line('Press Enter when the car is in place. ')
        got = self.measure()
        if got is None:
            return 1
        mount, plane, tilt = got
        log.info(
            f'Floor: {plane.inliers} points, {plane.share:.0%} on the plane, '
            f'{plane.rms_m * 1000:.1f} mm rms.'
        )
        log.info(
            f'Camera pitch {mount.pitch_deg:+.2f} deg, roll {mount.roll_deg:+.2f} deg, '
            f'case underside {mount.bottom_z_mm:.1f} mm above the floor.'
        )
        if tilt is not None and tilt > GRAVITY_DEG:
            log.warning(
                f'The camera accelerometer and the floor differ by {tilt:.1f} deg. '
                'Check the floor is level, or run racecar calibrate realsense.'
            )
        file = load_frames(stack_config_dir())
        if self.check:
            return finish_check(log, check_criteria(mount, file), self.report)
        written = update_local_values(
            'frames',
            {
                'camera_pitch_deg': round(mount.pitch_deg, 2),
                'camera_roll_deg': round(mount.roll_deg, 2),
                'camera_bottom_z_mm': round(mount.bottom_z_mm, 1),
            },
            TOOL,
        )
        log.info(backup_reminder(written))
        return 0


def main(argv: list[str] | None = None) -> int:
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true', help='check the mount in the file')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:] if argv is None else argv)
    return run_calibrator(lambda: CameraMountCalibrator(args.check, args.report))


if __name__ == '__main__':
    sys.exit(main())
