#!/usr/bin/env python3
"""
Lidar yaw calibration against a flat wall.

With the car squared to a flat wall ahead, the wall should appear in base_link
as a straight line perpendicular to x. Reads /scan, places the points in
base_link with the current config/frames.yaml offsets, fits a line to the wall
ahead, and writes the yaw that makes it perpendicular to
config/frames.local.yaml (laser_yaw_deg).

--check reports without writing.
--wall-distance-mm takes a tape distance from base_link to the wall and
reports the lidar x error it implies (nothing is written for x).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any
import warnings

import numpy as np
from racecar_neo_ros2_driver.frames import load_frames
import rclpy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    install_config_dir,
    output_dirs,
    read_line,
    source_config_dir,
    spin_until_done,
    update_local_values,
)

TOOL = 'calibrate_lidar.py'
SCAN_SECONDS = 5.0
# Wall points: within this bearing of straight ahead, seen from the lidar.
SECTOR_HALF_DEG = 30.0
MIN_WALL_POINTS = 50
# Beyond this the car is not squared to the wall, or the mount is not
# the configured one; refuse rather than write a large correction.
MAX_CORRECTION_DEG = 10.0
# The fit is planar; a tilted or upside-down mount needs the full transform.
MAX_TILT_DEG = 5.0
# --check limits: yaw to 0.5 deg; RMS residual above 10 mm means a cluttered
# or curved wall.
PASS_ANGLE_DEG = 0.5
# Hand squaring at the wheel centers is good to about 1 deg (0.32 deg per mm),
# so up to WARN_ANGLE_DEG warns; beyond it the mount moved or the car is badly
# placed.
WARN_ANGLE_DEG = 2.0
PASS_RMS_MM = 10.0


@dataclass(frozen=True)
class WallFit:
    """Line fit to the wall, in base_link."""

    angle_deg: float  # wall normal from +x; 0 when perpendicular to x
    distance_mm: float  # base_link origin to the wall, along the normal
    rms_mm: float
    max_mm: float
    points: int


def scan_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    """Return (N, 2) points in the laser frame for the valid returns."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[ok], angles[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def to_base(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Transform laser-frame points into base_link (planar; roll and pitch are 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def wall_sector(points: np.ndarray, x: float, y: float) -> np.ndarray:
    """Return the base_link points ahead of the lidar, within SECTOR_HALF_DEG."""
    bearing = np.degrees(np.arctan2(points[:, 1] - y, points[:, 0] - x))
    return points[np.abs(bearing) <= SECTOR_HALF_DEG]


def fit_wall(points: np.ndarray) -> WallFit | None:
    """Total-least-squares line fit; None with fewer than MIN_WALL_POINTS."""
    if len(points) < MIN_WALL_POINTS:
        return None
    centroid = points.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov((points - centroid).T))
    normal = vecs[:, 0]
    if normal[0] < 0:
        normal = -normal
    residuals = (points - centroid) @ normal
    return WallFit(
        angle_deg=math.degrees(math.atan2(normal[1], normal[0])),
        distance_mm=float(centroid @ normal) * 1e3,
        rms_mm=float(np.sqrt(np.mean(residuals**2))) * 1e3,
        max_mm=float(np.max(np.abs(residuals))) * 1e3,
        points=len(points),
    )


def median_ranges(scans: list[list[float]]) -> np.ndarray | None:
    """Per-beam median over scans of the most common length; inf counts as missing."""
    if not scans:
        return None
    lengths = [len(s) for s in scans]
    n = max(set(lengths), key=lengths.count)
    stack = np.array([s for s in scans if len(s) == n], dtype=float)
    stack[~np.isfinite(stack)] = np.nan
    # Beams with no return in any scan stay NaN, which scan_points drops.
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return np.nanmedian(stack, axis=0)


def corrected_yaw_deg(current_deg: float, fit: WallFit) -> float:
    """
    Return the laser yaw that makes the wall perpendicular to x.

    Points placed with a yaw too large by e show the wall rotated by +e.
    """
    return current_deg - fit.angle_deg


def check_criteria(fit: WallFit, yaw_deg: float) -> list[Criterion]:
    """Return the --check result for a wall fit made with the configured yaw."""
    angle = abs(fit.angle_deg)
    if angle <= PASS_ANGLE_DEG:
        status, advice = 'PASS', ''
    elif angle <= WARN_ANGLE_DEG:
        status = 'WARN'
        advice = 'within hand-squaring precision (1 mm at the wheel centers is 0.32 deg)'
    else:
        status = 'FAIL'
        advice = 'beyond hand-squaring error: check the lidar mount, re-square, recalibrate'
    return [
        Criterion(
            'wall angle',
            f'{fit.angle_deg:+.2f} deg (fits {corrected_yaw_deg(yaw_deg, fit):.2f})',
            f'within {PASS_ANGLE_DEG} deg (warn to {WARN_ANGLE_DEG})',
            status,
            advice,
        ),
        Criterion(
            'wall straightness',
            f'{fit.rms_mm:.1f} mm RMS, {fit.points} points',
            f'under {PASS_RMS_MM:.0f} mm',
            'PASS' if fit.rms_mm <= PASS_RMS_MM else 'FAIL',
            'wall cluttered or not flat; use a clear flat wall',
        ),
        Criterion('wall distance', f'{fit.distance_mm:.1f} mm from base_link', '', 'INFO'),
    ]


def config_dir() -> Path:
    """Return the config dir the running stack loaded: installed share, else source."""
    d = install_config_dir() or source_config_dir()
    if d is None:
        raise FileNotFoundError('no config directory found')
    return d


class LidarYawCalibrator(CalibratorNode):
    """Collect scans of a wall ahead, fit it, and correct laser_yaw_deg."""

    def __init__(
        self, check: bool, wall_distance_mm: float | None, report: str | None = None
    ) -> None:
        super().__init__('lidar_yaw_calibrator', LaserScan, '/scan')
        self.check = check
        self.wall_distance_mm = wall_distance_mm
        self.report = report
        self.meta: tuple[float, float, float, float] | None = None

    def extract(self, msg: Any) -> list[float]:
        self.meta = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        return list(msg.ranges)

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        frames = load_frames(config_dir())
        lx, ly = frames['laser_x_mm'] * 1e-3, frames['laser_y_mm'] * 1e-3
        yaw_deg = frames['laser_yaw_deg']
        if max(abs(frames['laser_roll_deg']), abs(frames['laser_pitch_deg'])) > MAX_TILT_DEG:
            log.error(f'Lidar roll or pitch over {MAX_TILT_DEG:.0f} deg; this tool assumes level.')
            return 1

        log.info('Lidar yaw calibration')
        log.info('Place the car on the ground, facing a flat, clear wall.')
        log.info('Ensure the front bumper is 0.5 to 2 meters away from the wall.')
        log.info('Use a tape measure to measure from the wall to the center of each front wheel.')
        log.info('Turn the car until the two distances are equal.')
        read_line(f'Press Enter, then keep the car still for {SCAN_SECONDS:.0f} seconds...')
        scans = self.collect(SCAN_SECONDS, 'Scan')
        ranges = median_ranges(scans)
        if ranges is None or self.meta is None:
            log.error('No scans collected; nothing written.')
            return 1

        points = to_base(scan_points(ranges, *self.meta), lx, ly, math.radians(yaw_deg))
        fit = fit_wall(wall_sector(points, lx, ly))
        if fit is None:
            log.error(f'Fewer than {MIN_WALL_POINTS} wall points ahead; nothing written.')
            return 1

        new_yaw = corrected_yaw_deg(yaw_deg, fit)
        log.info(f'Wall points: {fit.points}; RMS {fit.rms_mm:.1f} mm, max {fit.max_mm:.1f} mm')
        log.info(f'Wall distance from base_link: {fit.distance_mm:.1f} mm')
        log.info(f'Wall angle error: {fit.angle_deg:+.2f} deg')
        log.info(f'laser_yaw_deg: {yaw_deg:.2f} configured, {new_yaw:.2f} fitted')
        if self.wall_distance_mm is not None:
            dx = fit.distance_mm - self.wall_distance_mm
            log.info(
                f'Taped wall distance {self.wall_distance_mm:.1f} mm: lidar x error '
                f'{dx:+.1f} mm (laser_x_mm {frames["laser_x_mm"] - dx:.1f} would match)'
            )
        if fit.rms_mm > PASS_RMS_MM:
            log.warning(f'RMS above {PASS_RMS_MM:.0f} mm: the wall is cluttered or not flat.')

        if self.check:
            return finish_check(log, check_criteria(fit, yaw_deg), self.report)
        if abs(fit.angle_deg) > MAX_CORRECTION_DEG:
            log.error(
                f'Correction over {MAX_CORRECTION_DEG:.0f} deg: the car is not squared to '
                'the wall, or the lidar mount differs from frames.yaml. Nothing written.'
            )
            return 1
        dirs = output_dirs()
        if not dirs:
            log.error('No config directory found; nothing written.')
            return 1
        written = update_local_values('frames', {'laser_yaw_deg': round(new_yaw, 3)}, dirs, TOOL)
        log.info(backup_reminder(written))
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='Lidar yaw calibration against a flat wall.')
    parser.add_argument('--check', action='store_true', help='report only; exit 1 on FAIL')
    parser.add_argument(
        '--wall-distance-mm', type=float, help='taped distance from base_link to the wall'
    )
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    return spin_until_done(LidarYawCalibrator(args.check, args.wall_distance_mm, args.report))


if __name__ == '__main__':
    sys.exit(main())
