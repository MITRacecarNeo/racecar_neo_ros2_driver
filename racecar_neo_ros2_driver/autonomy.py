"""
autonomy_node logic without ROS: the localized test, status fields, and the map files.

Map directory layout (racecar_neo_ros2_driver/maps.py gives the root):

    <name>/<name>.posegraph, <name>.data   slam_toolbox serialization
    <name>/<name>.pgm, <name>.yaml         map_server format
    <name>/meta.yaml                       written with each save
    <name>/last_pose.yaml                  written while localized
    <name>/autosave/                       the same file set, replaced whole
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import socket
from typing import Any

import yaml

AUTOSAVE_DIR = 'autosave'
META_FILE = 'meta.yaml'
LAST_POSE_FILE = 'last_pose.yaml'
AUTOSAVE_FILE = 'autosave.yaml'

LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2


@dataclass(frozen=True)
class Limits:
    scan_timeout_sec: float = 0.5
    max_xy_sd_m: float = 0.25
    max_yaw_sd_deg: float = 20.0


def covariance_sd(cov: list[float] | tuple[float, ...]) -> tuple[float, float]:
    """Return (xy SD in m, yaw SD in deg) from a 6x6 row-major pose covariance."""
    xy = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))
    return xy, math.degrees(math.sqrt(max(cov[35], 0.0)))


def is_localized(
    mode: str,
    has_map_odom: bool,
    scan_age: float | None,
    sd: tuple[float, float] | None,
    poses_seen: int,
    global_pending: bool,
    limits: Limits,
) -> bool:
    """
    Decide `localized` for /autonomy/status.

    Both modes need map -> odom and a scan within scan_timeout_sec; a stale
    scan means the lidar stopped. That is all mapping needs: slam_toolbox
    publishes map -> odom from its first scan, and its pose only when the car
    moves. Localization also needs the localizer's pose spread under the
    limits, and stays false while a global search has not converged.
    """
    if not has_map_odom or scan_age is None or scan_age > limits.scan_timeout_sec:
        return False
    if mode == 'mapping':
        return True
    if global_pending or sd is None:
        return False
    return sd[0] <= limits.max_xy_sd_m and sd[1] <= limits.max_yaw_sd_deg


def status_level(localized: bool, has_map_odom: bool, seconds_running: float) -> int:
    if localized:
        return LEVEL_OK
    return LEVEL_ERROR if not has_map_odom and seconds_running > 5.0 else LEVEL_WARN


def write_yaml_atomic(path: Path, data: dict[str, Any]) -> None:
    """Write YAML through a temporary file and a rename, so readers never see half a file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(yaml.safe_dump(data, sort_keys=False))
    os.replace(tmp, path)


def replace_dir(tmp: Path, final: Path) -> None:
    """
    Move a finished directory over another.

    A crash at any point leaves either the old or the new directory in place
    under `final`, or the new one still under `tmp`.
    """
    old = final.with_name(final.name + '.old')
    if old.exists():
        shutil.rmtree(old)
    if final.exists():
        os.rename(final, old)
    os.rename(tmp, final)
    if old.exists():
        shutil.rmtree(old)


def pose_record(pose: tuple[float, float, float], now: datetime, **extra: Any) -> dict[str, Any]:
    x, y, yaw = pose
    return {
        'x': round(x, 4),
        'y': round(y, 4),
        'yaw': round(yaw, 5),
        'time': now.strftime('%Y-%m-%d %H:%M:%S'),
        **extra,
    }


def read_pose(path: Path) -> tuple[float, float, float] | None:
    """Read x, y, yaw from last_pose.yaml or autosave.yaml; None when missing or malformed."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
        return float(data['x']), float(data['y']), float(data['yaw'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def meta_record(
    name: str,
    now: datetime,
    driver_version: str,
    frames: dict[str, float],
    start_pose: str,
    session: str,
) -> dict[str, Any]:
    return {
        'name': name,
        'saved': now.strftime('%Y-%m-%d %H:%M:%S'),
        'driver_version': driver_version,
        'hostname': socket.gethostname(),
        'session': session,
        'start_pose': start_pose,
        'frames': frames,
    }


def status_values(**fields: Any) -> list[tuple[str, str]]:
    """Format status fields as DiagnosticStatus key-value strings, in the given order."""
    out = []
    for key, value in fields.items():
        if isinstance(value, bool):
            text = 'true' if value else 'false'
        elif isinstance(value, float):
            text = f'{value:.0f}' if key.endswith('_ms') else f'{value:.3f}'
        else:
            text = '' if value is None else str(value)
        out.append((key, text))
    return out
