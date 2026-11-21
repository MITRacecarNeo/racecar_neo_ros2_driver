"""
Parked checks: is the car fit to map and drive.

Pure functions over a few seconds of sensor data, with the car still. Each
returns rows of (name, status, detail, repair, link): the status is OK, WARN
or FAIL as in `racecar status`, and a row that is not OK names the step that
repairs it. scripts/racecar_check.py samples the topics and runs them; the
Car Profile Checks page and the map viewer's gate read its results.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
import math
from typing import Any

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'
ORDER = (OK, WARN, FAIL)
GRAVITY = 9.80665

# Gyro mean at rest, rad/s, after the stored bias. Odometry treats the car as
# stopped only under 2 deg/s, so a bias above that drifts the heading parked.
GYRO_OK, GYRO_FAIL = 0.01, math.radians(2.0)
# Gravity magnitude error at rest.
GRAVITY_OK, GRAVITY_FAIL = 0.01, 0.03
# Odometry drift over the sample with the car still.
DRIFT_OK_M, DRIFT_FAIL_M = 0.01, 0.05
TURN_OK_DEG, TURN_FAIL_DEG = 0.5, 2.0
# Lidar: rate against its 7.2 Hz nominal, share of valid ranges.
SCAN_HZ_OK, SCAN_HZ_FAIL = 6.0, 3.0
SCAN_VALID_OK, SCAN_VALID_FAIL = 0.5, 0.2
# Share of a stream's nominal rate: under OK warns, under FAIL fails.
RATE_OK, RATE_FAIL = 0.8, 0.5
CALIBRATION_PAGE = '/calibration'
HEALTH_PAGE = '/handbook/health'


@dataclass(frozen=True)
class Row:
    name: str
    status: str
    detail: str
    repair: str = ''
    link: str = ''

    def view(self) -> dict[str, str]:
        return asdict(self)


def row(name: str, status: str, detail: str, repair: str = '', link: str = '') -> Row:
    """Return a row; an OK row carries no repair."""
    if status == OK:
        return Row(name, OK, detail)
    return Row(name, status, detail, repair, link)


def worse(*statuses: str) -> str:
    return max(statuses, key=ORDER.index)


def worst(rows: Sequence[Row]) -> str:
    return worse(OK, *(r.status for r in rows))


def grade(value: float, ok: float, fail: float) -> str:
    """Grade a value that should stay small: OK up to `ok`, FAIL above `fail`."""
    return OK if value <= ok else WARN if value <= fail else FAIL


def grade_floor(value: float, ok: float, fail: float) -> str:
    """Grade a value that should stay large: OK from `ok` up, FAIL under `fail`."""
    return OK if value >= ok else WARN if value >= fail else FAIL


def calibration_rows(steps: Sequence[tuple[str, str, bool]], results: dict[str, Any]) -> list[Row]:
    """
    Rate the calibrations: (key, state, optional) per step, last check results by key.

    A required step not calibrated fails; a step whose last check failed warns.
    """
    missing = [key for key, state, optional in steps if state != 'calibrated' and not optional]
    failed = [key for key, *_ in steps if (results.get(key) or {}).get('result') == FAIL]
    if missing:
        detail = 'not calibrated: ' + ', '.join(missing)
        return [row('calibration', FAIL, detail, f'Calibrate {missing[0]}', CALIBRATION_PAGE)]
    if failed:
        detail = 'last check failed: ' + ', '.join(failed)
        return [row('calibration', WARN, detail, f'Calibrate {failed[0]} again', CALIBRATION_PAGE)]
    return [row('calibration', OK, 'every step calibrated')]


def imu_rows(samples: Sequence[Sequence[float]]) -> list[Row]:
    """Rate the IMU at rest from [ax, ay, az, wx, wy, wz] samples of /imu/lsm9ds1."""
    if len(samples) < 10:
        return [row('gyro at rest', FAIL, 'no IMU data', 'Restart teleop', HEALTH_PAGE)]
    n = len(samples)
    mean = [sum(s[k] for s in samples) / n for k in range(6)]
    gyro = max(abs(v) for v in mean[3:])
    g = math.sqrt(sum(v * v for v in mean[:3]))
    error = (g - GRAVITY) / GRAVITY
    gyro_status = grade(gyro, GYRO_OK, GYRO_FAIL)
    gravity_status = grade(abs(error), GRAVITY_OK, GRAVITY_FAIL)
    return [
        row('gyro at rest', gyro_status, f'{gyro:.4f} rad/s', 'Calibrate imu', CALIBRATION_PAGE),
        row('gravity', gravity_status, f'{g:.3f} m/s^2 ({error:+.1%})', 'Calibrate imu',
            CALIBRATION_PAGE),
    ]  # fmt: skip


def odom_rows(poses: Sequence[Sequence[float]]) -> list[Row]:
    """Rate how still the odometry holds: [x, y, yaw] poses of /odom/integrated."""
    if len(poses) < 2:
        return [row('odometry still', FAIL, 'no odometry', 'Restart teleop', HEALTH_PAGE)]
    x0, y0, a0 = poses[0][:3]
    x1, y1, a1 = poses[-1][:3]
    moved = math.hypot(x1 - x0, y1 - y0)
    turned = abs(math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))))
    status = worse(
        grade(moved, DRIFT_OK_M, DRIFT_FAIL_M), grade(turned, TURN_OK_DEG, TURN_FAIL_DEG)
    )
    detail = f'{moved * 1000:.0f} mm, {turned:.2f} deg'
    repair = 'Keep the car still, then calibrate imu and odom'
    return [row('odometry still', status, detail, repair, CALIBRATION_PAGE)]


def lidar_rows(scans: Sequence[tuple[int, int]], seconds: float) -> list[Row]:
    """Rate the lidar from (valid, total) ranges per scan over `seconds`."""
    if not scans or seconds <= 0:
        return [row('lidar', FAIL, 'no scans', 'Check the lidar cable', HEALTH_PAGE)]
    hz = len(scans) / seconds
    share = sum(v for v, _ in scans) / max(sum(t for _, t in scans), 1)
    status = worse(
        grade_floor(hz, SCAN_HZ_OK, SCAN_HZ_FAIL),
        grade_floor(share, SCAN_VALID_OK, SCAN_VALID_FAIL),
    )
    detail = f'{hz:.1f} Hz, {share:.0%} valid'
    return [row('lidar', status, detail, 'Clear the lidar and check its cable', HEALTH_PAGE)]


def rate_row(name: str, hz: float | None, nominal: float) -> Row:
    if hz is None or hz <= 0:
        return row(name, FAIL, 'no data', 'Restart teleop', HEALTH_PAGE)
    status = grade_floor(hz / nominal, RATE_OK, RATE_FAIL)
    return row(name, status, f'{hz:.0f} Hz of {nominal:g}', 'Restart teleop', HEALTH_PAGE)


def teleop_row(active: bool) -> Row:
    if active:
        return row('teleop', OK, 'running')
    return row('teleop', FAIL, 'not running', 'Start teleop: racecar service start teleop')


def battery_row(battery: dict[str, Any] | None) -> Row:
    """Rate the battery from the 8080 /api/battery summary: the guard's level and voltage."""
    if not battery or battery.get('stale') or battery.get('volts') is None:
        return row('battery', WARN, 'no reading', 'Start teleop')
    estimate = battery.get('estimate') or {}
    detail = f"{float(battery['volts']):.2f} V"
    if estimate.get('soc') is not None:
        detail += f", {estimate['soc']:.0f}%"
    level = estimate.get('level', 'ok')
    status = OK if level == 'ok' else WARN if level == 'warn' else FAIL
    return row('battery', status, detail, 'Charge or swap the battery')


def health_rows(report: dict[str, Any] | None, groups: Sequence[str]) -> list[Row]:
    """Rate each named group (power, link, nvme) from a `racecar health` report."""
    rows = []
    checks = (report or {}).get('checks') or []
    for group in groups:
        mine = [c for c in checks if c.get('group') == group]
        if not mine:
            rows.append(row(f'health {group}', FAIL, 'not run', f'Run: racecar health {group}'))
            continue
        bad = [c for c in mine if c.get('status') in (FAIL, WARN)]
        status = worse(OK, *(c.get('status', OK) for c in mine if c.get('status') in ORDER))
        detail = '; '.join(f"{c['name']} {c.get('detail', '')}" for c in bad)
        detail = detail or f'{len(mine)} checks'
        rows.append(row(f'health {group}', status, detail, 'See Health checks', HEALTH_PAGE))
    return rows
