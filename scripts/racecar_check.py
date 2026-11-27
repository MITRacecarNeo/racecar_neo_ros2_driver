#!/usr/bin/env python3
"""
`racecar check`: the parked checks, with the car still.

    racecar check              calibration, IMU at rest, odometry still, lidar,
                               topic rates, teleop, battery (about 5 seconds)
    racecar check --all        the same, then racecar health power, link and nvme
    racecar check imu lidar    only the named checks

Rows use the marks of `racecar status`; a row that is not OK names its repair.
The results are kept in checks/last.json under the data root, for the Car
Profile Checks page and the map viewer. The exit code is 1 when a check fails.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any
import urllib.request

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'calibration'))

from racecar_neo_ros2_driver import apps, car_profile, readiness, units  # noqa: E402, I100
from racecar_neo_ros2_driver.paths import config_dir  # noqa: E402
from racecar_neo_ros2_driver.term import paint, use_color  # noqa: E402
from sysinfo import DIAGNOSTIC_RATE_KEY, LINK_STATUS_NAME  # noqa: E402

PARKED = ('calibration', 'imu', 'odometry', 'lidar', 'rates', 'teleop', 'battery')
HEALTH = ('power', 'link', 'nvme')
NAMES = PARKED + HEALTH
# Checks that need the topics.
SAMPLED = ('imu', 'odometry', 'lidar', 'rates')
SAMPLE_SEC = 3.0
WAIT_SEC = 5.0  # for the first messages
MARK = {readiness.OK: '[ OK ]', readiness.WARN: '[WARN]', readiness.FAIL: '[FAIL]'}
# Rates the nodes report on /diagnostics (counting 200 Hz streams here would
# measure this script, not the car): label and nominal rate.
DIAG_RATES = {
    LINK_STATUS_NAME: ('PIT sensor rate', 200.0),
    'camera: color': ('camera color rate', 60.0),
    'camera: depth': ('camera depth rate', 30.0),
}
LAST_FILE = 'last.json'


def boot_id() -> str:
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return ''


def teleop_started() -> str:
    out = subprocess.run(
        [*units.systemctl_argv(), 'show', '-p', 'ActiveEnterTimestamp', '--value', units.TELEOP],
        capture_output=True, text=True, check=False,
    )  # fmt: skip
    return out.stdout.strip()


def teleop_active() -> bool:
    out = subprocess.run([*units.systemctl_argv(), 'is-active', units.TELEOP],
                         capture_output=True, text=True, check=False)  # fmt: skip
    return out.stdout.strip() == 'active'


def fetch_battery() -> dict[str, Any] | None:
    url = f'http://127.0.0.1:{apps.home_port()}/api/battery'
    try:
        with urllib.request.urlopen(url, timeout=2) as r:  # noqa: S310  local page
            return dict(json.loads(r.read())).get('battery')
    except (OSError, ValueError):
        return None


def calibration_mtime() -> float:
    """Newest change to the car's parameter files; a newer one makes a pass stale."""
    params = car_profile.profile_dir() / 'params'
    return max((p.stat().st_mtime for p in params.glob('*.yaml')), default=0.0)


class Sample:
    """A few seconds of the topics the parked checks read."""

    def __init__(self) -> None:
        self.imu: list[list[float]] = []
        self.poses: list[list[float]] = []
        self.scans: list[tuple[int, int]] = []
        self.diag: dict[str, float] = {}
        self.seconds = 0.0
        self._lock = threading.Lock()
        self._on = False

    def run(self, seconds: float) -> None:
        import math

        import rclpy
        from rclpy.qos import qos_profile_sensor_data

        from diagnostic_msgs.msg import DiagnosticArray  # noqa: I100
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan

        rclpy.init()
        node = rclpy.create_node('racecar_check')

        def on_imu(msg: Any) -> None:
            if self._on:
                a, w = msg.linear_acceleration, msg.angular_velocity
                with self._lock:
                    self.imu.append([a.x, a.y, a.z, w.x, w.y, w.z])

        def on_odom(msg: Any) -> None:
            if self._on:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self._lock:
                    self.poses.append([p.x, p.y, yaw])

        def on_scan(msg: Any) -> None:
            if self._on:
                valid = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
                with self._lock:
                    self.scans.append((valid, len(msg.ranges)))

        def on_diag(msg: Any) -> None:
            for status in msg.status:
                for kv in status.values:
                    if status.name in DIAG_RATES and kv.key == DIAGNOSTIC_RATE_KEY:
                        try:
                            self.diag[status.name] = float(kv.value)
                        except ValueError:
                            pass

        qos = qos_profile_sensor_data
        node.create_subscription(Imu, '/imu/lsm9ds1', on_imu, qos)
        node.create_subscription(Odometry, '/odom/integrated', on_odom, qos)
        node.create_subscription(LaserScan, '/scan', on_scan, qos)
        node.create_subscription(DiagnosticArray, '/diagnostics', on_diag, 10)
        try:
            from rclpy.experimental import EventsExecutor as Executor
        except ImportError:
            from rclpy.executors import SingleThreadedExecutor as Executor
        executor = Executor()
        executor.add_node(node)
        # Discovery first, then a clean window.
        end = time.monotonic() + min(WAIT_SEC, 2.0)
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        self._on = True
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            executor.spin_once(timeout_sec=0.05)
        self._on = False
        self.seconds = time.monotonic() - start
        node.destroy_node()
        rclpy.try_shutdown()


def sampled_rows(names: list[str], sample: Sample) -> list[readiness.Row]:
    rows: list[readiness.Row] = []
    if 'imu' in names:
        rows += readiness.imu_rows(sample.imu)
    if 'odometry' in names:
        rows += readiness.odom_rows(sample.poses)
    if 'lidar' in names:
        rows += readiness.lidar_rows(sample.scans, sample.seconds)
    if 'rates' in names:
        for name, (label, nominal) in DIAG_RATES.items():
            if name in sample.diag:
                rows.append(readiness.rate_row(label, sample.diag[name], nominal))
            elif name == LINK_STATUS_NAME:
                rows.append(readiness.rate_row(label, None, nominal))
            else:
                rows.append(readiness.row(label, readiness.WARN, 'camera off or not reporting',
                                          'Turn the camera on in 8080 Features'))  # fmt: skip
    return rows


def calibration_rows() -> list[readiness.Row]:
    import calibrate

    steps = [
        (key, state, step.optional)
        for step, (key, state, _) in zip(
            calibrate.STEPS, calibrate.status_rows(config_dir()), strict=True
        )
    ]
    results = car_profile.read_yaml(car_profile.profile_dir() / 'car.yaml').get('calibration')
    return readiness.calibration_rows(steps, results or {})


def health_report(group: str) -> dict[str, Any] | None:
    """Run `racecar health GROUP` and return its report."""
    from racecar_neo_ros2_driver.data_root import data_dir

    before = set(data_dir('health').glob('*/report.json'))
    subprocess.run([sys.executable, str(HERE / 'health.py'), group], check=False)
    new = sorted(set(data_dir('health').glob('*/report.json')) - before)
    if not new:
        return None
    try:
        return dict(json.loads(new[-1].read_text()))
    except (OSError, ValueError):
        return None


def print_rows(rows: list[readiness.Row], color: bool) -> None:
    width = max((len(r.name) for r in rows), default=0)
    for r in rows:
        mark = paint(MARK[r.status], r.status, color)
        print(f'  {mark}  {r.name:<{width}}  {r.detail}'.rstrip())
        if r.repair:
            print(f'  {"":6}  {"":<{width}}  {r.repair}')


ROW_GROUP = {'gyro at rest': 'imu', 'gravity': 'imu', 'odometry still': 'odometry'}


def group_of(row_name: str) -> str:
    """Return the check a row belongs to."""
    if row_name.startswith('health '):
        return row_name.removeprefix('health ')
    if row_name.endswith(' rate'):
        return 'rates'
    return ROW_GROUP.get(row_name, row_name)


def save(rows: list[readiness.Row], names: list[str]) -> Path:
    from racecar_neo_ros2_driver.data_root import data_dir

    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    path = data_dir('checks') / LAST_FILE
    # Checks not run this time keep their last rows, so one check leaves the rest.
    previous = read_last() or {}
    old = [{**r, 'group': r.get('group') or group_of(r['name'])} for r in previous.get('rows', [])]
    kept = [r for r in old if r['group'] not in names]
    fresh = [{**r.view(), 'group': group_of(r.name), 'time': now} for r in rows]
    merged = kept + fresh
    data = {
        'time': now,
        'boot_id': boot_id(),
        'teleop_started': teleop_started(),
        'calibration_mtime': calibration_mtime(),
        'checks': names,
        'result': readiness.worse(readiness.OK, *(r['status'] for r in merged)),
        'rows': merged,
    }
    path.write_text(json.dumps(data, indent=1))
    return path


def read_last() -> dict[str, Any] | None:
    from racecar_neo_ros2_driver.data_root import data_dir

    try:
        return dict(json.loads((data_dir('checks') / LAST_FILE).read_text()))
    except (OSError, ValueError):
        return None


def gate(last: dict[str, Any] | None = None) -> dict[str, Any]:
    """
    Return whether the last parked checks still let mapping start: {passed, reason, time}.

    A pass needs every parked check in the last results with none failed (a
    WARN passes), from this boot, this teleop start, and no calibration
    written since.
    """
    last = read_last() if last is None else last
    if not last:
        return {'passed': False, 'reason': 'The checks have not run.', 'time': ''}
    rows = [r for r in last.get('rows', []) if r.get('group') in PARKED]
    out = {'passed': False, 'time': last.get('time', ''), 'result': last.get('result', '')}
    if {r.get('group') for r in rows} < set(PARKED):
        return {**out, 'reason': 'Not every check has run.'}
    if any(r.get('status') == readiness.FAIL for r in rows):
        return {**out, 'reason': 'A check failed.'}
    if last.get('boot_id') != boot_id():
        return {**out, 'reason': 'The car restarted since the checks ran.'}
    if last.get('teleop_started') != teleop_started():
        return {**out, 'reason': 'Teleop restarted since the checks ran.'}
    if calibration_mtime() > float(last.get('calibration_mtime') or 0.0):
        return {**out, 'reason': 'A calibration changed since the checks ran.'}
    return {**out, 'passed': True, 'reason': ''}


def require_pass(out: Any = print) -> bool:
    """Run the parked checks unless a pass stands; return whether mapping may start."""
    if gate()['passed']:
        return True
    rows = run(list(PARKED))
    out('')
    print_rows(rows, use_color())
    save(rows, list(PARKED))
    return readiness.worst(rows) != readiness.FAIL


def run(names: list[str], seconds: float = SAMPLE_SEC) -> list[readiness.Row]:
    rows: list[readiness.Row] = []
    if 'calibration' in names:
        rows += calibration_rows()
    if 'teleop' in names:
        rows.append(readiness.teleop_row(teleop_active()))
    if 'battery' in names:
        rows.append(readiness.battery_row(fetch_battery()))
    if any(n in SAMPLED for n in names):
        print(f'Keep the car still for {seconds:.0f} seconds.', flush=True)
        sample = Sample()
        sample.run(seconds)
        rows += sampled_rows(names, sample)
    for group in HEALTH:
        if group in names:
            print(f'\nRunning racecar health {group}.', flush=True)
            rows += readiness.health_rows(health_report(group), [group])
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='racecar check', description=__doc__.split('\n\n')[0])
    parser.add_argument(
        'names', nargs='*', metavar='check', help=f'checks to run: {", ".join(NAMES)}'
    )
    parser.add_argument('--all', action='store_true', help='add racecar health power, link, nvme')
    parser.add_argument('--seconds', type=float, default=SAMPLE_SEC, help='sample time')
    args = parser.parse_args(argv)
    unknown = [n for n in args.names if n not in NAMES]
    if unknown:
        parser.error(f'unknown check: {", ".join(unknown)}')
    names = list(args.names) or list(PARKED) + (list(HEALTH) if args.all else [])
    rows = run(names, args.seconds)
    print()
    print_rows(rows, use_color())
    result = readiness.worst(rows)
    print(f'\n  RESULT {paint(result, result, use_color(), bold=True)}')
    save(rows, names)
    return 1 if result == readiness.FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
