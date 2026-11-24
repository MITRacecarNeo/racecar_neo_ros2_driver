"""
`racecar check` result rows worded for the pages (Car Profile Checks, the Mapper checks sheet).

`racecar check` and readiness.py keep their own wording for the terminal;
these functions read the same rows and return them in the page wording:
the tile names, whole words, plurals and units, and repairs that name the
step rather than a command.
"""

from __future__ import annotations

import re
from typing import Any

STEP_NAMES = {
    'imu': 'IMU', 'mag': 'magnetometer', 'realsense': 'RealSense IMU', 'lidar': 'lidar',
    'camera': 'camera mount', 'odom': 'odometry', 'steering': 'steering',
    'dynamics': 'vehicle dynamics',
}  # fmt: skip
CHECK_NAMES = {'health link': 'Board link', 'health nvme': 'Data drive', 'health power': 'Power'}
REPAIRS = {
    'Start teleop: racecar service start teleop': 'Start teleop',
    'Keep the car still, then calibrate imu and odom': (
        'Keep the car still. Then calibrate the IMU and odometry.'
    ),
}
NO_VALUE = {'no data', 'no reading'}
PLURAL = re.compile(r'(\d+) ((?:[A-Za-z]+ )*)([A-Za-z]+)\(s\)')
KERNEL_LOG = re.compile(r'^kernel log (\d+) NVMe, (\d+) PCIe error line\(s\) this boot')
STEP_LIST = re.compile(r'(last check failed|not calibrated): ([\w, ]+)')


def _plural(m: re.Match[str]) -> str:
    n = int(m.group(1))
    return f'{n} {m.group(2)}{m.group(3)}{"" if n == 1 else "s"}'


def _steps(keys: str) -> str:
    return ', '.join(STEP_NAMES.get(k.strip(), k.strip()) for k in keys.split(','))


def check_detail(detail: str) -> str:
    """Return a check result's detail for the page: whole words, plurals, units."""
    if detail.strip().lower() in NO_VALUE:
        return 'No reading'
    m = KERNEL_LOG.match(detail)
    if m:
        nvme, pcie = int(m.group(1)), int(m.group(2))
        parts = [f'{n} {what} error{"" if n == 1 else "s"}' for n, what in
                 ((nvme, 'NVMe'), (pcie, 'PCIe')) if n]  # fmt: skip
        return (', '.join(parts) or 'No errors') + ' this boot'
    m = STEP_LIST.fullmatch(detail)
    if m:
        detail = f'{m.group(1)}: {_steps(m.group(2))}'
    detail = PLURAL.sub(_plural, detail).replace('m/s^2', 'm/s²')
    return detail[:1].upper() + detail[1:] if detail[:1].islower() else detail


def check_repair(repair: str) -> str:
    """Return a repair as the page shows it: the step's name, no shell command."""
    if repair in REPAIRS:
        return REPAIRS[repair]
    m = re.fullmatch(r'Calibrate (\w+)( again)?', repair)
    if m:
        step = STEP_NAMES.get(m.group(1), m.group(1))
        return f'Recalibrate the {step}' if m.group(2) else f'Calibrate the {step}'
    return repair


def check_row(row: dict[str, Any]) -> dict[str, Any]:
    """Return one `racecar check` result row worded for the page."""
    name = str(row.get('name', ''))
    name = CHECK_NAMES.get(name, name[:1].upper() + name[1:])
    # Results saved before the pages moved under a name path link from the root.
    link = str(row.get('link', '')).lstrip('/')
    return {**row, 'name': name, 'detail': check_detail(str(row.get('detail', ''))),
            'repair': check_repair(str(row.get('repair', ''))), 'link': link}  # fmt: skip


def checks_last(last: dict[str, Any] | None) -> dict[str, Any] | None:
    """Return the saved `racecar check` results with every row worded for the page."""
    if not last:
        return last
    return {**last, 'rows': [check_row(r) for r in last.get('rows') or []]}
