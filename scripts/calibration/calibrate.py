#!/usr/bin/env python3
"""
Guided whole-car calibration behind `racecar calibrate`.

Runs the calibrate_*.py tools in this folder in a fixed order, each with its
own prompts, then shows old and new values, backs the results up to
~/.config/racecar/calibration/, and offers to restart teleop so the nodes load
them. `racecar calibrate status` lists each calibration's state without ROS.

The tools write config/<name>.local.yaml in both the source and the installed
config directory (calibrate_common.output_dirs), so no rebuild is needed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibrate_common import BACKUP_DIR, source_config_dir  # noqa: E402, I100

TELEOP_UNIT = 'racecar-teleop'


@dataclass(frozen=True)
class Step:
    """One calibration in the sequence. tool None: not written yet (`pending` says when)."""

    key: str
    label: str
    tool: str | None
    position: str
    local_name: str
    keys: tuple[str, ...]
    pending: str = ''


# Order: hand-held IMU steps first, then steps with the car on the floor.
STEPS = (
    Step(
        'imu',
        'LSM9DS1 accel bias and scale, gyro bias',
        'calibrate_imu.py',
        'car held in 6 orientations',
        'lsm9ds1_cal',
        ('accelerometer.bias', 'accelerometer.scale', 'gyroscope.bias'),
    ),
    Step(
        'mag',
        'LSM9DS1 magnetometer',
        'calibrate_mag.py',
        'car rotated by hand about all axes',
        'lsm9ds1_mag_cal',
        ('magnetometer.hard_iron_bias',),
    ),
    Step(
        'realsense',
        'RealSense accel bias and scale, gyro bias',
        'calibrate_realsense_imu.py',
        'car held in 6 orientations',
        'realsense_cal',
        ('realsense_accel_bias', 'realsense_accel_scale', 'realsense_gyro_bias'),
    ),
    Step(
        'lidar',
        'Lidar yaw',
        'calibrate_lidar.py',
        'car squared to a flat wall ahead (wheel centres equal within 1 mm)',
        'frames',
        ('laser_yaw_deg',),
    ),
    Step(
        'camera',
        'Camera pitch, roll and height',
        None,
        'car on a flat floor, clear ahead',
        'frames',
        ('camera_pitch_deg', 'camera_roll_deg', 'camera_bottom_z_mm'),
        'arrives before phase 3',
    ),
    Step(
        'odom',
        'Odometry scale',
        None,
        '3 m tape run and a spin',
        'odom',
        (),
        'arrives with phase 2',
    ),
)
STEP_KEYS = [s.key for s in STEPS]


def read_values(path: Path) -> dict[str, Any]:
    """Return a local YAML's values: a ROS parameter file's parameters, or a plain mapping."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    if len(data) == 1:
        (inner,) = data.values()
        if isinstance(inner, dict) and 'ros__parameters' in inner:
            return dict(inner['ros__parameters'] or {})
    return data


def step_values(step: Step, config_dir: Path) -> dict[str, Any]:
    values = read_values(config_dir / f'{step.local_name}.local.yaml')
    return {k: values[k] for k in step.keys if k in values}


def backup_state(local: Path, backup_dir: Path) -> str:
    backup = backup_dir / local.name
    if not backup.exists():
        return 'no backup'
    return 'backed up' if backup.read_bytes() == local.read_bytes() else 'backup differs'


def status_rows(config_dir: Path, backup_dir: Path = BACKUP_DIR) -> list[tuple[str, str, str]]:
    """Return (step, state, detail) for each step."""
    rows = []
    for step in STEPS:
        local = config_dir / f'{step.local_name}.local.yaml'
        values = step_values(step, config_dir)
        if step.tool is None:
            rows.append((step.key, 'pending', step.pending))
        elif values:
            stamp = datetime.fromtimestamp(local.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
            rows.append((step.key, 'calibrated', f'{stamp}, {backup_state(local, backup_dir)}'))
        else:
            rows.append((step.key, 'default', 'committed defaults; not calibrated on this car'))
    return rows


def fmt(value: Any) -> str:
    if isinstance(value, list):
        return '[' + ', '.join(f'{v:.4g}' if isinstance(v, float) else str(v) for v in value) + ']'
    return f'{value:.4g}' if isinstance(value, float) else str(value)


def systemctl() -> list[str]:
    return shlex.split(os.environ.get('RACECAR_SYSTEMCTL', 'systemctl'))


def teleop_active() -> bool:
    result = subprocess.run(
        [*systemctl(), 'is-active', TELEOP_UNIT], capture_output=True, text=True, check=False
    )
    return result.stdout.strip() == 'active'


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return 'q'


def run_steps(steps: list[Step], config_dir: Path, backup_dir: Path = BACKUP_DIR) -> int:
    """Run the steps in order; return 0 when every attempted step succeeded."""
    results: list[tuple[Step, str, dict[str, Any], dict[str, Any]]] = []
    for i, step in enumerate(steps, start=1):
        print(f'\n=== {i}/{len(steps)} {step.label} ===')
        print(f'Position: {step.position}')
        answer = ask('Enter to run, s to skip, q to stop: ')
        if answer == 'q':
            break
        if answer == 's':
            results.append((step, 'skipped', {}, {}))
            continue
        before = step_values(step, config_dir)
        rc = subprocess.run([sys.executable, str(HERE / str(step.tool))], check=False).returncode
        results.append(
            (step, 'done' if rc == 0 else 'failed', before, step_values(step, config_dir))
        )

    print('\n=== Summary ===')
    for step, outcome, before, after in results:
        print(f'{step.key:10s} {outcome}')
        for key in step.keys if outcome == 'done' else ():
            old = fmt(before[key]) if key in before else 'default'
            print(f'    {key}: {old} -> {fmt(after.get(key, "?"))}')

    changed = {
        config_dir / f'{step.local_name}.local.yaml'
        for step, outcome, *_ in results
        if outcome == 'done'
    }
    if changed:
        backup_dir.mkdir(parents=True, exist_ok=True)
        for local in sorted(changed):
            if local.exists():
                shutil.copy2(local, backup_dir / local.name)
                print(f'Backed up {local.name} to {backup_dir}/')
        restart = [*shlex.split(os.environ.get('RACECAR_SUDO', 'sudo')), *systemctl()]
        restart += ['restart', TELEOP_UNIT]
        if ask('Restart teleop now so the nodes load the new values? [y/N] ') == 'y':
            subprocess.run(restart, check=False)
        else:
            print(f'Apply later: {shlex.join(restart)}')
    failed = [step.key for step, outcome, *_ in results if outcome == 'failed']
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='racecar calibrate',
        description='Calibrate the car: IMUs, magnetometer, lidar yaw. '
        '`status` lists what is calibrated.',
    )
    parser.add_argument(
        'steps',
        nargs='*',
        metavar='step',
        help=f'status, or steps to run in order ({", ".join(STEP_KEYS)}); default: all available',
    )
    args = parser.parse_args(argv)

    config_dir = source_config_dir(Path(__file__))
    if config_dir is None:
        print('No package config directory found.', file=sys.stderr)
        return 1

    if args.steps == ['status']:
        for key, state, detail in status_rows(config_dir):
            print(f'{key:10s} {state:11s} {detail}')
        return 0

    unknown = [s for s in args.steps if s not in STEP_KEYS]
    if unknown:
        parser.print_usage(sys.stderr)
        print(f'unknown step: {", ".join(unknown)}', file=sys.stderr)
        return 2
    chosen = [s for s in STEPS if not args.steps or s.key in args.steps]
    pending = [s for s in chosen if s.tool is None]
    for step in pending:
        print(f'{step.key}: not available yet ({step.pending}); skipped')
    chosen = [s for s in chosen if s.tool is not None]
    if not chosen:
        return 2 if args.steps else 0

    if not teleop_active():
        print(f'{TELEOP_UNIT} is not active; the tools read its topics. Start it first.')
        return 1
    return run_steps(chosen, config_dir)


if __name__ == '__main__':
    sys.exit(main())
