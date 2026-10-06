#!/usr/bin/env python3
"""
Guided whole-car calibration behind `racecar calibrate`.

Runs the calibrate_*.py tools in this folder in a fixed order, each with its
own prompts, then shows old and new values, backs the results up to
~/.config/racecar/calibration/, and offers to restart teleop so the nodes load
them. `racecar calibrate status` lists each calibration's state without ROS.
`racecar calibrate check` runs each tool's --check against the live sensors,
the last step of car prep: calibrate, restart teleop, check.

The tools write config/<name>.local.yaml in both the source and the installed
config directory (calibrate_common.output_dirs), so no rebuild is needed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibrate_common import (  # noqa: E402, I100
    BACKUP_DIR,
    criteria_lines,
    Criterion,
    paint,
    read_line,
    SESSION_ENV,
    source_config_dir,
    use_color,
)

TELEOP_UNIT = 'racecar-teleop'
RUN_PROMPT = 'Press Enter to run, s to skip, and q to stop: '
RUN_PROMPT_BACK = 'Press Enter to run, s to skip, b to go back a step, and q to stop: '
STEERING_SETUP = (
    'Use a clear area of about 3 by 3 meters surrounded by walls or furniture.\n'
    'Connect the controller. Hold RB to autonomously drive the car.'
)
DYNAMICS_SETUP = (
    'Use a clear straight path of about 4 meters with walls or furniture nearby.\n'
    'Use a clear area of about 2.3 by 1.6 meters for driving circles.\n'
    'Connect the controller. Hold RB to autonomously drive the car.'
)
# calibrate_dynamics.py writes these; per speed (0.25, 0.5, 1.0) and per side.
DYNAMICS_KEYS = tuple(
    [
        f'{name}_{tag}'
        for tag in ('025', '05', '10')
        for name in ('steady_speed_mps', 'rise_90_s', 'stop_distance_mm', 'stop_time_s')
    ]
    + [
        f'{name}_{side}'
        for side in ('right', 'left')
        for name in (
            'turn_radius_mm',
            'steer_inner_deg',
            'steer_outer_deg',
            'outer_front_track_diameter_mm',
            'encoder_turn_overread',
            'servo_center_to_lock_s',
        )
    ]
    + ['servo_lock_to_lock_s']
)
ODOM_SETUP = (
    'Use a room with walls or furniture nearby.\n'
    'Clear about 2 meters around the car for driving circles.\n'
    'Clear a straight path of 3 to 5 meters that ends at a wall or furniture.'
)


@dataclass(frozen=True)
class Step:
    """One calibration in the sequence."""

    key: str
    label: str
    tool: str
    position: str
    local_name: str
    keys: tuple[str, ...]
    check: str = ''  # how to place the car for the tool's --check; empty: no check


# Order: hand-held IMU steps first, then steps with the car on the floor.
STEPS = (
    Step(
        'imu',
        'LSM9DS1 accel bias and scale, gyro bias',
        'calibrate_imu.py',
        'Place the car on a table.\n'
        'The instructions will direct you to hold the car in 6 poses.',
        'lsm9ds1_cal',
        ('accelerometer.bias', 'accelerometer.scale', 'gyroscope.bias'),
        check='Place the car on a level surface.',
    ),
    Step(
        'mag',
        'LSM9DS1 magnetometer',
        'calibrate_mag.py',
        'Hold the car in your hands, away from large metal structures.\n'
        'The instructions will direct you to turn the car about 3 axes.',
        'lsm9ds1_mag_cal',
        ('magnetometer.hard_iron_bias',),
        check='Place the car down flat on the floor and away from large metal structures.',
    ),
    Step(
        'realsense',
        'RealSense accel bias and scale, gyro bias',
        'calibrate_realsense_imu.py',
        'Place the car on a table.\n'
        'The instructions will direct you to hold the car in 6 poses.',
        'realsense_cal',
        ('realsense_accel_bias', 'realsense_accel_scale', 'realsense_gyro_bias'),
        check='Place the car on a level surface.',
    ),
    Step(
        'lidar',
        'Lidar yaw',
        'calibrate_lidar.py',
        'Place the car down on the floor facing a flat, clear wall.\n'
        'Ensure that the front bumper is 0.5 to 2 meters away from the wall.',
        'frames',
        ('laser_yaw_deg',),
        check='Place the car down facing a flat, clear wall, about 0.5 to 2 meters away '
        'from it.',
    ),
    Step(
        'odom',
        'Odometry yaw and speed scale',
        'calibrate_odom.py',
        ODOM_SETUP,
        'odom',
        ('odometry.yaw_rate_scale', 'odometry.speed_scale'),
        check=ODOM_SETUP,
    ),
    Step(
        'steering',
        'Steering trim',
        'calibrate_steering.py',
        STEERING_SETUP,
        'throttle',
        ('steering_trim',),
        check=STEERING_SETUP,
    ),
    # Measured for the simulator; nothing reads it back, so there is no check.
    Step(
        'dynamics',
        'Vehicle dynamics for the simulator',
        'calibrate_dynamics.py',
        DYNAMICS_SETUP,
        'dynamics',
        DYNAMICS_KEYS,
    ),
)
STEP_KEYS = [s.key for s in STEPS]


def setup_text(text: str) -> str:
    """Return a step's setup lines under a `Setup:` label, one instruction per line."""
    return 'Setup: ' + text.replace('\n', '\n       ')


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


def status_verdict(step: Step, state: str, detail: str) -> str:
    """Return the color class of a status row: PASS, WARN, or FAIL."""
    if state == 'calibrated':
        return 'PASS' if detail.endswith('backed up') else 'WARN'
    # Uncalibrated sensors run on zero bias and unit scale; dynamics is optional.
    return 'FAIL' if step.check else 'WARN'


def status_lines(
    config_dir: Path, backup_dir: Path = BACKUP_DIR, color: bool = False
) -> list[str]:
    """Return `racecar calibrate status`: one block per step, blank lines between."""
    lines: list[str] = []
    for step, (key, state, detail) in zip(STEPS, status_rows(config_dir, backup_dir), strict=True):
        if lines:
            lines.append('')
        header = f'{key}: {step.label}'
        lines.append(f'\033[1m{header}\033[0m' if color else header)
        verdict = status_verdict(step, state, detail)
        lines.append(
            f'  {paint(state, verdict, color, bold=True)}  {paint(detail, verdict, color)}'
        )
        values = step_values(step, config_dir)
        width = max((len(k) for k in values), default=0)
        lines += [f'  {k:<{width}}  {fmt(values[k], 7)}' for k in step.keys if k in values]
    return lines


def status_rows(config_dir: Path, backup_dir: Path = BACKUP_DIR) -> list[tuple[str, str, str]]:
    """Return (step, state, detail) for each step."""
    rows = []
    for step in STEPS:
        local = config_dir / f'{step.local_name}.local.yaml'
        values = step_values(step, config_dir)
        if values:
            stamp = datetime.fromtimestamp(local.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
            rows.append((step.key, 'calibrated', f'{stamp}, {backup_state(local, backup_dir)}'))
        elif (config_dir / f'{step.local_name}.yaml').exists():
            rows.append((step.key, 'default', 'committed defaults; not calibrated on this car'))
        else:
            rows.append((step.key, 'default', 'not measured on this car'))
    return rows


def fmt(value: Any, digits: int = 4) -> str:
    if isinstance(value, list):
        return '[' + ', '.join(fmt(v, digits) for v in value) + ']'
    return f'{value:.{digits}g}' if isinstance(value, float) else str(value)


def systemctl() -> list[str]:
    return shlex.split(os.environ.get('RACECAR_SYSTEMCTL', 'systemctl'))


def teleop_active() -> bool:
    result = subprocess.run(
        [*systemctl(), 'is-active', TELEOP_UNIT], capture_output=True, text=True, check=False
    )
    return result.stdout.strip() == 'active'


def ask(prompt: str) -> str:
    """Prompt for one line (calibrate_common.read_line); end of input reads as q."""
    try:
        return read_line(prompt).strip().lower()
    except (EOFError, OSError):
        return 'q'


def walk_steps(steps: list[Step], header: Any, run: Any) -> list[Any]:
    """
    Prompt for each step in order: Enter runs it, s skips, b goes back a step, q stops.

    `header(i)` prints the step's banner and `run(i)` returns its result entry.
    A skipped step gives (step, 'skipped') and the steps left by q give
    (step, 'not run'). Returns one entry per step reached, in order.
    """
    entries: dict[int, Any] = {}
    i = 0
    while i < len(steps):
        header(i)
        answer = ask(RUN_PROMPT_BACK if i > 0 else RUN_PROMPT)
        if answer == 'b':
            i = max(i - 1, 0)
            continue
        if answer == 'q':
            for k in range(i, len(steps)):
                entries[k] = (steps[k], 'not run')
            break
        entries[i] = (steps[i], 'skipped') if answer == 's' else run(i)
        i += 1
    return [entries[k] for k in sorted(entries)]


def run_steps(steps: list[Step], config_dir: Path, backup_dir: Path = BACKUP_DIR) -> int:
    """Run the steps in order; return 0 when every attempted step succeeded."""
    # A step run again keeps its values from before its first run, for the summary.
    first_before: dict[int, dict[str, Any]] = {}

    def header(i: int) -> None:
        print(f'\n=== {i + 1}/{len(steps)} {steps[i].label} ===')
        print(setup_text(steps[i].position))

    def run(i: int) -> tuple[Step, str, dict[str, Any], dict[str, Any]]:
        step = steps[i]
        before = first_before.setdefault(i, step_values(step, config_dir))
        rc = subprocess.run(
            [sys.executable, str(HERE / step.tool)],
            check=False,
            env={**os.environ, SESSION_ENV: '1'},
        ).returncode
        return step, 'done' if rc == 0 else 'failed', before, step_values(step, config_dir)

    results = [e if len(e) == 4 else (*e, {}, {}) for e in walk_steps(steps, header, run)]

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
        print('Restarting teleop uses sudo, which may ask for your password.')
        if ask('Restart teleop now so the nodes load the new values? [y/N] ') == 'y':
            subprocess.run(restart, check=False)
        else:
            print(f'Apply later: {shlex.join(restart)}')
        print('Then verify: racecar calibrate check')
    failed = [step.key for step, outcome, *_ in results if outcome == 'failed']
    return 1 if failed else 0


def read_report(path: Path) -> tuple[str, list[Criterion]] | None:
    """Return (overall, criteria) from a tool's --report file, or None if absent."""
    try:
        data = json.loads(path.read_text())
        return data['overall'], [Criterion(**c) for c in data['criteria']]
    except (OSError, ValueError, KeyError, TypeError):
        return None


def summary_lines(
    results: list[tuple[Step, str, list[Criterion]]], color: bool = False
) -> list[str]:
    """Render the end-of-run table: each step's verdict, then each criterion."""
    lines = ['', '=== Check summary ===']
    for step, verdict, criteria in results:
        mark = paint(f'{verdict:7s}', verdict, color, bold=True)
        lines.append(f'{step.key:10s} {mark} {step.label}')
        lines += criteria_lines(criteria, indent='    ', color=color)
    counts = {
        v: sum(1 for _, verdict, _ in results if verdict == v) for v in ('PASS', 'WARN', 'FAIL')
    }
    skipped = sum(1 for _, verdict, _ in results if verdict == 'skipped')
    not_run = sum(1 for _, verdict, _ in results if verdict == 'not run')
    tallies = '  '.join(
        paint(f'{counts[v]} {v.lower()}', v, color and counts[v] > 0)
        for v in ('PASS', 'WARN', 'FAIL')
    )
    worst = 'FAIL' if counts['FAIL'] else 'WARN' if counts['WARN'] else 'PASS'
    lines.append(
        f'RESULT {paint(worst, worst, color, bold=True)}  {tallies}  {skipped} skipped'
        + (f'  {not_run} not run' if not_run else '')
    )
    return lines


def run_checks(steps: list[Step]) -> int:
    """Run each step's --check; print a per-criterion summary; return 1 when any failed."""
    with tempfile.TemporaryDirectory() as tmp:

        def header(i: int) -> None:
            print(f'\n=== Check {i + 1}/{len(steps)}: {steps[i].label} ===')
            print(setup_text(steps[i].check))

        def run(i: int) -> tuple[Step, str, list[Criterion]]:
            step = steps[i]
            report = Path(tmp) / f'{step.key}.json'
            report.unlink(missing_ok=True)
            cmd = [sys.executable, str(HERE / step.tool), '--check', '--report', str(report)]
            rc = subprocess.run(cmd, check=False).returncode
            parsed = read_report(report)
            if parsed is None:
                criteria = [
                    Criterion('check', f'exit {rc}', 'a result', 'FAIL', 'see output above')
                ]
                return step, 'FAIL', criteria
            verdict, criteria = parsed
            return step, verdict, criteria

        results = [e if len(e) == 3 else (*e, []) for e in walk_steps(steps, header, run)]
    print('\n'.join(summary_lines(results, use_color())))
    return 1 if any(verdict == 'FAIL' for _, verdict, _ in results) else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='racecar calibrate',
        description='Calibrate the car: IMUs, magnetometer, lidar yaw, odometry, steering, '
        'dynamics. '
        '`status` lists what is calibrated.',
    )
    parser.add_argument(
        'steps',
        nargs='*',
        metavar='step',
        help=f'status; check [steps]; or steps to run in order ({", ".join(STEP_KEYS)}); '
        'default: all available',
    )
    args = parser.parse_args(argv)

    config_dir = source_config_dir(Path(__file__))
    if config_dir is None:
        print('No package config directory found.', file=sys.stderr)
        return 1

    if args.steps == ['status']:
        print('\n'.join(status_lines(config_dir, color=use_color())))
        return 0

    checking = bool(args.steps) and args.steps[0] == 'check'
    if checking:
        args.steps = args.steps[1:]
    unknown = [s for s in args.steps if s not in STEP_KEYS]
    if unknown:
        parser.print_usage(sys.stderr)
        print(f'unknown step: {", ".join(unknown)}', file=sys.stderr)
        return 2
    chosen = [s for s in STEPS if not args.steps or s.key in args.steps]
    if checking:
        chosen = [s for s in chosen if s.check]
        if not chosen:
            return 2
    if not teleop_active():
        print(
            f'{TELEOP_UNIT} is not running. Start it with racecar service start teleop, '
            'then run racecar calibrate again.'
        )
        return 1
    return run_checks(chosen) if checking else run_steps(chosen, config_dir)


if __name__ == '__main__':
    sys.exit(main())
