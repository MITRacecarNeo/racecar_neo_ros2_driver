#!/usr/bin/env python3
"""
Shared plumbing for the calibrate_*.py tools.

Results go to config/<name>.local.yaml (gitignored) in the source tree and in
the installed share config dir when it exists. The launch files load them
after the tracked zero-default YAMLs, so the local values win.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import sys
import termios
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import yaml

PACKAGE = 'racecar_neo_ros2_driver'
BACKUP_DIR = Path.home() / '.config' / 'racecar' / 'calibration'
# Operator-facing output: the message only, without the ROS severity, time and node.
# rcutils reads this at rclpy.init(), which every tool calls after importing this.
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}')
# Set by calibrate.py for the tools it runs; it backs up and restarts teleop itself.
SESSION_ENV = 'RACECAR_CALIBRATE_SESSION'
FIRST_MESSAGE_TIMEOUT = 15.0
PROGRESS_INTERVAL = 5.0


def install_config_dir() -> Path | None:
    """Return the installed share config dir, or None when not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory(PACKAGE)) / 'config'
    except (ImportError, KeyError):
        return None
    return share if share.is_dir() else None


def source_config_dir(script: Path | None = None) -> Path | None:
    """
    Return the package's source config dir, or None.

    Checks the package root above the resolved script first (source checkout
    or symlink install), then <ws>/src/<package>/config derived from the
    install prefix.
    """
    for here in (script or Path(__file__)).resolve().parents:
        if (here / 'setup.py').is_file() and (here / 'config').is_dir():
            return here / 'config'
    share = install_config_dir()
    if share is not None and '/install/' in str(share):
        ws = Path(str(share).split('/install/')[0])
        candidate = ws / 'src' / PACKAGE / 'config'
        if candidate.is_dir():
            return candidate
    return None


def output_dirs() -> list[Path]:
    """Return every config dir a calibration result is written to."""
    dirs: list[Path] = []
    for d in (source_config_dir(), install_config_dir()):
        if d is not None and d.resolve() not in [x.resolve() for x in dirs]:
            dirs.append(d)
    return dirs


def write_local_yaml(
    name: str,
    node: str,
    params: dict[str, Any],
    dirs: list[Path],
    tool: str,
    merge: bool = False,
) -> list[Path]:
    """
    Write `<name>.local.yaml` holding `params` for `node` into each dir.

    With `merge`, parameters already in the file for `node` are kept unless
    `params` sets them, so a tool that calibrates in steps can write each step.
    """
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    header = (
        f'# Per-car calibration from {tool}, {stamp}. Not tracked in git;\n'
        f'# keep a copy in {BACKUP_DIR}/.\n'
    )
    written = []
    for d in dirs:
        path = d / f'{name}.local.yaml'
        merged = dict(params)
        if merge and path.exists():
            old = (yaml.safe_load(path.read_text()) or {}).get(node) or {}
            merged = {**(old.get('ros__parameters') or {}), **params}
        body = yaml.safe_dump({node: {'ros__parameters': merged}}, default_flow_style=None)
        path.write_text(header + body)
        written.append(path)
    return written


def update_local_values(
    name: str, values: dict[str, float], dirs: list[Path], tool: str
) -> list[Path]:
    """
    Set `values` in `<name>.local.yaml` in each dir, keeping its other keys.

    For plain key: value files such as frames.local.yaml, which several tools
    write into (not ROS parameter files).
    """
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    header = (
        f'# Per-car values; last written by {tool}, {stamp}. Not tracked in git;\n'
        f'# keep a copy in {BACKUP_DIR}/.\n'
    )
    written = []
    for d in dirs:
        path = d / f'{name}.local.yaml'
        current = yaml.safe_load(path.read_text()) if path.exists() else None
        merged = {**(current or {}), **values}
        path.write_text(header + yaml.safe_dump(merged, sort_keys=True))
        written.append(path)
    return written


def backup_reminder(paths: list[Path]) -> str:
    """
    Return the post-write message: where the file went and how to back it up.

    Under `racecar calibrate` (SESSION_ENV set) only the paths: the session
    backs the file up and offers the teleop restart itself.
    """
    lines = [f'Wrote {p}' for p in paths]
    if paths and not os.environ.get(SESSION_ENV):
        lines.append(f'Back it up: mkdir -p {BACKUP_DIR} && cp {paths[0]} {BACKUP_DIR}/')
        lines.append('Apply it: racecar service restart teleop (no rebuild needed).')
    return '\n'.join(lines)


def node_params(cal_name: str, node: str, config_dir: Path | None = None) -> dict[str, Any]:
    """
    Return `node`'s parameters from `<cal_name>.yaml` with `<cal_name>.local.yaml` on top.

    Reads the config the running stack loaded: the installed share directory,
    else the source tree.
    """
    config_dir = config_dir or install_config_dir() or source_config_dir()
    params: dict[str, Any] = {}
    if config_dir is None:
        return params
    for name in (f'{cal_name}.yaml', f'{cal_name}.local.yaml'):
        path = config_dir / name
        if path.exists():
            data = yaml.safe_load(path.read_text()) or {}
            params.update((data.get(node) or {}).get('ros__parameters') or {})
    return params


# ---------------------------------------------------------------------------
# Check results (--check), shared by every tool and read by calibrate.py
# ---------------------------------------------------------------------------

STATUS_ORDER = ('INFO', 'PASS', 'WARN', 'FAIL')


@dataclass(frozen=True)
class Criterion:
    """One line of a --check result. status: PASS, WARN, FAIL, or INFO (no verdict)."""

    name: str
    measured: str
    limit: str
    status: str
    advice: str = ''


def overall(criteria: list[Criterion]) -> str:
    """Return the worst status among the criteria (INFO lines do not count)."""
    judged = [c.status for c in criteria if c.status != 'INFO']
    return max(judged, key=STATUS_ORDER.index) if judged else 'PASS'


# Same colors and opt-outs as `racecar status` (scripts/diagnose.py).
ANSI = {'PASS': '32', 'WARN': '33', 'FAIL': '31'}


def use_color() -> bool:
    """Color only an interactive terminal; NO_COLOR (no-color.org) opts out."""
    return (
        sys.stdout.isatty()
        and 'NO_COLOR' not in os.environ
        and os.environ.get('TERM', '') != 'dumb'
    )


def paint(text: str, status: str, color: bool, bold: bool = False) -> str:
    code = ANSI.get(status)
    if not color or code is None:
        return text
    return f'\033[{"1;" if bold else ""}{code}m{text}\033[0m'


def criteria_lines(
    criteria: list[Criterion], indent: str = '  ', color: bool = False
) -> list[str]:
    """Render criteria as aligned table lines, advice on its own line below a WARN or FAIL."""
    lines = []
    for c in criteria:
        status = paint(f'{c.status:5s}', c.status, color)
        lines.append(f'{indent}{status} {c.name:18s} {c.measured:28s} {c.limit}'.rstrip())
        if c.advice and c.status in ('WARN', 'FAIL'):
            lines.append(f'{indent}      -> {c.advice}')
    return lines


def finish_check(log: Any, criteria: list[Criterion], report: str | None) -> int:
    """Log the criteria table, write the JSON report, and return the exit code (1 on FAIL)."""
    import json

    for line in criteria_lines(criteria):
        log.info(line)
    verdict = overall(criteria)
    log.info(f'Check: {verdict}')
    if report:
        Path(report).write_text(
            json.dumps({'overall': verdict, 'criteria': [c.__dict__ for c in criteria]})
        )
    return 1 if verdict == 'FAIL' else 0


def read_line(prompt: str = '') -> str:
    """
    input() for these tools: print the prompt, read one line a byte at a time.

    input() reads a piped stdin in blocks and would swallow the lines meant for
    the next prompt or the next tool, so a scripted run (one line per prompt)
    stops early. Raises EOFError at end of input, as input() does.

    On a terminal, keys pressed before the prompt appears are discarded, so an
    extra Enter during a recording cannot answer the next prompt.
    """
    if os.isatty(0):
        termios.tcflush(0, termios.TCIFLUSH)
    print(prompt, end='', flush=True)
    line = bytearray()
    while True:
        ch = os.read(0, 1)
        if not ch:
            if line:
                return line.decode(errors='replace')
            raise EOFError
        if ch == b'\n':
            return line.decode(errors='replace')
        line += ch


STEP_PROMPT = 'Press Enter to run this step, or s to skip it: '
STEP_PROMPT_BACK = 'Press Enter to run this step, s to skip it, or b to redo the previous step: '


def step_prompt(index: int) -> str:
    """Return the run prompt for step `index`; b is offered from the second step on."""
    return STEP_PROMPT_BACK if index > 0 else STEP_PROMPT


def run_sequence(
    count: int,
    show: Callable[[int], None],
    run: Callable[[int], Any],
    prompt: Callable[[int], str] = step_prompt,
    allow_skip: bool = True,
) -> dict[int, Any]:
    """
    Run steps 0..count-1 in order; return {index: result} for the steps that ran.

    At each step's prompt Enter runs it, s skips it (when allowed), and b goes
    back to the previous step and discards its result. A step whose run
    returns None (stopped, or a failed measurement) is offered again.
    """
    results: dict[int, Any] = {}
    i = 0
    while i < count:
        show(i)
        answer = read_line(prompt(i)).strip().lower()
        if answer == 'b':
            if i > 0:
                i -= 1
                results.pop(i, None)
            continue
        if answer == 's' and allow_skip:
            results.pop(i, None)
            i += 1
            continue
        result = run(i)
        if result is None:
            continue
        results[i] = result
        i += 1
    return results


def mean_vector(samples: list[list[float]]) -> list[float]:
    """Return the per-axis mean of a list of 3-vectors."""
    return [float(v) for v in np.asarray(samples, dtype=float).mean(axis=0)]


class CalibratorNode(Node):
    """
    Base node: one sensor subscription, a first-message gate, timed collection.

    Subclasses implement `extract(msg)` (one sample per message) and `run()`,
    which returns the process exit code. `run()` executes on a worker thread
    while `spin_until_done` spins the node on the main thread.
    """

    def __init__(self, node_name: str, msg_type: Any, topic: str) -> None:
        super().__init__(node_name)
        self.topic = topic
        self._first = threading.Event()
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[list[float]] = []
        self.create_subscription(msg_type, topic, self._callback, qos_profile_sensor_data)

    def extract(self, msg: Any) -> list[float]:
        raise NotImplementedError

    def run(self) -> int:
        raise NotImplementedError

    def _callback(self, msg: Any) -> None:
        self._first.set()
        if self._collecting:
            sample = self.extract(msg)
            with self._lock:
                self._samples.append(sample)

    def wait_for_first_message(self, timeout: float = FIRST_MESSAGE_TIMEOUT) -> bool:
        self.get_logger().info(f'Waiting for {self.topic}...')
        if self._first.wait(timeout):
            return True
        self.get_logger().error(f'No messages on {self.topic} after {timeout:.0f} seconds.')
        self.get_logger().error(f'Check: ros2 topic hz {self.topic}')
        return False

    def collect(self, duration: float, description: str) -> list[list[float]]:
        """Record samples for `duration` seconds and return them."""
        with self._lock:
            self._samples = []
        self.get_logger().info(f'{description}: collecting for {duration:.0f} seconds')
        self._collecting = True
        start = time.monotonic()
        next_report = start + PROGRESS_INTERVAL
        while (now := time.monotonic()) - start < duration:
            if now >= next_report:
                with self._lock:
                    n = len(self._samples)
                remaining = duration - (now - start)
                self.get_logger().info(f'{description}: {remaining:.0f} seconds left, {n} samples')
                next_report += PROGRESS_INTERVAL
            time.sleep(0.1)
        self._collecting = False
        with self._lock:
            samples = list(self._samples)
        self.get_logger().info(f'{description}: {len(samples)} samples')
        return samples


def spin_until_done(node: CalibratorNode) -> int:
    """Run `node.run()` on a worker thread, spin on this one, return its exit code."""
    result = [1]

    def work() -> None:
        try:
            result[0] = node.run()
        except EOFError:
            node.get_logger().error('Input closed; calibration aborted.')
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'Calibration failed: {exc}')

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    try:
        while worker.is_alive() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Calibration interrupted.')
        result[0] = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return result[0]


# ---------------------------------------------------------------------------
# Accelerometer bias and scale, gyroscope bias (calibrate_imu.py,
# calibrate_realsense_imu.py)
# ---------------------------------------------------------------------------

GRAVITY = 9.80665  # m/s^2, standard gravity
# A fitted scale outside this band means a position was held wrong, not a
# sensor gain error.
SCALE_LIMITS = (0.8, 1.25)
GYRO_SECONDS = 10.0
POSITION_SECONDS = 5.0
# One pose per axis pointing up, then down. /imu/lsm9ds1/raw is in REP-103 body
# axes (x forward, y left, z up).
BODY_POSITIONS = (
    ('X+', 'Stand the car on its rear end, nose pointing up.'),
    ('X-', 'Stand the car on its nose, rear pointing up.'),
    ('Y+', 'Lay the car on its right side, left side facing up.'),
    ('Y-', 'Lay the car on its left side, right side facing up.'),
    ('Z+', 'Set the car upright on its wheels.'),
    ('Z-', 'Turn the car upside down.'),
)
# /imu/realsense stays in the camera's optical axes (x right, y down, z forward).
OPTICAL_POSITIONS = (
    ('X+', 'Lay the car on its left side, right side facing up.'),
    ('X-', 'Lay the car on its right side, left side facing up.'),
    ('Y+', 'Turn the car upside down.'),
    ('Y-', 'Set the car upright on its wheels.'),
    ('Z+', 'Stand the car on its rear end, nose pointing up.'),
    ('Z-', 'Stand the car on its nose, rear pointing up.'),
)


@dataclass(frozen=True)
class ImuTarget:
    """Where an IMU bias calibration reads from and writes to."""

    label: str
    topic: str
    cal_name: str
    node: str
    accel_key: str
    gyro_key: str
    tool: str
    scale_key: str
    # --check reads check_topic. When check_applies_cal is true the stored
    # calibration is applied to it here, for a sensor whose node does not
    # publish a calibrated topic of its own (the RealSense).
    check_topic: str = ''
    check_applies_cal: bool = False
    positions: tuple[tuple[str, str], ...] = BODY_POSITIONS  # axes of `topic`


def imu_bias_params(
    target: ImuTarget, gyro: list[list[float]], positions: list[list[list[float]]]
) -> dict[str, list[float]] | None:
    """
    Return the node parameters for a fit, or None when data is missing.

    Gyro bias is the mean at rest. For each accelerometer axis, `up` and
    `down` are its mean with that axis pointing up and down (target.positions order):
    bias = (up + down) / 2 and scale = 2 g / (up - down), applied by the node
    as (raw - bias) * scale. Raises ValueError when an axis did not read about
    +g up and -g down, meaning the car was not in the position asked for.
    """
    if not gyro or len(positions) != len(target.positions) or not all(positions):
        return None
    means = [mean_vector(p) for p in positions]
    bias, scale = [], []
    for axis in range(3):
        up, down = means[2 * axis][axis], means[2 * axis + 1][axis]
        name = 'XYZ'[axis]
        if up - down <= 0:
            raise ValueError(
                f'{name} axis read {up:.2f} up and {down:.2f} down; check the positions'
            )
        gain = 2.0 * GRAVITY / (up - down)
        if not SCALE_LIMITS[0] <= gain <= SCALE_LIMITS[1]:
            raise ValueError(
                f'{name} axis read {up:.2f} up and {down:.2f} down m/s^2 (scale {gain:.3f}); '
                'check the positions'
            )
        bias.append((up + down) / 2.0)
        scale.append(gain)
    return {
        target.accel_key: bias,
        target.scale_key: scale,
        target.gyro_key: mean_vector(gyro),
    }


class ImuBiasCalibrator(CalibratorNode):
    """Interactive gyro-at-rest plus six-orientation accelerometer calibration."""

    def __init__(self, target: ImuTarget) -> None:
        super().__init__(f'{target.cal_name}_calibrator', Imu, target.topic)
        self.target = target

    def extract(self, msg: Any) -> list[float]:
        a, w = msg.linear_acceleration, msg.angular_velocity
        return [a.x, a.y, a.z, w.x, w.y, w.z]

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        log.info(f'{self.target.label} IMU calibration')

        # Step 0 is the gyroscope at rest; steps 1 to 6 are the poses.
        def show(i: int) -> None:
            if i == 0:
                log.info('Step 1 of 2: gyroscope. Set the car on a stable surface.')
                return
            if i == 1:
                log.info(
                    f'Step 2 of 2: accelerometer. Hold the car still in 6 poses, '
                    f'{POSITION_SECONDS:.0f} seconds each.'
                )
            axis, description = self.target.positions[i - 1]
            log.info(f'Pose {i} of {len(self.target.positions)} ({axis}): {description}')

        def prompt(i: int) -> str:
            if i == 0:
                return (
                    f'Press Enter, then keep your hands off the car for {GYRO_SECONDS:.0f} '
                    'seconds...'
                )
            return (
                'Press Enter when the car is in place, then hold it still. '
                'Type b to redo the previous step: '
            )

        def run_step(i: int) -> list[list[float]] | None:
            if i == 0:
                samples = [s[3:] for s in self.collect(GYRO_SECONDS, 'Gyroscope')]
            else:
                axis = self.target.positions[i - 1][0]
                samples = [s[:3] for s in self.collect(POSITION_SECONDS, axis)]
            return samples or None

        count = len(self.target.positions)
        steps = run_sequence(count + 1, show, run_step, prompt, allow_skip=False)
        gyro = steps[0]
        positions = [steps[k] for k in range(1, count + 1)]

        try:
            params = imu_bias_params(self.target, gyro, positions)
        except ValueError as exc:
            log.error(f'{exc}; nothing written.')
            return 1
        if params is None:
            log.error('Missing samples for at least one step; nothing written.')
            return 1
        dirs = output_dirs()
        if not dirs:
            log.error('No config directory found; nothing written.')
            return 1
        written = write_local_yaml(
            self.target.cal_name, self.target.node, params, dirs, self.target.tool
        )
        log.info(f'Gyroscope bias (rad/s): {params[self.target.gyro_key]}')
        log.info(f'Accelerometer bias (m/s^2): {params[self.target.accel_key]}')
        log.info(f'Accelerometer scale: {params[self.target.scale_key]}')
        log.info(backup_reminder(written))
        return 0


def run_imu_calibration(target: ImuTarget) -> int:
    rclpy.init()
    return spin_until_done(ImuBiasCalibrator(target))


# ---------------------------------------------------------------------------
# IMU check at rest (--check)
# ---------------------------------------------------------------------------

REST_SECONDS = 3.0
# --check limits: gravity within 1% at rest; gyro residual under 0.01 rad/s
# (0.57 deg/s).
GRAVITY_TOLERANCE = 0.01
GYRO_REST_LIMIT = 0.01


@dataclass(frozen=True)
class RestCheck:
    """Mean accel and gyro of a stationary IMU, after calibration."""

    accel: list[float]
    gyro: list[float]

    @property
    def gravity(self) -> float:
        return float(np.linalg.norm(self.accel))

    @property
    def gyro_rate(self) -> float:
        return float(np.linalg.norm(self.gyro))

    def criteria(self) -> list[Criterion]:
        error = self.gravity / GRAVITY - 1
        return [
            Criterion(
                'gravity',
                f'{self.gravity:.3f} m/s^2 ({error:+.2%})',
                f'within {GRAVITY_TOLERANCE:.0%} of {GRAVITY:.3f}',
                'PASS' if abs(error) <= GRAVITY_TOLERANCE else 'FAIL',
                'accelerometer bias or scale has moved; recalibrate this IMU',
            ),
            Criterion(
                'gyro at rest',
                f'{self.gyro_rate:.4f} rad/s',
                f'under {GYRO_REST_LIMIT} rad/s',
                'PASS' if self.gyro_rate <= GYRO_REST_LIMIT else 'FAIL',
                'gyro bias has moved, or the car was not still; recalibrate this IMU',
            ),
        ]

    @property
    def ok(self) -> bool:
        return overall(self.criteria()) != 'FAIL'


def apply_imu_cal(
    samples: list[list[float]], target: ImuTarget, params: dict[str, Any]
) -> list[list[float]]:
    """Apply stored bias and scale to raw [ax, ay, az, gx, gy, gz] samples, as the node does."""
    bias = np.asarray(params.get(target.accel_key, [0.0, 0.0, 0.0]), float)
    scale = np.asarray(params.get(target.scale_key, [1.0, 1.0, 1.0]), float)
    gyro_bias = np.asarray(params.get(target.gyro_key, [0.0, 0.0, 0.0]), float)
    data = np.asarray(samples, float)
    accel = (data[:, :3] - bias) * scale
    gyro = data[:, 3:] - gyro_bias
    return np.column_stack((accel, gyro)).tolist()


def rest_check(samples: list[list[float]]) -> RestCheck | None:
    if not samples:
        return None
    data = np.asarray(samples, float)
    return RestCheck(
        accel=mean_vector(data[:, :3].tolist()), gyro=mean_vector(data[:, 3:].tolist())
    )


class ImuRestChecker(CalibratorNode):
    """Check a stored IMU calibration against the live sensor, car at rest."""

    def __init__(self, target: ImuTarget, report: str | None = None) -> None:
        super().__init__(f'{target.cal_name}_checker', Imu, target.check_topic or target.topic)
        self.target = target
        self.report = report

    def extract(self, msg: Any) -> list[float]:
        a, w = msg.linear_acceleration, msg.angular_velocity
        return [a.x, a.y, a.z, w.x, w.y, w.z]

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        log.info(f'{self.target.label} IMU check. Set the car on a level surface.')
        read_line(
            f'Press Enter, then keep your hands off the car for {REST_SECONDS:.0f} seconds...'
        )
        samples = self.collect(REST_SECONDS, 'At rest')
        if self.target.check_applies_cal:
            params = node_params(self.target.cal_name, self.target.node)
            samples = apply_imu_cal(samples, self.target, params)
        result = rest_check(samples)
        if result is None:
            log.error('No samples collected.')
            return finish_check(
                log,
                [Criterion('samples', '0', 'at least 1', 'FAIL', 'is teleop running?')],
                self.report,
            )
        return finish_check(log, result.criteria(), self.report)


def run_imu_check(target: ImuTarget, report: str | None = None) -> int:
    rclpy.init()
    return spin_until_done(ImuRestChecker(target, report))


def imu_main(target: ImuTarget, argv: list[str] | None = None) -> int:
    """Entry point shared by calibrate_imu.py and calibrate_realsense_imu.py."""
    import argparse

    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description=f'{target.label} IMU calibration.')
    parser.add_argument(
        '--check', action='store_true', help='check the stored calibration at rest; exit 1 on FAIL'
    )
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(argv if argv is not None else remove_ros_args(sys.argv)[1:])
    return run_imu_check(target, args.report) if args.check else run_imu_calibration(target)
