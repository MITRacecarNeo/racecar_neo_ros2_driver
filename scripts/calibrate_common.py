#!/usr/bin/env python3
"""
Shared plumbing for the calibrate_*.py tools.

Results go to config/<name>.local.yaml (gitignored) in the source tree and in
the installed share config dir when it exists. The launch files load them
after the tracked zero-default YAMLs, so the local values win.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
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
FIRST_MESSAGE_TIMEOUT = 15.0
PROGRESS_INTERVAL = 2.0


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

    Checks next to the resolved script first (source checkout or symlink
    install), then <ws>/src/<package>/config derived from the install prefix.
    """
    here = (script or Path(__file__)).resolve().parent.parent
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
    name: str, node: str, params: dict[str, Any], dirs: list[Path], tool: str
) -> list[Path]:
    """Write `<name>.local.yaml` holding `params` for `node` into each dir."""
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    header = (
        f'# Per-car calibration from {tool}, {stamp}. Not tracked in git;\n'
        f'# keep a copy in {BACKUP_DIR}/.\n'
    )
    body = yaml.safe_dump({node: {'ros__parameters': params}}, default_flow_style=None)
    written = []
    for d in dirs:
        path = d / f'{name}.local.yaml'
        path.write_text(header + body)
        written.append(path)
    return written


def backup_reminder(paths: list[Path]) -> str:
    """Return the post-write message: where the file went and how to back it up."""
    lines = [f'Wrote {p}' for p in paths]
    if paths:
        lines.append(f'Back it up: mkdir -p {BACKUP_DIR} && cp {paths[0]} {BACKUP_DIR}/')
        lines.append('Apply it: racecar build, then restart teleop.')
    return '\n'.join(lines)


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
        self.get_logger().error(f'No messages on {self.topic} after {timeout:.0f}s.')
        self.get_logger().error(f'Check: ros2 topic hz {self.topic}')
        return False

    def collect(self, duration: float, description: str) -> list[list[float]]:
        """Record samples for `duration` seconds and return them."""
        with self._lock:
            self._samples = []
        self.get_logger().info(f'{description}: collecting for {duration:.0f}s')
        self._collecting = True
        start = time.monotonic()
        next_report = start + PROGRESS_INTERVAL
        while (now := time.monotonic()) - start < duration:
            if now >= next_report:
                with self._lock:
                    n = len(self._samples)
                remaining = duration - (now - start)
                self.get_logger().info(f'{description}: {remaining:.0f}s left, {n} samples')
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
# Accelerometer and gyroscope bias (calibrate_imu.py, calibrate_realsense_imu.py)
# ---------------------------------------------------------------------------

GYRO_SECONDS = 10.0
POSITION_SECONDS = 5.0
POSITIONS = [
    ('X+', 'X axis pointing up'),
    ('X-', 'X axis pointing down'),
    ('Y+', 'Y axis pointing up'),
    ('Y-', 'Y axis pointing down'),
    ('Z+', 'Z axis pointing up'),
    ('Z-', 'Z axis pointing down'),
]


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


def imu_bias_params(
    target: ImuTarget, gyro: list[list[float]], positions: list[list[list[float]]]
) -> dict[str, list[float]] | None:
    """
    Return the node parameters for a bias fit, or None when data is missing.

    Gyro bias is the mean at rest. Accel bias is the mean of the six
    orientation means, where gravity cancels.
    """
    if not gyro or len(positions) != len(POSITIONS) or not all(positions):
        return None
    accel = mean_vector([mean_vector(p) for p in positions])
    return {target.accel_key: accel, target.gyro_key: mean_vector(gyro)}


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

        log.info('1. Gyroscope: place the car on a stable surface and do not move it.')
        input('Press Enter to start...')
        gyro = [s[3:] for s in self.collect(GYRO_SECONDS, 'Gyroscope')]

        log.info('2. Accelerometer: six orientations.')
        positions = []
        for i, (axis, description) in enumerate(POSITIONS, start=1):
            log.info(f'Position {i}/{len(POSITIONS)}: {axis}, {description}')
            input('Press Enter when in position...')
            positions.append([s[:3] for s in self.collect(POSITION_SECONDS, axis)])

        params = imu_bias_params(self.target, gyro, positions)
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
        log.info(backup_reminder(written))
        return 0


def run_imu_calibration(target: ImuTarget) -> int:
    rclpy.init()
    return spin_until_done(ImuBiasCalibrator(target))
