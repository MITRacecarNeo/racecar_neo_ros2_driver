#!/usr/bin/env python3
"""
LSM9DS1 magnetometer hard- and soft-iron calibration for pit_node.

Reads /mag/raw while the car is rotated about each axis, fits an ellipsoid,
and writes config/lsm9ds1_mag_cal.local.yaml.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import numpy as np
import rclpy
from sensor_msgs.msg import MagneticField

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    output_dirs,
    spin_until_done,
    write_local_yaml,
)

CAL_NAME = 'lsm9ds1_mag_cal'
NODE = 'pit_node'
TOOL = 'calibrate_mag.py'
MIN_SAMPLES = 100
STEP_SECONDS = 15.0
STEPS = [
    ('Yaw', 'turn the car flat, like driving in a circle'),
    ('Roll', 'barrel-roll the car about its long axis'),
    ('Pitch', 'tip the nose up and down'),
]


@dataclass
class MagFit:
    """Hard-iron offset (T) and soft-iron matrix from an ellipsoid fit."""

    hard_iron: np.ndarray
    soft_iron: np.ndarray


def fit_ellipsoid(samples: list[list[float]]) -> MagFit | None:
    """
    Fit an ellipsoid to raw field samples, or return None when it cannot.

    None means fewer than MIN_SAMPLES or a fit that is not an ellipsoid (a
    quadric matrix that is not positive definite, or non-finite values).
    """
    if len(samples) < MIN_SAMPLES:
        return None
    data = np.asarray(samples, dtype=float)
    scale = float(np.mean(np.linalg.norm(data, axis=1)))
    if not np.isfinite(scale) or scale <= 0.0:
        return None
    x, y, z = (data / scale).T
    # Quadric 2*b.v + v^T A v = 1, solved by least squares on normalized data.
    design = np.column_stack(
        [2 * x, 2 * y, 2 * z, x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z]
    )
    p, *_ = np.linalg.lstsq(design, np.ones(len(data)), rcond=None)
    a = np.array([[p[3], p[6], p[7]], [p[6], p[4], p[8]], [p[7], p[8], p[5]]])
    evals, evecs = np.linalg.eigh(a)
    if not np.all(np.isfinite(evals)) or np.any(evals <= 0.0):
        return None
    hard_iron = -np.linalg.solve(a, p[:3]) * scale
    soft_iron = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    if not (np.all(np.isfinite(hard_iron)) and np.all(np.isfinite(soft_iron))):
        return None
    return MagFit(hard_iron=hard_iron, soft_iron=soft_iron)


def mag_params(fit: MagFit) -> dict[str, list[float]]:
    return {
        'magnetometer.hard_iron_bias': [float(v) for v in fit.hard_iron],
        'magnetometer.soft_iron_matrix.data': [float(v) for v in fit.soft_iron.flatten()],
    }


def fit_and_write(
    samples: list[list[float]], dirs: list[Path]
) -> tuple[MagFit, list[Path]] | None:
    """Fit and write the local YAML; write nothing and return None if the fit fails."""
    fit = fit_ellipsoid(samples)
    if fit is None:
        return None
    return fit, write_local_yaml(CAL_NAME, NODE, mag_params(fit), dirs, TOOL)


def plot_results(samples: list[list[float]], fit: MagFit) -> None:
    import matplotlib.pyplot as plt

    raw = np.asarray(samples, dtype=float)
    corrected = (fit.soft_iron @ (raw - fit.hard_iron).T).T
    for data, color, title in (
        (raw, 'r', 'Uncorrected magnetometer data (ellipsoid)'),
        (corrected, 'b', 'Corrected magnetometer data (sphere)'),
    ):
        ax = plt.figure(figsize=(10, 8)).add_subplot(111, projection='3d')
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], c=color, marker='.')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)
    print('Close the plot windows to exit.')
    plt.show()


class MagnetometerCalibrator(CalibratorNode):
    def __init__(self) -> None:
        super().__init__('magnetometer_calibrator', MagneticField, '/mag/raw')
        self.samples: list[list[float]] = []
        self.fit: MagFit | None = None

    def extract(self, msg: Any) -> list[float]:
        f = msg.magnetic_field
        return [f.x, f.y, f.z]

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            log.error('Is pit_node running? Check: ros2 topic echo /mag/raw')
            return 1
        log.info('LSM9DS1 magnetometer calibration')
        for i, (axis, how) in enumerate(STEPS, start=1):
            log.info(f'Step {i}/{len(STEPS)}: {axis} axis; {how}.')
            input('Press Enter, then start rotating...')
            self.samples += self.collect(STEP_SECONDS, f'{axis} axis')

        dirs = output_dirs()
        if not dirs:
            log.error('No config directory found; nothing written.')
            return 1
        result = fit_and_write(self.samples, dirs)
        if result is None:
            log.error(
                f'Fit failed with {len(self.samples)} samples (need {MIN_SAMPLES}, '
                'covering all three axes); nothing written. Rotate more and retry.'
            )
            return 1
        self.fit, written = result
        log.info(f'Hard-iron bias (T): {self.fit.hard_iron.tolist()}')
        log.info(backup_reminder(written))
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--no-plot', action='store_true', help='skip the 3D before/after plots')
    args = ap.parse_args()

    rclpy.init()
    node = MagnetometerCalibrator()
    code = spin_until_done(node)
    if code == 0 and node.fit is not None and not args.no_plot:
        plot_results(node.samples, node.fit)
    return code


if __name__ == '__main__':
    sys.exit(main())
