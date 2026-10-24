#!/usr/bin/env python3
"""
LSM9DS1 magnetometer hard- and soft-iron calibration for pit_node.

Reads /mag/raw while the car is rotated about each axis, fits an ellipsoid,
and writes the car profile's params/lsm9ds1_mag_cal.yaml.

--check verifies the stored calibration: with the car turned one full circle
flat on the floor, pit_node's corrected /mag should trace a round circle
centered on the turning axis, at Earth-field strength. /mag/raw is fitted the
same way for comparison.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import MagneticField

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    read_line,
    run_sequence,
    spin_until_done,
    write_local_yaml,
)

CAL_NAME = 'lsm9ds1_mag_cal'
NODE = 'pit_node'
TOOL = 'calibrate_mag.py'
MIN_SAMPLES = 100
STEP_SECONDS = 15.0
STEPS = [
    (
        'Yaw',
        'Hold the car with its wheels down and rotate it horizontally, parallel to the '
        'floor. Turn it a full turn or more for best results.',
    ),
    (
        'Roll',
        'Roll the car over sideways (barrel roll). Rotate the car a full turn or more '
        'for best results.',
    ),
    (
        'Pitch',
        'Rotate the car front to back (pitch down, pitch up). Rotate the car a full turn '
        'or more for best results.',
    ),
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
    # The quadric below fixes its constant term at 1, which only describes an
    # ellipsoid around the origin. Centering on the samples' mean (always
    # inside the ellipsoid they lie on) keeps it so when the hard iron exceeds
    # the field; the mean is added back to the offset after the fit.
    centroid = data.mean(axis=0)
    data = data - centroid
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
    hard_iron = centroid - np.linalg.solve(a, p[:3]) * scale
    soft_iron = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    if not (np.all(np.isfinite(hard_iron)) and np.all(np.isfinite(soft_iron))):
        return None
    return MagFit(hard_iron=hard_iron, soft_iron=soft_iron)


# --check: one flat turn. Limits: the circle covers CHECK_MIN_COVERAGE_DEG of
# the turn; its center lies within CHECK_MAX_OFFSET of its radius from the
# turning axis (a hard-iron error moves it off); its radius varies by under
# CHECK_MAX_ROUNDNESS (a soft-iron error squashes it); and the field is
# Earth-like (about 25 to 65 uT; the band allows for indoor steel).
CHECK_SECONDS = 25.0
CHECK_MIN_COVERAGE_DEG = 300.0
CHECK_MAX_OFFSET = 0.10
CHECK_MAX_ROUNDNESS = 0.05
CHECK_FIELD_UT = (20.0, 70.0)


@dataclass(frozen=True)
class CircleCheck:
    """Fit of a flat turn's field vectors to a circle about the turning axis."""

    coverage_deg: float
    offset: float  # circle center's distance from the turning axis / radius
    roundness: float  # RMS radial residual / radius
    field_ut: float  # mean |B|

    def criteria(self, raw: CircleCheck | None = None) -> list[Criterion]:
        lo, hi = CHECK_FIELD_UT
        out = [
            Criterion(
                'turn coverage',
                f'{self.coverage_deg:.0f} deg',
                f'at least {CHECK_MIN_COVERAGE_DEG:.0f} deg',
                'PASS' if self.coverage_deg >= CHECK_MIN_COVERAGE_DEG else 'FAIL',
                'turn the car a full circle during the check',
            ),
            Criterion(
                'center offset',
                f'{self.offset:.1%} of radius',
                f'under {CHECK_MAX_OFFSET:.0%}',
                'PASS' if self.offset <= CHECK_MAX_OFFSET else 'FAIL',
                'hard-iron offset has moved (battery or parts moved?); racecar calibrate mag',
            ),
            Criterion(
                'roundness',
                f'{self.roundness:.1%} of radius',
                f'under {CHECK_MAX_ROUNDNESS:.0%}',
                'PASS' if self.roundness <= CHECK_MAX_ROUNDNESS else 'FAIL',
                'soft-iron distortion has changed; racecar calibrate mag',
            ),
            # A uniform scale error leaves the field's direction, and so a
            # compass heading, unchanged: a warning, not a failure.
            Criterion(
                'field strength',
                f'{self.field_ut:.1f} uT',
                f'{lo:.0f} to {hi:.0f} uT (Earth about 50)',
                'PASS' if lo <= self.field_ut <= hi else 'WARN',
                'steel nearby, or a magnetometer gain mismatch (imu.mag_scale is '
                'unverified; docs/troubleshooting.md); heading is unaffected',
            ),
        ]
        if raw is not None:
            out.append(
                Criterion(
                    'raw, for contrast',
                    f'{raw.offset:.1%} off center, {raw.roundness:.1%} out of round, '
                    f'{raw.field_ut:.1f} uT',
                    '',
                    'INFO',
                )
            )
        return out


def check_circle(samples: list[list[float]]) -> CircleCheck | None:
    """
    Fit field vectors from a flat turn to a circle; None with too few samples.

    The turning axis is the direction of least variance. In a correct
    calibration the field's component across that axis keeps a constant
    length, so the circle is centered where the axis passes through it.
    """
    if len(samples) < MIN_SAMPLES:
        return None
    data = np.asarray(samples, dtype=float)
    centroid = data.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov((data - centroid).T))
    normal, u, v = vecs[:, 0], vecs[:, 2], vecs[:, 1]
    p = np.column_stack(((data - centroid) @ u, (data - centroid) @ v))
    # Algebraic circle fit: x^2 + y^2 + D x + E y + F = 0.
    design = np.column_stack((p[:, 0], p[:, 1], np.ones(len(p))))
    (d, e, f), *_ = np.linalg.lstsq(design, -(p**2).sum(axis=1), rcond=None)
    cx, cy = -d / 2.0, -e / 2.0
    radius_sq = cx * cx + cy * cy - f
    if not np.isfinite(radius_sq) or radius_sq <= 0.0:
        return None
    radius = float(np.sqrt(radius_sq))
    center = centroid + cx * u + cy * v
    off_axis = center - (center @ normal) * normal
    radial = np.hypot(p[:, 0] - cx, p[:, 1] - cy) - radius
    angles = np.sort(np.degrees(np.arctan2(p[:, 1] - cy, p[:, 0] - cx)))
    gaps = np.diff(np.append(angles, angles[0] + 360.0))
    return CircleCheck(
        coverage_deg=float(360.0 - gaps.max()),
        offset=float(np.linalg.norm(off_axis)) / radius,
        roundness=float(np.sqrt(np.mean(radial**2))) / radius,
        field_ut=float(np.mean(np.linalg.norm(data, axis=1))) * 1e6,
    )


def mag_params(fit: MagFit) -> dict[str, list[float]]:
    return {
        'magnetometer.hard_iron_bias': [float(v) for v in fit.hard_iron],
        'magnetometer.soft_iron_matrix.data': [float(v) for v in fit.soft_iron.flatten()],
    }


def fit_and_write(samples: list[list[float]]) -> tuple[MagFit, list[Path]] | None:
    """Fit and write the profile override; write nothing and return None if the fit fails."""
    fit = fit_ellipsoid(samples)
    if fit is None:
        return None
    return fit, write_local_yaml(CAL_NAME, NODE, mag_params(fit), TOOL)


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

        def show(i: int) -> None:
            axis, how = STEPS[i]
            log.info(
                f'Step {i + 1} of {len(STEPS)} ({axis}): {how} Keep moving for '
                f'{STEP_SECONDS:.0f} seconds.'
            )

        def prompt(i: int) -> str:
            if i == 0:
                return 'Press Enter, then start...'
            return 'Press Enter, then start. Type b to redo the previous step: '

        steps = run_sequence(
            len(STEPS),
            show,
            lambda i: self.collect(STEP_SECONDS, f'{STEPS[i][0]} axis') or None,
            prompt,
            allow_skip=False,
        )
        self.samples = [sample for i in range(len(STEPS)) for sample in steps[i]]

        result = fit_and_write(self.samples)
        if result is None:
            log.error(
                f'Fit failed with {len(self.samples)} samples (need {MIN_SAMPLES}); nothing '
                'written. The samples do not trace an ellipsoid: rotate through all three '
                'steps, with real roll and pitch, and retry.'
            )
            return 1
        self.fit, written = result
        log.info(f'Hard-iron bias (T): {self.fit.hard_iron.tolist()}')
        log.info(backup_reminder(written))
        return 0


class MagnetometerChecker(CalibratorNode):
    """Record /mag and /mag/raw through one flat turn and fit both."""

    def __init__(self, report: str | None = None) -> None:
        super().__init__('magnetometer_checker', MagneticField, '/mag')
        self.report = report
        self._raw: list[list[float]] = []
        self._raw_on = False
        self.create_subscription(MagneticField, '/mag/raw', self._raw_cb, qos_profile_sensor_data)

    def extract(self, msg: Any) -> list[float]:
        f = msg.magnetic_field
        return [f.x, f.y, f.z]

    def _raw_cb(self, msg: Any) -> None:
        if self._raw_on:
            self._raw.append(self.extract(msg))

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            log.error('Is pit_node running? Check: ros2 topic echo /mag')
            return 1
        log.info(
            'Magnetometer check. Place the car down on the floor and away from large metal '
            'structures.'
        )
        log.info(
            'Rotate (yaw) the car on the floor slowly and steadily for one or more full '
            f'turns, over {CHECK_SECONDS:.0f} seconds.'
        )
        read_line('Press Enter, then start turning...')
        self._raw_on = True
        samples = self.collect(CHECK_SECONDS, 'Turning')
        self._raw_on = False
        result = check_circle(samples)
        if result is None:
            log.error(f'Too few samples ({len(samples)}) for a fit.')
            too_few = Criterion(
                'samples',
                str(len(samples)),
                f'at least {MIN_SAMPLES}',
                'FAIL',
                'is teleop running?',
            )
            return finish_check(log, [too_few], self.report)
        return finish_check(log, result.criteria(check_circle(self._raw)), self.report)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--no-plot', action='store_true', help='skip the 3D before/after plots')
    ap.add_argument(
        '--check', action='store_true', help='check the stored calibration; exit 1 on FAIL'
    )
    ap.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    if args.check:
        return spin_until_done(MagnetometerChecker(args.report))
    node = MagnetometerCalibrator()
    code = spin_until_done(node)
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    if code == 0 and node.fit is not None and not args.no_plot and has_display:
        plot_results(node.samples, node.fit)
    return code


if __name__ == '__main__':
    sys.exit(main())
