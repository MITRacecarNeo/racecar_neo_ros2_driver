#!/usr/bin/env python3
"""
Odometry scale calibration for pit_node: gyro yaw scale and encoder speed scale.

Spin: the car is parked with its left side against a strip of tape, rests 3 s
(the gyro's offset at rest), is turned 3 full turns (driven in circles or
rotated by hand; the gyro counts rotation whatever the path), and is parked
against the tape again. The yaw rate
the odometry uses (gyro axis and sign from config/odom.yaml, bias removed) is
integrated over the turn; the turn count is rounded from it, and
odometry.yaw_rate_scale = turns x 360 deg / measured. A scale far from 1 is
refused: it points at imu.gyro_scale (a deg/s versus rad/s mix-up is 57x).

Tape: the car is driven over a taped distance and stopped;
odometry.speed_scale = current scale x tape / measured, from
/odom/integrated.

Writes config/odom.local.yaml (pit_node), keeping the other step's value.
--check repeats a 2-turn spin on /odom/integrated, and with --tape-m a tape
run, and judges them against the scales now in use.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    node_params,
    output_dirs,
    read_line,
    spin_until_done,
    write_local_yaml,
)

CAL_NAME = 'odom'
NODE = 'pit_node'
TOOL = 'calibrate_odom.py'
REST_SECONDS = 3.0
SCALE_LIMITS = (0.8, 1.25)
# A turn count is accepted when the measurement is within this of a whole turn.
WHOLE_TURN_SLACK = 0.15
CHECK_TURNS = 2
SPIN_PASS, SPIN_WARN = 0.01, 0.03
TAPE_PASS, TAPE_WARN = 0.02, 0.05


@dataclass(frozen=True)
class SpinFit:
    turns: int
    measured_deg: float
    scale: float  # multiplies the current yaw rate


def integrate_rate(samples: list[tuple[float, float]], bias: float = 0.0) -> float:
    """Integrate (time s, rate rad/s) samples, minus `bias`, by the trapezoid rule."""
    if len(samples) < 2:
        return 0.0
    data = np.asarray(samples, float)
    rate = data[:, 1] - bias
    return float(np.sum(0.5 * (rate[1:] + rate[:-1]) * np.diff(data[:, 0])))


def fit_spin(measured_rad: float) -> SpinFit | None:
    """Round a hand spin to whole turns; None unless close to a whole turn."""
    turns_float = abs(measured_rad) / (2 * math.pi)
    turns = round(turns_float)
    if turns < 1 or abs(turns_float - turns) > WHOLE_TURN_SLACK:
        return None
    return SpinFit(turns, math.degrees(measured_rad), turns * 2 * math.pi / abs(measured_rad))


def scale_ok(scale: float) -> bool:
    return SCALE_LIMITS[0] <= scale <= SCALE_LIMITS[1]


def graded(error: float, passing: float, warning: float) -> str:
    return 'PASS' if abs(error) <= passing else 'WARN' if abs(error) <= warning else 'FAIL'


def spin_criterion(measured_deg: float, turns: int) -> Criterion:
    error = abs(measured_deg) / (turns * 360.0) - 1.0
    return Criterion(
        'spin heading',
        f'{measured_deg:+.1f} deg ({error:+.2%})',
        f'{turns * 360} within {SPIN_PASS:.0%} (warn to {SPIN_WARN:.0%})',
        graded(error, SPIN_PASS, SPIN_WARN),
        'recalibrate the yaw scale: racecar calibrate odom',
    )


def tape_criterion(measured_m: float, tape_m: float) -> Criterion:
    error = measured_m / tape_m - 1.0
    return Criterion(
        'tape distance',
        f'{measured_m:.3f} m ({error:+.2%})',
        f'{tape_m:g} m within {TAPE_PASS:.0%} (warn to {TAPE_WARN:.0%})',
        graded(error, TAPE_PASS, TAPE_WARN),
        'recalibrate the speed scale: racecar calibrate odom',
    )


class OdomCalibrator(CalibratorNode):
    """Gyro yaw rate (raw, for the spin) and /odom/integrated (for tape and checks)."""

    def __init__(self, check: bool, tape_m: float | None, report: str | None) -> None:
        super().__init__('odom_calibrator', Odometry, '/odom/integrated')
        self.check = check
        self.tape_m = tape_m
        self.report = report
        params = node_params(CAL_NAME, NODE)
        self.axis = int(params.get('odometry.yaw_rate_axis', 1))
        self.sign = float(params.get('odometry.yaw_rate_sign', -1.0))
        self.yaw_scale = float(params.get('odometry.yaw_rate_scale', 1.0))
        self.speed_scale = float(params.get('odometry.speed_scale', 1.0))
        self._gyro: list[tuple[float, float]] = []
        self._gyro_on = False
        self._gyro_lock = threading.Lock()
        self.create_subscription(Imu, '/imu/lsm9ds1', self._gyro_cb, qos_profile_sensor_data)

    def extract(self, msg: Any) -> list[float]:
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return [
            stamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * math.atan2(q.z, q.w),
        ]

    def _gyro_cb(self, msg: Any) -> None:
        if self._gyro_on:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rate = self.sign * float(getattr(msg.angular_velocity, 'xyz'[self.axis]))
            with self._gyro_lock:
                self._gyro.append((stamp, rate))

    def _gyro_window(self, prompt: str) -> list[tuple[float, float]]:
        with self._gyro_lock:
            self._gyro = []
        self._gyro_on = True
        read_line(prompt)
        self._gyro_on = False
        with self._gyro_lock:
            return list(self._gyro)

    def _pose_window(self, prompt: str) -> list[list[float]]:
        """Record /odom/integrated poses until Enter."""
        with self._lock:
            self._samples = []
        self._collecting = True
        read_line(prompt)
        self._collecting = False
        with self._lock:
            return list(self._samples)

    def _rest_bias(self) -> float:
        self._gyro_on = True
        with self._gyro_lock:
            self._gyro = []
        time.sleep(REST_SECONDS)
        self._gyro_on = False
        with self._gyro_lock:
            rates = [r for _, r in self._gyro]
        return float(np.mean(rates)) if rates else 0.0

    def spin(self) -> SpinFit | None:
        log = self.get_logger()
        log.info('Lay a straight strip of tape on the floor.')
        log.info('Park the car with its left side touching the tape.')
        read_line('Press Enter, then keep your hands off the car for 3 s...')
        bias = self._rest_bias()
        log.info('Turn the car 3 full turns, either direction: drive circles with the')
        log.info('controller, or rotate it by hand on the spot.')
        log.info('Then park it with its left side touching the tape again; push it by hand')
        log.info('to line it up if needed.')
        samples = self._gyro_window('Press Enter when it is back against the tape...')
        measured = integrate_rate(samples, bias) * self.yaw_scale
        fit = fit_spin(measured)
        if fit is None:
            log.error(
                f'Measured {math.degrees(measured):+.1f} deg, not a whole number of turns. '
                'Park the car flush against the tape and try again.'
            )
        return fit

    def tape(self) -> float | None:
        log = self.get_logger()
        log.info('Park the car with its front bumper on the first mark, wheels straight.')
        read_line('Press Enter, then drive straight to the second mark...')
        poses = self._pose_window(
            'Stop with the front bumper on the second mark, then press Enter...'
        )
        if len(poses) < 2:
            log.error('No odometry received. Check that teleop is running.')
            return None
        return math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2])

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        if self.check:
            return self._run_check()
        params: dict[str, float] = {}

        log.info('Step 1 of 2: gyro scale. You turn the car 3 full turns on the floor.')
        if read_line('Enter to run, s to skip: ').strip().lower() != 's':
            fit = self.spin()
            if fit is None:
                return 1
            new = self.yaw_scale * fit.scale
            log.info(f'{fit.turns} turns measured as {fit.measured_deg:+.1f} deg')
            if not scale_ok(fit.scale):
                log.error(
                    f'Gyro scale {fit.scale:.3f} is outside 0.8 to 1.25, so imu.gyro_scale in '
                    'config/pit.yaml is likely wrong (deg/s for rad/s). Nothing written.'
                )
                return 1
            params['odometry.yaw_rate_scale'] = round(new, 5)

        log.info('Step 2 of 2: speed scale. You drive the car straight between two floor')
        log.info('marks about 3 m apart.')
        if read_line('Enter to run, s to skip: ').strip().lower() != 's':
            tape_m = self.tape_m or float(read_line('Distance between the marks, in metres: '))
            measured = self.tape()
            if measured is None or measured <= 0:
                return 1
            ratio = tape_m / measured
            if not scale_ok(ratio):
                log.error(
                    f'Measured {measured:.3f} m for {tape_m:g} m, off by more than 20%. Check the '
                    'marks and the distance typed. Nothing written.'
                )
                return 1
            log.info(f'{tape_m:g} m measured as {measured:.3f} m')
            params['odometry.speed_scale'] = round(self.speed_scale * ratio, 5)

        if not params:
            log.info('Nothing calibrated.')
            return 0
        dirs = output_dirs()
        if not dirs:
            log.error('No config directory found; nothing written.')
            return 1
        written = write_local_yaml(CAL_NAME, NODE, params, dirs, TOOL, merge=True)
        for key, value in params.items():
            log.info(f'{key}: {value}')
        log.info(backup_reminder(written))
        return 0

    def _run_check(self) -> int:
        log = self.get_logger()
        criteria = []
        log.info('Park the car with its left side touching the strip of tape.')
        read_line('Press Enter to start...')
        log.info(f'Turn the car {CHECK_TURNS} full turns (drive circles or rotate it by hand),')
        log.info('then park it with its left side touching the tape again.')
        poses = self._pose_window('Press Enter when it is back against the tape...')
        if len(poses) >= 2:
            heading = math.degrees(float(np.unwrap([p[3] for p in poses])[-1] - poses[0][3]))
            criteria.append(spin_criterion(heading, CHECK_TURNS))
        else:
            criteria.append(
                Criterion('spin heading', 'no data', '', 'FAIL', 'is pit_node running?')
            )
        if self.tape_m:
            measured = self.tape()
            if measured is not None:
                criteria.append(tape_criterion(measured, self.tape_m))
        criteria.append(
            Criterion(
                'scales in use',
                f'yaw {self.yaw_scale:g}, speed {self.speed_scale:g}',
                '',
                'INFO',
            )
        )
        return finish_check(log, criteria, self.report)


def main() -> int:
    parser = argparse.ArgumentParser(description='Odometry scale calibration (spin and tape).')
    parser.add_argument('--tape-m', type=float, help='taped distance, m (asked if not given)')
    parser.add_argument('--check', action='store_true', help='check the scales in use')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    return spin_until_done(OdomCalibrator(args.check, args.tape_m, args.report))


if __name__ == '__main__':
    sys.exit(main())
