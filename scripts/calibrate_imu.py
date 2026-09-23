#!/usr/bin/env python3
"""
LSM9DS1 accelerometer and gyroscope bias calibration for pit_node.

Reads /imu/lsm9ds1/raw and writes config/lsm9ds1_cal.local.yaml.

Author: Koneshka Dey
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import ImuTarget, run_imu_calibration  # noqa: E402

TARGET = ImuTarget(
    label='LSM9DS1',
    topic='/imu/lsm9ds1/raw',
    cal_name='lsm9ds1_cal',
    node='pit_node',
    accel_key='accelerometer.bias',
    gyro_key='gyroscope.bias',
    tool='calibrate_imu.py',
)


def main() -> int:
    return run_imu_calibration(TARGET)


if __name__ == '__main__':
    sys.exit(main())
