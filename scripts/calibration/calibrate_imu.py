#!/usr/bin/env python3
"""
LSM9DS1 accelerometer bias and scale, gyroscope bias calibration for pit_node.

Reads /imu/lsm9ds1/raw and writes the car profile's params/lsm9ds1_cal.yaml. --check
reads pit_node's calibrated /imu/lsm9ds1 at rest.

Author: Koneshka Dey
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import imu_main, ImuTarget  # noqa: E402, I100

TARGET = ImuTarget(
    label='LSM9DS1',
    topic='/imu/lsm9ds1/raw',
    cal_name='lsm9ds1_cal',
    node='pit_node',
    accel_key='accelerometer.bias',
    gyro_key='gyroscope.bias',
    tool='calibrate_imu.py',
    scale_key='accelerometer.scale',
    check_topic='/imu/lsm9ds1',
)


def main() -> int:
    return imu_main(TARGET)


if __name__ == '__main__':
    sys.exit(main())
