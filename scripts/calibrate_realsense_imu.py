#!/usr/bin/env python3
"""
RealSense D435i IMU bias calibration for imu_fusion_node.

Reads /imu/realsense and writes config/realsense_cal.local.yaml.
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import ImuTarget, run_imu_calibration  # noqa: E402

TARGET = ImuTarget(
    label='RealSense',
    topic='/imu/realsense',
    cal_name='realsense_cal',
    node='imu_fusion_node',
    accel_key='realsense_accel_bias',
    gyro_key='realsense_gyro_bias',
    tool='calibrate_realsense_imu.py',
)


def main() -> int:
    return run_imu_calibration(TARGET)


if __name__ == '__main__':
    sys.exit(main())
