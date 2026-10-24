#!/usr/bin/env python3
"""
RealSense D435i IMU calibration for imu_fusion_node: accel bias and scale, gyro bias.

Reads /imu/realsense and writes the car profile's params/realsense_cal.yaml. --check
applies the stored calibration to the live sensor at rest.
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import imu_main, ImuTarget, OPTICAL_POSITIONS  # noqa: E402, I100

TARGET = ImuTarget(
    label='RealSense',
    topic='/imu/realsense',
    cal_name='realsense_cal',
    node='imu_fusion_node',
    accel_key='realsense_accel_bias',
    gyro_key='realsense_gyro_bias',
    tool='calibrate_realsense_imu.py',
    scale_key='realsense_accel_scale',
    check_topic='/imu/realsense',
    check_applies_cal=True,
    positions=OPTICAL_POSITIONS,
)


def main() -> int:
    return imu_main(TARGET)


if __name__ == '__main__':
    sys.exit(main())
