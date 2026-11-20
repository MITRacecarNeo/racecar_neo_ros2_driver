# RACECAR Neo handbook

This handbook sets up a new RACECAR Neo car and keeps it healthy. Follow the
pages in order on a new car. Each step says how to place the car, what to
run, what a pass looks like, and where to go when it fails.

## Power safety {#power-safety}

- Never leave the car on battery when nobody is with it.
- After any shutdown, turn off the power switch.
- Unplug the battery before you fit or remove a part.

## Pages

1. [Installation](installation.md): the operating system, the driver, and the
   guided car setup.
2. [Health checks](health.md): the checks that show every part of the car
   works.
3. [Calibration](calibration.md): the ESC, the sensors, and the car's
   measurements.
4. [Upgrade](upgrade.md): an existing car onto a new driver release.

Problems not covered here are in the
[troubleshooting guide](../troubleshooting.md).

## Setup order

Follow the steps in order on a new car.

| Step | Task | Page |
| ---- | ---- | ---- |
| 1 | Operating system on the SD card | [Installation](installation.md#os-image) |
| 2 | needrestart, upgrade, driver clone | [Installation](installation.md#driver-install) |
| 3 | Hardware fitting | [Installation](installation.md#hardware-fitting) |
| 4 | Car identity | [Installation](installation.md#identity) |
| 5 | Hardware connection check | [Installation](installation.md#hardware) |
| 6 | NVMe data drive | [Installation](installation.md#nvme) |
| 7 | Access point | [Installation](installation.md#access_point) |
| 8 | Reboot | [Installation](installation.md#reboot) |
| 9 | Teensy firmware | [Installation](installation.md#firmware) |
| 10 | ESC setup | [Calibration](calibration.md#esc-program-settings), then [Installation](installation.md#esc) |
| 11 | Student library and camera firmware | [Installation](installation.md#extras) |
| 12 | Car geometry | [Installation](installation.md#measurements) |
| 13 | Sensor calibration | [Calibration](calibration.md#sensor-calibration) |
| 14 | Hardware health check | [Health checks](health.md#health-run) |
| 15 | Car record | [Installation](installation.md#record) |
| 16 | Profile backup | [Installation](installation.md#finish) |
