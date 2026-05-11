# racecar_neo_ros2_driver

ROS2 driver for the **MIT RACECAR Neo v2** — a 1:14-scale autonomous Ackermann-steering racing robot.

This package is the v2 successor to [`racecar-neo-ros2-backend`](https://github.com/MITRacecarNeo/racecar-neo-ros2-backend), with the safety, uptime, and recovery infrastructure ported from [`uav_neo_ros2_driver`](https://github.com/MITUavNeo/uav_neo_ros2_driver). For the full feature catalog of the patterns being inherited, see [docs/features.md](https://github.com/MITUavNeo/uav_neo_ros2_driver/blob/main/docs/features.md) in the UAV Neo repo.

## Hardware

| Subsystem | Component | Interface |
|---|---|---|
| Forward camera | Logitech BRIO | gscam over V4L2 |
| Backward camera | Arducam B0578 | gscam over V4L2 |
| 2D LIDAR | RPLIDAR | UART (`/dev/ttyUSB0`) |
| IMU | LSM9DS1 | I²C (`0x6B` + `0x1E`) |
| Gamepad | EasySMX | USB HID (`/dev/input/js0`) |
| Motor / steering | Pololu Maestro | USB serial (`/dev/ttyACM0`) |
| ML inference | Coral EdgeTPU | USB |
| Display | MAX7219 dot matrix | SPI (`/dev/spidev0.0`) |

## Architecture

```
EasySMX ─→ joy_node ─→ gamepad_node ──┐
                                       ├──→ mux ──→ throttle ──→ pwm ──→ Maestro
                       /drive (auto) ──┘
```

Sensor nodes publish independently:
- `/camera/forward`, `/camera/backward` (sensor_msgs/Image)
- `/imu`, `/mag` (sensor_msgs/Imu, MagneticField)
- `/scan` (sensor_msgs/LaserScan)
- `/edgetpu/inference` (vision_msgs/Detection2DArray)

Safety/uptime layers (inherited from UAV Neo):
- Mux node enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- Watchdog with two-signal liveness (ROS topic + `pgrep`), hardware-aware restart skip, and FastRTPS SHM orphan cleanup.
- systemd units (`racecar-teleop.service`, `racecar-watchdog.service`) with `BindsTo=` graphs and `KillMode=control-group`.
- Per-session timestamped log dirs with `~/logs/latest` atomic symlink.
- Pre-flight `colcon test` suite asserting every peripheral and embedding fix commands in failure messages.

## Build

ROS2 Jazzy on Ubuntu 24.04.

```sh
cd ~/ros2_ws
colcon build --packages-select racecar_neo_ros2_driver
source install/setup.bash
```

## Launch

```sh
ros2 launch racecar_neo_ros2_driver teleop.launch.py
```

For boot-time startup, see [scripts/](./scripts/) for systemd units and the `setup_all.sh` idempotent installer.

## License

GPLv3 — see [LICENSE](./LICENSE).
