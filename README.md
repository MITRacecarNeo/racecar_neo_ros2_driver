# racecar_neo_ros2_driver

ROS2 driver for the **MIT RACECAR Neo v2** — a 1:14-scale autonomous Ackermann-steering racing robot.

This package is the v2 successor to [`racecar-neo-ros2-backend`](https://github.com/MITRacecarNeo/racecar-neo-ros2-backend), with the safety, uptime, and recovery infrastructure ported from [`uav_neo_ros2_driver`](https://github.com/MITUavNeo/uav_neo_ros2_driver). For the full feature catalog of the patterns being inherited, see [docs/features.md](https://github.com/MITUavNeo/uav_neo_ros2_driver/blob/main/docs/features.md) in the UAV Neo repo.

## Hardware

| Subsystem | Component | Interface |
|---|---|---|
| Forward camera | Logitech BRIO | gscam over V4L2 (`/dev/cam_forward`) |
| Backward camera | Arducam B0578 | gscam over V4L2 (`/dev/cam_backward`) |
| 2D LIDAR | RPLIDAR A3-class | UART (`/dev/lidar`) |
| IMU | LSM9DS1 | I²C (`0x6B` + `0x1E`) |
| Gamepad | EasySMX | USB HID (`/dev/input/js0`) |
| Motor / steering | Pololu Maestro | USB serial (`/dev/maestro`) |
| ML inference | Coral EdgeTPU | USB |
| Display | MAX7219 dot matrix (3 cascaded) | SPI (`/dev/spidev0.0`) |

All `/dev/*` paths are stable udev symlinks installed by `scripts/setup_udev.sh` — devices won't shift between `ttyACM0`/`ttyACM1` or `video0`/`video4` across reboots.

## Architecture

```
EasySMX ─→ joy_node ─→ gamepad_node ──┐
                                       ├──→ mux ──→ throttle ──→ pwm ──→ Maestro
                       /drive (auto) ──┘
```

Sensor and ML nodes publish independently:
- `/camera/forward`, `/camera/backward` (sensor_msgs/Image)
- `/imu`, `/mag` (sensor_msgs/Imu, MagneticField)
- `/scan` (sensor_msgs/LaserScan)
- `/edgetpu/inference` (vision_msgs/Detection2DArray) — `edgetpu_node` consumes `/camera/forward`

Display node subscribes:
- `/dotmatrix/text` (std_msgs/String) — renders user messages; falls back to a mode glyph (IDLE / TELEOP / AUTO) tied to the gamepad state

Safety/uptime layers (inherited from UAV Neo):
- Mux node enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- Watchdog with two-signal liveness (ROS topic + `pgrep`), hardware-aware restart skip, and FastRTPS SHM orphan cleanup.
- systemd units (`racecar-teleop.service`, `racecar-watchdog.service`) with `BindsTo=` graphs and `KillMode=control-group`.
- Per-session timestamped log dirs with `~/logs/latest` atomic symlink.
- Pre-flight `colcon test` suite asserting every peripheral and embedding fix commands in failure messages.

## Quick start (fresh machine)

Ubuntu 24.04 (Noble) on a Raspberry Pi 5.

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
# Log out + back in (group changes take effect)
racecar teleop
```

`setup_all.sh` is idempotent — re-running is safe. It runs eight phases:

1. **`setup_ros2.sh`** — ROS2 Jazzy apt repo + message/driver packages
2. **`setup_dev_tools.sh`** — build tools, Python hardware libs (`smbus`/`serial`/`spidev`), GStreamer dev headers
3. **`setup_user_env.sh`** — joins `dialout`/`i2c`/`spi`/`gpio`/`video` groups; sources ROS2 + the `racecar` shell tool in `.bashrc`
4. **`setup_udev.sh`** — installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/maestro` etc.)
5. **`setup_dotmatrix.sh`** — enables SPI via `raspi-config` and installs `luma.led_matrix`
6. **`setup_coral.sh`** — installs `libedgetpu1-std`, `tflite_runtime`, `pycoral` from vendored `depend/` artifacts
7. **`patch_gscam.sh`** — clones `ros-drivers/gscam`, applies the appsink memory-leak fix, builds it as a colcon overlay
8. **`setup_workspace.sh`** — clones `sllidar_ros2` and runs `colcon build --symlink-install`

Individual phase scripts can be run on their own to re-do or skip steps.

## The `racecar` shell tool

`setup_user_env.sh` sources [`scripts/racecar-tool.sh`](scripts/racecar-tool.sh) into your `~/.bashrc`. Once you re-open a shell you'll have a single `racecar` command for the common workflows:

```sh
racecar build              # colcon build --symlink-install + source overlay
racecar test               # colcon test + verbose results
racecar source             # source the workspace overlay
racecar teleop             # launch the full teleop stack
racecar launch dotmatrix   # ros2 launch racecar_neo_ros2_driver dotmatrix.launch.py
racecar clear --dmatrix    # flash + clear the MAX7219 display
racecar udev               # re-install the udev rules
racecar status             # USB peripherals + device symlinks + running ros2 nodes
racecar help               # full usage
```

Tab completion is registered for subcommands; `racecar launch <TAB>` discovers launch files dynamically.

## Manual build

If you'd rather not use the shell tool:

```sh
cd ~/ros2_ws
colcon build --packages-select racecar_neo_ros2_driver --symlink-install
source install/setup.bash
```

## Launch

```sh
racecar teleop                          # or: ros2 launch racecar_neo_ros2_driver teleop.launch.py
racecar launch camera_forward           # individual nodes too
racecar launch camera_backward
racecar launch imu
racecar launch lidar
racecar launch edgetpu
racecar launch dotmatrix
```

For boot-time startup, see [scripts/](./scripts/) for systemd units and the `setup_all.sh` idempotent installer.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md).

## License

GPLv3 — see [LICENSE](./LICENSE).
