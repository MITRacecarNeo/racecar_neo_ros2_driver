# racecar_neo_ros2_driver

ROS2 driver for the **MIT RACECAR Neo v2** — a 1:14-scale autonomous Ackermann-steering racing robot.

This package is the v2 successor to [`racecar-neo-ros2-backend`](https://github.com/MITRacecarNeo/racecar-neo-ros2-backend), with the safety, uptime, and recovery infrastructure ported from [`uav_neo_ros2_driver`](https://github.com/MITUavNeo/uav_neo_ros2_driver). For the full feature catalog of the patterns being inherited, see [docs/features.md](https://github.com/MITUavNeo/uav_neo_ros2_driver/blob/main/docs/features.md) in the UAV Neo repo.

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Quickstart (fresh Ubuntu 24.04 install)](#quickstart-fresh-ubuntu-2404-install)
- [The `racecar` shell tool](#the-racecar-shell-tool)
- [Networking (optional)](#networking-optional)
- [Web dashboard](#web-dashboard)
- [Jupyter notebooks](#jupyter-notebooks)
- [Manual build](#manual-build)
- [Launch](#launch)
- [Changelog](#changelog)
- [License](#license)

## Hardware

| Subsystem | Component | Interface |
|---|---|---|
| Forward camera | Logitech BRIO | gscam over V4L2 (`/dev/cam_forward`) |
| Backward camera | Arducam B0578 | gscam over V4L2 (`/dev/cam_backward`) |
| 2D LIDAR | RPLIDAR A3-class | UART (`/dev/lidar`) |
| IMU | LSM9DS1 | I²C (`0x6B` + `0x1E`) |
| Gamepad | Switch Pro / EasySMX | USB HID (`/dev/input/event*` or `/dev/input/js*`) |
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

Safety/uptime layers (inherited from UAV Neo, shipped in v0.0.4):
- **Mux** enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- **Watchdog** (`scripts/watchdog.py`) supervises 8 nodes with two-signal liveness (ROS topic + `pgrep` on the entry-point path), 30 s restart cooldown, SIGTERM → SIGKILL escalation, FastRTPS SHM orphan sweep every 60 s, Pi 5 PMIC under-voltage alarm. Hardware-aware: skips restart when the device is physically missing.
- **Four systemd units** (`racecar-{teleop,watchdog,dashboard,jupyter}.service`) wired with `BindsTo=` so watchdog dies when teleop dies, and `Wants=` so watchdog auto-starts when teleop starts.
- **Launch wrapper** (`scripts/launch_teleop.sh`) creates `~/logs/<timestamp>/`, updates `~/logs/latest` atomically, sweeps FastRTPS SHM orphans, and `exec`s `ros2 launch` so systemd tracks the launch PID directly.
- **Web dashboard** at `http://<robot>:8080` — 10 node cards, 7 topic-rate rows, System Health (RTC battery + Pi under-voltage alarm), watchdog log tail. Auto-refresh.
- **JupyterLab** at `http://<robot>:8888` with PYTHONPATH/AMENT_PREFIX_PATH pre-set so `import rclpy` works in notebooks.
- **Pre-flight `colcon test` suite** (332 tests) asserting every peripheral, embedding fix commands in failure messages.

## Quickstart (fresh Ubuntu 24.04 install)

Target: Raspberry Pi 5 running **Ubuntu Server 24.04 LTS for arm64** (Noble). ROS2 Jazzy is the only supported distro for this driver — older Ubuntu releases (22.04 Jammy) are **not** supported because Jazzy doesn't install there.

### 1. Image the SD card / NVMe

Use Raspberry Pi Imager → *Other general-purpose OS* → *Ubuntu* → *Ubuntu Server 24.04 LTS (64-bit)*. Before writing, click the gear icon and pre-set:

- **Hostname**: `racecar-neo` (matches what the systemd services + dashboard expect)
- **Username**: `racecar` (the `racecar` shell tool, udev groups, and service unit `User=` are all hard-coded to this name — don't change it)
- **Password**: your choice
- **Wireless LAN**: your home/lab SSID (only needed for the initial setup; later replaced by the AP via `racecar setup networking`)
- **SSH**: enabled, password auth

Boot the Pi, find its IP (`ip neigh` from another machine, or check your router), then `ssh racecar@<ip>`.

### 2. Silence `needrestart` so `apt full-upgrade` doesn't prompt

Ubuntu Server 24.04 ships with `needrestart`, which throws an interactive "restart services?" dialog mid-`apt` if any library upgrade affects a running daemon. Configure it to auto-restart silently before the big upgrade so the rest of setup is unattended:

```sh
sudo apt update && sudo apt -y install needrestart git
sudo sed -i "s/^#\$nrconf{restart} =.*/\$nrconf{restart} = 'a';/" /etc/needrestart/needrestart.conf
sudo sed -i "s/^#\$nrconf{kernelhints} =.*/\$nrconf{kernelhints} = -1;/" /etc/needrestart/needrestart.conf
```

### 3. System upgrade

```sh
sudo apt -y full-upgrade
```

Largest single block of the install (~8–15 min on a fresh image at 10 MB/s). With needrestart silenced above, this runs hands-off.

### 4. Clone and run the orchestrator

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` is idempotent — re-running is safe (each phase checks for existing state and skips when already applied). Sudo password is prompted **once** at the top of the run and cached via a background keepalive for the remaining ~45 min — you can walk away after that prompt.

### 5. Apply group memberships

The setup adds your user to `dialout`, `i2c`, `spi`, `gpio`, and `video`. Group membership applies to **new login sessions only**, so:

```sh
exit                     # close SSH
ssh racecar@<ip>         # back in — groups now active
groups                   # verify: dialout i2c spi gpio video should appear
```

### 6. Plug in the hardware and reboot

With the Pi powered off: connect the Maestro, both cameras, the lidar, the dot matrix (SPI), the IMU (I²C), the Coral EdgeTPU, and the EasySMX gamepad's USB dongle. Power on and:

```sh
sudo reboot
```

After reboot, `racecar-teleop.service` auto-starts and pulls the watchdog via `Wants=racecar-watchdog.service`. Verify:

```sh
racecar status              # USB peripherals + device symlinks + running ros2 nodes
racecar service status      # all 4 racecar-* units should be active+enabled
```

Browse to `http://racecar-neo.local:8080` for the live dashboard.

### 7. (Optional) Switch to AP-mode networking

Once the wired setup works, you can untether the robot from your home WiFi by running:

```sh
racecar setup networking --ssid=racecar-neo-1 --psk='your-password'
```

This brings up an isolated AP on `wlan0` and configures eth0 with both a static IP and DHCP. See [Networking (optional)](#networking-optional). **Run this from a wired (eth0) session or directly on the console** — it reconfigures `wlan0` and will drop SSH-over-WiFi.

### What `setup_all.sh` actually does

Eleven phases, all under `scripts/`:

1. **`setup_ros2.sh`** — ROS2 Jazzy apt repo + message/driver packages
2. **`setup_dev_tools.sh`** — build tools, Python hardware libs (`smbus` / `serial` / `spidev`), GStreamer dev headers
3. **`setup_user_env.sh`** — joins `dialout` / `i2c` / `spi` / `gpio` / `video` groups; sources ROS2 + the `racecar` shell tool in `.bashrc`
4. **`setup_raspi_config.sh`** — `raspi-config` flags: enable I2C, enable SPI, disable serial console (frees `/dev/serial0`)
5. **`setup_udev.sh`** — installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/maestro` etc.)
6. **`setup_dotmatrix.sh`** — `pip install --user luma.led_matrix`
7. **`setup_coral.sh`** — installs `libedgetpu1-std`, `tflite_runtime`, `pycoral` from vendored `depend/` artifacts
8. **`patch_gscam.sh`** — clones `ros-drivers/gscam`, applies the appsink memory-leak fix, builds it as a colcon overlay
9. **`setup_workspace.sh`** — clones `sllidar_ros2` and runs `colcon build --symlink-install`
10. **`setup_jupyter.sh`** — `pip install --user jupyterlab`, creates `~/jupyter_ws/`
11. **`setup_services.sh`** — installs and enables the four systemd units (`racecar-{teleop,watchdog,dashboard,jupyter}.service`)

Individual phase scripts can be run on their own to re-do or skip steps (e.g. `racecar setup networking` for just the networking phase, or `bash scripts/setup_udev.sh` to reinstall the udev rules after a hardware swap).

## The `racecar` shell tool

`setup_user_env.sh` sources [`scripts/racecar-tool.sh`](scripts/racecar-tool.sh) into your `~/.bashrc`. Once you re-open a shell, a single `racecar` command covers the common workflows:

```sh
racecar build               # colcon build --symlink-install + source overlay
racecar test                # colcon test + verbose results
racecar source              # source the workspace overlay
racecar cd                  # chdir to the package source root
racecar teleop              # launch the full stack via launch_teleop.sh
racecar launch dotmatrix    # ros2 launch racecar_neo_ros2_driver dotmatrix.launch.py
racecar watchdog            # run the supervisor in the foreground

racecar service status      # active/enabled snapshot for all 4 racecar-* units
racecar service install     # drop unit files in /etc/systemd/system/ + enable
racecar service start       # default: start teleop (watchdog follows via Wants=)
racecar service stop        # default: stop teleop (watchdog follows via BindsTo=)
racecar service logs teleop # journalctl -u racecar-teleop -f

racecar setup all                       # run the 11-phase orchestrator
racecar setup networking --ssid=foo     # configure eth0 dual-IP + wlan0 AP
racecar setup networking --show         # print persisted overrides

racecar selftest --dmatrix          # run all dot matrix patterns
racecar selftest --dmatrix=font     # just the font scroll
racecar clear --dmatrix             # flash + clear the MAX7219 display
racecar udev                        # re-install the udev rules
racecar cleanup [--force]           # list / kill stale racecar processes + SHM orphans
racecar status                      # USB peripherals + device symlinks + running ros2 nodes
racecar help                        # full usage
```

Tab completion is registered for subcommands; `racecar launch <TAB>` discovers launch files dynamically, `racecar service <TAB>` offers actions, etc.

## Networking (optional)

`scripts/setup_networking.sh` configures two things and is **not** invoked by `setup_all.sh` — it's a separate step because it reconfigures `wlan0` and would drop SSH-over-WiFi sessions during a fresh install. Run it from a wired (eth0) session or directly on the console:

```sh
racecar setup networking --ssid=racecar-neo-1 --psk='your-password'
```

What it does:

1. **eth0 dual-IP** via netplan — eth0 carries both a static address (default `192.168.52.200/24`) and a DHCP-assigned address. Lets you reach the robot at a known IP on a wired-only switch *and* via DHCP on a home network.
2. **wlan0 isolated AP** via NetworkManager — the Pi hosts its own 2.4 GHz WiFi network. Clients can SSH / browse the dashboard / use jupyter, but a NetworkManager dispatcher installs `iptables FORWARD REJECT` rules so AP clients **cannot** route through the Pi to the internet (intentional isolation — keeps the robot's WiFi from becoming a janky general-purpose gateway).

Tunables (persisted to `~/.config/racecar/networking.env` and replayed on every re-run):

| Flag | Default |
|---|---|
| `--ssid=NAME` | `racecar-neo-1` |
| `--psk=PASS` | `racecar@mit` |
| `--channel=N` | `6` |
| `--ap-addr=CIDR` | `10.42.0.1/24` |
| `--eth-static=CIDR` | `192.168.52.200/24` |

Inspect / clear the saved overrides:

```sh
racecar setup networking --show    # print current persisted values
racecar setup networking --reset   # delete the file, revert to defaults
```

Verify after running:

```sh
ip -br addr show eth0           # static + DHCP both present
iw dev wlan0 info               # type AP, your SSID, channel 6
sudo iptables -L FORWARD -n     # two REJECT rules for wlan0
```

## Web dashboard

Once `racecar-teleop.service` is running, browse to `http://<robot>:8080` for a live status page:

- **Nodes**: one card per monitored subsystem (10 total) — green when the expected topic is being advertised, red when not.
- **System Health**: RTC backup battery voltage (green ≥ 3.0 V, yellow 2.7–3.0 V, red < 2.7 V) and the Pi 5 PMIC sticky under-voltage alarm.
- **Topic Rates**: live Hz for `/motor`, `/mux_out`, `/imu`, `/scan`, both cameras, and `/edgetpu/inference`. Yellow when stale (< 0.5 Hz), red when missing.
- **Watchdog Log**: tail of `~/logs/latest/watchdog.log` so you can see restart events.

Refreshes every 3 s; System Health refreshes on a slower 60 s cadence (RTC drifts on the order of weeks, not seconds).

## Jupyter notebooks

`http://<robot>:8888/lab` — JupyterLab with `import rclpy` working out of the box. Notebooks land in `~/jupyter_ws/`. No token / password by default (the systemd unit assumes the robot's network is trusted).

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
