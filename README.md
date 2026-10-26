# racecar_neo_ros2_driver

ROS2 driver for the **MIT RACECAR Neo v2**: a 1:14-scale autonomous Ackermann-steering racing robot.

This package is the v2 successor to [`racecar-neo-ros2-backend`](https://github.com/MITRacecarNeo/racecar-neo-ros2-backend), with the safety, uptime, and recovery infrastructure ported from [`uav_neo_ros2_driver`](https://github.com/MITUavNeo/uav_neo_ros2_driver). For the full feature catalog of the patterns being inherited, see [docs/features.md](https://github.com/MITUavNeo/uav_neo_ros2_driver/blob/main/docs/features.md) in the UAV Neo repo.

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Quickstart (fresh Ubuntu 24.04 install)](#quickstart-fresh-ubuntu-2404-install)
- [The `racecar` shell tool](#the-racecar-shell-tool)
- [Linters](#linters)
- [Networking (optional)](#networking-optional)
- [Ethernet addressing](#ethernet-addressing)
- [WiFi client](#wifi-client)
- [Desktop toggle](#desktop-toggle)
- [Web dashboard](#web-dashboard)
- [Lab dashboards](#lab-dashboards)
- [Autonomy gate](#autonomy-gate)
- [Low-battery shutdown](#low-battery-shutdown)
- [Car data](#car-data)
- [Bag recording](#bag-recording)
- [Maps](#maps)
- [Jupyter notebooks](#jupyter-notebooks)
- [Manual build](#manual-build)
- [Launch](#launch)
- [Sensor calibration](#sensor-calibration)
- [Car profile](#car-profile)
- [RTC backup cell](#rtc-backup-cell)
- [Bootloader EEPROM](#bootloader-eeprom)
- [ROS discovery scope](#ros-discovery-scope)
- [FlySky transmitter](#flysky-transmitter)
- [Changelog](#changelog)
- [License](#license)

## Hardware

| Subsystem | Component | Interface |
|---|---|---|
| Camera (color+depth+IMU) | Intel RealSense D435i | realsense2_camera over USB 3.x (`8086:0b3a`) |
| 2D LIDAR | RPLIDAR A3-class | UART (`/dev/lidar`) |
| Gamepad | Switch Pro / EasySMX | USB HID (`/dev/input/event*` or `/dev/input/js*`) |
| Motor / steering / IMU / power | NEO-PIT PCB (Teensy 4.1; LSM9DS1 + INA226 + hall encoder onboard) | UART (`/dev/neo-pit-pcb`) |
| ML inference | Coral Edge TPU M.2 (Apex) | PCIe (`/dev/apex_0`) |
| Display | MAX7219 dot matrix (3 cascaded) | driven by the Teensy; frames sent over the NEO-PIT UART |

All `/dev/*` paths are stable udev symlinks installed by `scripts/setup_udev.sh`, so devices won't shift between `ttyACM0` and `ttyACM1` across reboots.

## Architecture

```
EasySMX ─→ joy_node ─→ gamepad_node ──┐
                                       ├──→ mux ──→ throttle ──→ pit ──→ NEO-PIT PCB
                       /drive (auto) ──┘
```

Sensor and ML nodes publish independently:
- `/camera/color`, `/camera/depth` (sensor_msgs/Image): RealSense D435i color and depth streams
- `/imu/realsense` (sensor_msgs/Imu): RealSense D435i IMU, remapped from `/camera/imu`; the camera's optical axes (x right, y down, z forward)
- `/imu/lsm9ds1`, `/mag` (sensor_msgs/Imu, MagneticField): LSM9DS1 on the NEO-PIT board, republished from `pit_node` telemetry in REP-103 body axes (x forward, y left, z up)
- `/imu/lsm9ds1/raw`, `/mag/raw` (sensor_msgs/Imu, MagneticField): the same telemetry axis-remapped and sensitivity-scaled but with no bias applied; the calibration utilities fit against these
- `/imu/fused` (sensor_msgs/Imu): `imu_fusion_node` blends `/imu/realsense` and `/imu/lsm9ds1` in REP-103 body axes (single-source passthrough when one is live)
- `/scan` (sensor_msgs/LaserScan)
- `/autonomy/pose`, `/autonomy/status`, `/autonomy/save_map`, `/autonomy/autosave`, `/autonomy/global_localization`: `autonomy_node`, started by `racecar launch autonomy` (see docs/architecture.md, "Autonomy interface")
- `/edgetpu/inference` (vision_msgs/Detection2DArray): `edgetpu_node` consumes `/camera/color`

Display node subscribes:
- `/dotmatrix/text` (std_msgs/String): renders user messages; falls back to a mode glyph (IDLE / MAN / AUTO) tied to the gamepad state

Safety and uptime layers:
- **Mux** enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- **Watchdog** (`scripts/watchdog.py`) supervises 7 nodes with two liveness signals (ROS topic + the node's entry-point path in `/proc`) and restarts a node when either fails. A 60 s startup grace, backoff of 30, 60, 120 and 240 s between restarts, and a give-up after 5 restarts that do not hold 60 s, re-armed by replugging the device or restarting teleop. Logs on state changes only, skips subsystems disabled at launch, and writes `~/logs/latest/watchdog_state.json`, which `racecar status` and the dashboard read. SIGTERM to SIGKILL escalation, FastRTPS SHM orphan sweep every 60 s, Pi 5 PMIC under-voltage alarm. Restart is skipped when the device is physically missing.
- **Seven core systemd units** (`racecar-{data,teleop,watchdog,dashboard,jupyter,mapview,battery-guard}.service`), enabled at boot. `racecar-data` records the data root first; the watchdog is wired with `BindsTo=` so it stops with teleop, and `Wants=` so it starts with teleop. `racecar-autonomy.service` installs disabled and starts on demand. Four more units carry the lab dashboards; they install disabled and start from the 8080 apps page.
- **Launch wrapper** (`scripts/launch_teleop.sh`) creates `~/logs/<timestamp>/`, updates `~/logs/latest` atomically, sweeps FastRTPS SHM orphans, and `exec`s `ros2 launch` so systemd tracks the launch PID directly.
- **Battery guard** (`scripts/battery_guard.py`, `racecar-battery-guard.service`) warns at 15 minutes of battery left and turns the car off safely at 7.1 V; see [Low-battery shutdown](#low-battery-shutdown).
- **Web dashboard** at `http://<robot>:8080`: 9 node cards, 9 topic-rate rows, System Health (data drive, RTC battery, Pi under-voltage alarm), watchdog log tail. Auto-refresh.
- **JupyterLab** at `http://<robot>:8888` with PYTHONPATH/AMENT_PREFIX_PATH pre-set so `import rclpy` works in notebooks.
- **`colcon test` suite** covering the nodes, scripts, setup phases and attached peripherals, with fix commands in the failure messages.

Node responsibilities, the full topic reference, launch composition, and the
calibration data flow are in [docs/architecture.md](./docs/architecture.md).
Failure modes that shaped the code, and the reasoning behind choices that look
arbitrary at the call site, are in
[docs/troubleshooting.md](./docs/troubleshooting.md).

## Quickstart (fresh Ubuntu 24.04 install)

Target: Raspberry Pi 5 running **Ubuntu Desktop 24.04 LTS for arm64** (Noble). ROS2 Jazzy is the only supported distro for this driver; older Ubuntu releases (22.04 Jammy) are **not** supported because Jazzy doesn't install there.

### 1. SD card or NVMe image

Use Raspberry Pi Imager -> *Other general-purpose OS* -> *Ubuntu* -> *Ubuntu Desktop 24.04 LTS (64-bit)*. Before writing, click the gear icon and pre-set:

- **Hostname**: `racecar-neo` (matches what the systemd services + dashboard expect)
- **Username**: `racecar` (the `racecar` shell tool, udev groups, and service unit `User=` are all hard-coded to this name; don't change it)
- **Password**: your choice
- **Wireless LAN**: your home/lab SSID. The Pi's built-in `wlan0` stays a client of this network; the AP set up later runs on a separate dongle.
- **SSH**: enabled, password auth

Boot the Pi, find its IP (`ip neigh` from another machine, or check your router), then `ssh racecar@<ip>`.

### 2. needrestart configuration

Ubuntu 24.04 ships with `needrestart`, which throws an interactive "restart services?" dialog mid-`apt` if any library upgrade affects a running daemon. Configure it to auto-restart silently before the big upgrade so the rest of setup is unattended:

```sh
sudo apt update && sudo apt -y install needrestart git
sudo sed -i "s/^#\$nrconf{restart} =.*/\$nrconf{restart} = 'a';/" /etc/needrestart/needrestart.conf
sudo sed -i "s/^#\$nrconf{kernelhints} =.*/\$nrconf{kernelhints} = -1;/" /etc/needrestart/needrestart.conf
```

### 3. System upgrade

```sh
sudo apt -y full-upgrade
```

Largest single block of the install (~8-15 min on a fresh image at 10 MB/s). With needrestart silenced above, this runs hands-off.

### 4. Repository clone and setup run

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
```

Run it as the `racecar` user, without `sudo`, from that folder; it refuses otherwise, because the services name that user and path. `setup_all.sh` is idempotent; re-running is safe (each phase checks for existing state and skips when already applied). Sudo password is prompted **once** at the top of the run and cached via a background keepalive for the remaining ~45 min; you can walk away after that prompt. It ends by asking for a reboot.

### 5. Group membership activation

The setup adds your user to `dialout`, `i2c`, `spi`, `gpio`, and `video`. Group membership applies to **new login sessions only**, so:

```sh
exit                     # close SSH
ssh racecar@<ip>         # back in; groups now active
groups                   # verify: dialout i2c spi gpio video should appear
```

### 6. Hardware connection and reboot

With the Pi powered off: connect the NEO-PIT PCB (motor, steering, IMU, and the dot matrix chain all hang off it), the RealSense camera, the lidar, the Coral Edge TPU M.2 card, and the EasySMX gamepad's USB dongle. Power on and:

```sh
sudo reboot
```

`racecar-autonomy.service` (SLAM mapping or localization on top of teleop) is installed but not enabled; it starts on demand.

After reboot, `racecar-teleop.service` auto-starts and pulls the watchdog via `Wants=racecar-watchdog.service`. Verify:

```sh
racecar status              # full diagnostic; exits non-zero only on a FAIL
racecar service status      # the 6 core racecar-* units should be active+enabled
```

Browse to `http://racecar-neo.local:8080` for the live dashboard.

### 7. AP networking (optional)

To add the robot's own isolated access point:

```sh
racecar setup networking --ssid=racecar-neo-1 --psk='your-password'
```

This brings up an isolated AP on the ALFA dongle (`wlan1`) and puts eth0 in one addressing mode, static by default. See [Networking (optional)](#networking-optional). **Run it from the console or over `wlan0`**; the AP is reconfigured, and the eth0 step can drop an SSH session arriving over eth0.

### `setup_all.sh` phases

Twelve phases, all under `scripts/`:

1. **`setup_ros2.sh`**: ROS2 Jazzy apt repo + message/driver packages (`xacro` and `robot_state_publisher` for the frame tree, slam_toolbox, Nav2 AMCL and map server), `nvme-cli` and `gdisk` for `racecar disk`
2. **`setup_dev_tools.sh`**: build tools, Python hardware libs (`smbus` / `serial` / `spidev`), Pi tools and services the car relies on (`raspi-config`, `rpi-eeprom`, `vcgencmd`, NetworkManager, avahi, polkit, matplotlib), and the pinned linters (`ruff`, `black`, `mypy`) per-user
3. **`setup_user_env.sh`**: joins `dialout` / `i2c` / `spi` / `gpio` / `video` groups; installs the polkit rules in `scripts/polkit/` (NetworkManager for `racecar wifi`, the autonomy unit for `racecar map`, the lab dashboards for the apps page, the teleop restart for Features); sources ROS2 + the `racecar` shell tool in `.bashrc` and puts `~/.local/bin` on `PATH`
4. **`setup_raspi_config.sh`**: boot-level configuration: enable I2C, enable SPI, disable serial console (frees the GPIO UART / `ttyAMA0` for the NEO-PIT link), `usb_max_power_enable=1`, enable RTC backup-cell trickle charging (`RTC_VCHG_UV=0` turns it off; see [RTC backup cell](#rtc-backup-cell)), and reconcile the bootloader EEPROM (`RACECAR_EEPROM=0` skips it; see [Bootloader EEPROM](#bootloader-eeprom))
5. **`setup_udev.sh`**: installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/neo-pit-pcb`, `/dev/lidar`)
6. **`setup_dotmatrix.sh`**: `pip install --user luma.led_matrix` (pinned), the font source for the `dotmatrix_node` rasterizer
7. **`setup_coral.sh`**: installs `libedgetpu1-std`, `tflite_runtime`, `pycoral` from vendored `depend/` artifacts; on a Pi 5, also the M.2 card's gasket DKMS driver and the `coral-msi` overlay, so a card fitted after setup works (reboot required)
8. **`setup_realsense.sh`**: installs `realsense2_camera` (apt) + the Pi 5 IMU IIO permission fix (script, udev rule, boot service)
9. **`setup_workspace.sh`**: clones `sllidar_ros2` at a pinned commit and runs `colcon build --symlink-install`; `setup_all.sh` then runs `racecar profile migrate` (see [Car profile](#car-profile))
10. **`setup_jupyter.sh`**: `pip install --user jupyterlab` and the library's notebook packages (pinned), creates `~/jupyter_ws/`, links `~/jupyter_ws/data` and `~/jupyter_ws/maps` to the [data root](#car-data)
11. **`setup_services.sh`**: installs and enables the core systemd units (`racecar-{data,teleop,watchdog,dashboard,jupyter,mapview}.service`), installs `racecar-autonomy.service` disabled, and installs `/etc/profile.d/racecar-data.sh`
12. **`setup_dashboards.sh`**: installs a stopped, disabled `racecar-*` unit for each of the four lab dashboards in `scripts/dashboards/` (web teleop, line follow, wall follow, camera label) and removes units left by retired dashboards (`RACECAR_DASHBOARDS=0` skips it)

Individual phase scripts can be run on their own to re-do or skip steps (e.g. `racecar setup networking` for just the networking phase, or `bash scripts/setup_udev.sh` to reinstall the udev rules after a hardware swap).

## The `racecar` shell tool

`setup_user_env.sh` sources [`scripts/racecar-tool.sh`](scripts/racecar-tool.sh) into your `~/.bashrc`. Once you re-open a shell, a single `racecar` command covers the common workflows:

```sh
racecar build               # colcon build --symlink-install + source overlay
racecar test                # colcon test + verbose results
racecar lint                # ruff, black --check and mypy over the package
racecar source              # source the workspace overlay
racecar cd                  # chdir to the package source root
racecar teleop              # launch the full stack via launch_teleop.sh
racecar launch dotmatrix    # ros2 launch racecar_neo_ros2_driver dotmatrix.launch.py
racecar watchdog            # run the supervisor in the foreground

racecar service status      # active/enabled snapshot, core units, autonomy, and dashboards
racecar service install     # drop unit files in /etc/systemd/system/ + enable
racecar service start       # default: start teleop (watchdog follows via Wants=)
racecar service stop        # default: stop teleop (watchdog follows via BindsTo=)
racecar service logs teleop # journalctl -u racecar-teleop -f
racecar service start wallfollow  # a lab dashboard; stops the other /drive publishers
racecar service enable wallfollow # enable one unit; bare `enable` covers the core five

racecar setup all                       # run the 12-phase orchestrator
racecar setup networking --ssid=racecar-neo-2   # configure eth0 addressing + ALFA-dongle AP
racecar setup networking --show         # print persisted overrides
racecar setup dashboards                # clone the lab dashboards + install units
racecar setup realsense                 # offline D435i firmware flash

racecar udev                        # re-install the udev rules
racecar cleanup [--force]           # list / kill stale racecar processes + SHM orphans
racecar status                      # full diagnostic (devices, sensors, autonomy, system, network)
racecar status --quick              # host checks only; skips the ROS sampling phase
racecar status --strict             # exit non-zero on any WARN or SKIP as well
racecar eth status                  # eth0 addressing mode + conflict checks
racecar wifi list                   # visible networks on wlan0
racecar desktop status              # GNOME on/off for the next boot
racecar log start lap3              # record a bag; racecar log stop to finalize
racecar log analyze                 # summarize the newest bag
racecar map start room_1            # build a map; racecar map save, then racecar map stop
racecar map load room_1             # localize on a saved map
racecar profile                     # this car's settings in ~/.config/racecar
racecar data                        # where the car's data goes, free space
racecar disk                        # the NVMe data drive; setup, format, check
racecar library --list              # student library folders under ~/jupyter_ws
racecar help                        # full usage
```

Tab completion is registered for subcommands; `racecar launch <TAB>` discovers launch files dynamically, `racecar service <TAB>` offers actions, etc.

The dot matrix self-test is `scripts/dmatrix_patterns.py`. It publishes patterns to `/dotmatrix/pixels`, so `dotmatrix_node` must be running:

```sh
racecar launch dotmatrix                                    # in another shell
python3 ~/ros2_ws/src/racecar_neo_ros2_driver/scripts/dmatrix_patterns.py all
```

Patterns: `all` (default), `checkerboard`, `all-on`, `sweep`, `module-id`, `font`.

## Linters

`racecar lint` runs three checks from the package source root and exits non-zero if any fails:

| Tool | Checks | Scope |
|---|---|---|
| `ruff check` | pycodestyle, pyflakes, bugbear, comprehensions, quotes, pyupgrade | whole package |
| `black --check` | formatting (99 columns, single quotes kept) | whole package |
| `mypy` | types; every function signature must be annotated | `racecar_neo_ros2_driver/`, `launch/`, `scripts/` |

Settings live in `pyproject.toml`; `scripts/dashboards/` and the colcon `build/`, `install/`, `log/` trees are excluded. `setup_dev_tools.sh` installs pinned versions per-user into `~/.local/bin`, and `setup_user_env.sh` puts that directory on `PATH`. To reformat rather than check, run `black .` from the package root.

`racecar test` still runs `ament_flake8` and `ament_pep257` (`test/test_flake8.py`, `test/test_pep257.py`), which own import order and docstring style.

## Networking (optional)

`scripts/setup_networking.sh` configures two things and is **not** invoked by `setup_all.sh`. Run it from the console or over `wlan0` (client WiFi): it reconfigures the AP, so a session over the AP drops, and switching the eth0 mode drops a session over eth0.

```sh
racecar setup networking --psk='your-password'
```

With no `--ssid` or saved car ID, it prompts for this car's ID and sets the SSID to `racecar-neo-<id>`, so multiple cars on the same network don't collide. The ID is persisted and reused on later runs.

What it does:

1. **eth0 addressing** via `setup_eth.sh`; eth0 is put in exactly one IPv4 mode, static by default at `192.168.52.200/24`, so the robot is reachable at a known IP on a bare switch. The eth0-SSH check in `setup_eth.sh` applies here too: a mode switch asks first when the session arrives over eth0. See [Ethernet addressing](#ethernet-addressing).
2. **ALFA-dongle isolated AP** via NetworkManager; the AP runs on the ALFA MT7612U dongle (pinned to `wlan1` by the udev rule), hosting its own 2.4 GHz WiFi network. Clients can SSH / browse the dashboard / use jupyter, but a NetworkManager dispatcher installs `iptables FORWARD REJECT` rules so AP clients **cannot** route through the Pi to the internet. The Pi's built-in `wlan0` is left in default client mode.

Tunables (persisted to `~/.config/racecar/networking.env` and replayed on every re-run):

| Flag | Default |
|---|---|
| `--ssid=NAME` | `racecar-neo-<id>` (id from the prompt; full override) |
| `--psk=PASS` | `racecar@mit` |
| `--channel=N` | `6` |
| `--ap-addr=CIDR` | `10.42.0.1/24` |
| `--ap-iface=NAME` | `wlan1` (the ALFA dongle) |
| `--eth-static=CIDR` | `192.168.52.200/24` |
| `--eth-mode=MODE` | `static` (or `dynamic`) |

Inspect / clear the saved overrides:

```sh
racecar setup networking --show    # print current persisted values
racecar setup networking --reset   # disable the wlan1 AP + clear the saved car ID
```

`--reset` disables the AP on `wlan1` (downs and deletes the connection) and clears the saved car ID/overrides, leaving eth0 untouched. Run it before capturing a golden image so the clone ships with no active AP and no baked-in SSID; each car then sets its own ID on first `racecar setup networking`.

Verify after running:

```sh
racecar eth status              # exactly one IPv4 address, no conflict
iw dev wlan1 info               # type AP, your SSID, channel 6 (ALFA dongle)
iw dev wlan0 info               # type managed (Pi built-in, client/default)
sudo iptables-nft -L FORWARD -nv   # two REJECT rules for wlan1, above nm-sh-fw-wlan1
```

Use `iptables-nft ... -nv`. NetworkManager installs its shared-mode forward rules through nftables and the dispatcher adds its REJECT rules through `iptables-nft`; without `-v` the listing hides the `in`/`out` columns, so the two REJECT rules (one `in wlan1`, one `out wlan1`) look like they match nothing. `sudo nft list ruleset` shows both tables. From a client joined to the AP, `curl -m 3 https://google.com` must time out.

## Ethernet addressing

eth0 holds exactly one IPv4 addressing mode. Carrying a static address and a DHCP lease at once made the static drop; the reasoning is in [docs/troubleshooting.md](docs/troubleshooting.md#eth0-addressing).

```sh
racecar eth                 # or: racecar eth status
racecar eth static          # 192.168.52.200/24, no gateway (the default)
racecar eth dynamic         # address and default route from DHCP
racecar eth static --addr=10.0.0.50/24
```

Static carries no gateway, no DNS and no IPv6 default route, so a static car reaches the internet over `wlan0` or not at all; switch to `dynamic` when you need `apt` over the wire.

`status` reports the configured mode, the live addresses, both default routes, and fails when it finds more than one global IPv4 address on eth0. The link-local `fe80::` address and SLAAC addresses do not count.

**Switching modes drops an SSH session arriving over eth0.** The command detects that and asks first; use `wlan0`, the AP, or an HDMI console, or pass `--force`.

### Link monitor

`racecar eth monitor` records the addresses, both default routes, carrier, operstate and NetworkManager state, logging a line whenever any of them changes plus a heartbeat every 15 minutes.

```sh
racecar eth monitor                     # foreground, Ctrl-C to stop
racecar eth monitor --once              # one sample, for a quick look
```

For a soak measured in days, install the unit instead:

```sh
sudo cp scripts/racecar-eth-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now racecar-eth-monitor
grep -v '\[OK\]' ~/logs/eth-monitor.log     # every state change, heartbeats hidden
```

A clean run is `START` followed by heartbeats. Any `ADDR_LOST`, `CARRIER` or `OPERSTATE` line is the event worth reading. The unit is a diagnostic; disable it once the soak is done.

## WiFi client

`wlan0` is the Pi's built-in Broadcom radio and the only client interface. `wlan1` is the ALFA dongle carrying the AP, and nothing in this command touches it.

```sh
racecar wifi                    # or: racecar wifi status
racecar wifi list               # cached scan, one row per SSID
racecar wifi list --rescan      # force a fresh scan (about ten seconds)
racecar wifi connect <ssid>     # prompts for whatever it needs
racecar wifi disconnect
```

`list` groups the scan by SSID and keeps the strongest signal, because a scan returns one row per BSSID: on a car parked in a lab that is 30-plus rows for about a dozen real networks. Hidden networks are collapsed into a count.

`connect` brings up a saved profile as-is, whatever its security type. For a new network it asks for a passphrase, or for an identity and password on an enterprise (802.1X) network, and nothing else. Every enterprise profile gets system CA certificates plus a `domain-suffix-match` derived from the identity's realm, so credentials are never offered to an access point that cannot prove who it is. Use `--ca-cert=` and `--domain-suffix-match=` where that derivation does not fit.

A joined network survives a reboot: `connect` sets both the profile's `connection.autoconnect` and the device's autoconnect flag. `disconnect` drops the link until the next `connect` or reboot; to stop the car rejoining after a reboot, delete the profile with `nmcli connection delete <ssid>`. `racecar wifi status` reports the boot outcome on its `after boot` line. Details: [docs/troubleshooting.md](docs/troubleshooting.md#wifi-persistence).

### Networking authorization

On a car whose polkit still carries the stock policy, `connect` and `disconnect` fail over SSH with:

```
Error: Connection activation failed: Not authorized to control networking.
```

`setup_user_env.sh` installs `/etc/polkit-1/rules.d/49-racecar-network.rules`, which grants the five NetworkManager actions `racecar wifi` needs to the `sudo` group. Cars imaged before v0.8.1 need one run; it takes effect without a restart:

```sh
bash ~/ros2_ws/src/racecar_neo_ros2_driver/scripts/setup_user_env.sh
```

`racecar wifi connect` checks the permission before it prompts and names this remedy. The trade-off is in [docs/troubleshooting.md](docs/troubleshooting.md#networkmanager-authorization).

## Desktop toggle

The GNOME desktop ships enabled. Headless users turn it off without removing anything:

```sh
racecar desktop             # or: racecar desktop status
racecar desktop disable     # boot to multi-user.target
racecar desktop enable      # boot to graphical.target
```

Changes apply on the **next boot**; `status` reports a pending change when the default and active targets disagree. Packages are never removed, so the toggle works on a car with no network. Scope: [docs/troubleshooting.md](docs/troubleshooting.md#desktop-toggle-scope).

## Web dashboard

Once `racecar-teleop.service` is running, browse to `http://<robot>:8080`.

**Apps.** One tile per app, in two groups. Hover over a tile for what it
does. A white border means the app is running.

- Core: map viewer and JupyterLab, which start at boot, the safety
  override, and Turn Off (below).
- Utilities: web teleop, line follow, wall follow, and camera label.
- Tap a tile to open the app in a new tab. A stopped utility is started
  first, and the tab shows a loading bar until the app is ready. Cancel
  closes that tab without opening the app.
- Web teleop, line follow, wall follow, and autonomy (mapping or
  localization) drive the car, so only one runs at a time. Opening one while
  another runs asks before closing the other; a map being built is saved
  first. The map viewer and the Autonomy tile ask the same way, and
  `racecar map start`, `load`, and `resume` close the driving apps.
- Clear apps stops every utility to free the CPU. Core apps keep running.
- Starting and stopping utilities from the page needs the permission rule
  that `scripts/setup_user_env.sh` installs.

**Features.** Expand it below the apps. Parts of the car that use the most
CPU; turn off what a lab does not need. A white border means on.

- Camera: color and depth. Line follow and Camera label need it; opening
  either with the camera off asks to turn it on.
- Detections: object detection on the camera image. Turning it on also
  turns on the camera.
- Autonomy: mapping or localization. Off stops it (mapping asks to save
  first). On starts the last run again, or opens the map viewer if there
  is none.

Turning the camera or detections on or off asks first, then restarts
teleop. The car stops for about 15 seconds. A running map or localization
stops first and starts again after. Every boot starts with the camera on
and detections off. Needs the permission rule that
`scripts/setup_user_env.sh` installs.

**Safety override.** The Safety override tile lets programs and lab
dashboards drive with no controller in hand. It asks first. While it is on,
the tile turns red and:

- A red banner shows on this page and on web teleop, line follow, and wall
  follow. The dot matrix shows AUTO, and the LED strip flashes amber.
- Holding LB drives from the controller.
- Any other controller button turns it off.
- Turn off in any banner turns it off.
- It stays on while any of those pages is open. It turns off 2 s after the
  last one closes, or after the phone puts them all in the background.

The car may start moving without warning. Do not leave it unattended.

**Battery.** The header shows the battery: time left, and average power
over the last 5 minutes. Click the time to switch between percent left,
time left, and the time the car turns off ("until 12:32"). The page
remembers the choice. Both hide until the battery guard has an estimate,
about 30 s after boot.

- With 15 minutes or less left, a red banner shows on this page and on
  every app page.
- During a shutdown countdown the banner counts down each second.
  Cancel shutdown shows when the countdown came from the time estimate.
  It works once per boot. A countdown at 7.1 V cannot be canceled.

See [Low-battery shutdown](#low-battery-shutdown).

**Turn Off.** The Turn Off tile shuts the car down safely. It asks first.
It stops a recording, the driving apps, autonomy (saving a map being
built), and teleop, then powers off. Turn off the power switch after the
car is off.

**Overview.** Expand it to see the car live: color camera, depth camera,
lidar (seen from above, forward up), IMU rotation rates and encoder speed over
the last 30 s, battery voltage and current over the last 30 minutes, and CPU
temperature and RAM in use over the last 2 hours. Hover or tap a chart to read a value.
The camera tiles start paused; Show camera starts them. The lidar, IMU, and
encoder feeds run only while the section is open.
The 2-hour history starts over when the dashboard service restarts.

**System status.** Expand it at the bottom of the page.

- Basic view: indicators grouped as battery, sensors, compute, actuation, and
  autonomy, each with its name, topic rate in Hz, and a status mark. Hover or
  tap an indicator for details. The LED strip shows Off until a program
  drives it.
- Basic view also has a Data indicator: the data root, NVMe or SD card, and
  free space.
- Advanced view: node cards, health (data drive, RTC backup cell, Pi
  under-voltage alarm),
  topic rates, autonomy, and the watchdog log. Clear log hides the lines so
  far; the log file is kept.
- The header shows "System OK", "Check system", "Low battery", or
  "Shutting down".

The page refreshes every 3 s; Refresh at the bottom updates it at once. Topic
rates are measured only while the page is open, so they read "measuring" for
the first few seconds. Health
refreshes every 60 s, since the cell voltage changes over weeks.

## Lab dashboards

Four browser-based labs in `scripts/dashboards/`, installed as `racecar-*`
systemd units and opened from the [web dashboard](#web-dashboard). Internals:
[docs/dashboards.md](docs/dashboards.md).

| Dashboard | Port | Reads | Publishes |
|---|---|---|---|
| `webteleop` | 8081 | `/camera/color`, `/camera/depth`, `/scan`, `/odom`, `/edgetpu/inference` | `/drive` |
| `linefollow` | 8082 | `/camera/color`, `/odom` | `/drive` |
| `wallfollow` | 8083 | `/scan`, `/odom` | `/drive` |
| `camlabel` | 8085 | `/camera/color` | nothing; saves frames and labels to `datasets/camlabel/` under the [data root](#car-data) |

Install (also runs as phase 12 of `setup_all.sh`):

```bash
racecar setup dashboards          # install the units, stopped and disabled
```

The install also stops, disables and removes the units of the three retired
dashboards (`eps`, `pursuit`, `smartfollow`) if a car still has them.

`webteleop`, `linefollow` and `wallfollow` publish `/drive`, so only one runs
at a time. `camlabel` runs alongside any of them.

Run one:

```bash
racecar service start wallfollow   # stops the other /drive publishers first
racecar service status             # core and dashboards, with URLs
racecar service logs wallfollow
racecar service stop wallfollow
```

**One at a time.** All three publish `/drive`, and a second publisher fights the
mux, so `racecar service start` stops the others before starting the one you
asked for. Units install disabled; `racecar service enable <name>` makes one
survive a reboot, and bare `racecar service enable` covers only the core five.

**Safety.** `mux_node` forwards `/drive` only while the RB bumper is held, and
zeroes the output when `/joy` or the active source goes stale. See
[Autonomy gate](#autonomy-gate).

**Tuning.** Save in the page writes `dashboards/<name>.yaml` in the
[car profile](#car-profile); the shipped `<name>.yaml` keeps the defaults. Expect to
retune `max_mps`, `kp`, `kd`, `lookahead` and the `linefollow` HSV thresholds
per car. Logs go to `dashboards/<name>/` under the [data root](#car-data).

## Autonomy gate

The gamepad bumpers govern: LB for manual, RB for autonomy, neither or both
for idle. The safety override on the 8080 page lets `/drive` through with no
gamepad. An optional FlySky transmitter can hold the gate instead; see
[FlySky transmitter](#flysky-transmitter).

## Low-battery shutdown

`racecar-battery-guard.service` watches the battery from boot and turns the
car off safely before the battery runs flat. It runs on its own, apart from
teleop.

| When | What happens |
|---|---|
| 15 minutes left | A red line on every terminal and in the tmux status bar: "Low battery: 14 min left. Charge or swap the battery." A banner on 8080 and every app page. LOW, BATT, and "!" on the dot matrix. The LED strip flashes orange, then red. |
| Battery at 7.1 V (3.55 V per cell) | A 60 s countdown: "Shutting down in 30 s." on terminals and pages, the seconds left on the dot matrix from 20 s, faster LED flashing. |
| 0 s | "Turning the car off now. Turn off the power switch." SHUT and DOWN on the dot matrix. The guard stops a `racecar log` recording, the driving apps, autonomy (saving a map being built), and teleop, then powers off. |

After a shutdown, turn off the power switch. The PIT board and the halted Pi
still draw about 100 to 200 mA until the switch is off, so the battery keeps
draining. Do not leave the car unattended on battery.

The time left comes from a measured discharge curve of the Gens ace 5000 mAh
2S battery and the current over the last 5 minutes. The PIT firmware's LED
charge bar uses the same curve. The guard reads the cell count at boot and
measures the capacity of the battery in the car as it runs. A shutdown at
5 minutes left is built in but off (`shutdown_enabled: false`); the
15-minute warning still shows.

- Status: `/battery/estimate` (`diagnostic_msgs/DiagnosticStatus`, latched,
  1 Hz). `journalctl -u racecar-battery-guard -f` shows the warning,
  countdown, and each shutdown step.
- Log: `<data root>/health/battery/<start>_guard.csv`, one line a second,
  saved line by line.
- Settings: `config/battery.yaml`; see
  [docs/advanced-settings.md](./docs/advanced-settings.md), "Battery guard".
- Bench test without powering off: stop the service, run
  `python3 scripts/battery_guard.py --dry-run --floor-cell-volts 4.3` from the
  driver folder, then start the service again. The countdown starts about
  25 s in and the shutdown steps are logged, not run.

## Car data

Everything the car produces goes under one folder, the data root:
`/mnt/nvme/data` when the NVMe is mounted at `/mnt/nvme` and that folder can
be written, else `~/data` on the SD card. Settings never go there; they stay
in the [car profile](#car-profile).

| Folder | Holds |
|---|---|
| `bags/` | `racecar log` recordings |
| `images/` | image captures |
| `datasets/` | datasets; `datasets/camlabel/` from the camera label dashboard |
| `maps/` | saved maps (`racecar map`) |
| `slam_eval/` | SLAM validation records |
| `dashboards/<name>/` | lab dashboard logs |
| `health/` | health reports |

```sh
racecar data          # data root, NVMe or SD card, free space, a warning on fallback
racecar data path     # the path only, for scripts
racecar disk          # model, serial, size, partitions, fstab line, health, data root
racecar disk setup    # set up the NVMe at /mnt/nvme
racecar disk format   # erase the NVMe; asks for its serial
racecar disk check    # unmount, e2fsck -f -p, mount again
```

`racecar disk setup` on a blank drive makes one ext4 partition labeled
`racecar-data` and adds an fstab line by UUID with `nofail` and a 45 s device
timeout. It mounts the drive at `/mnt/nvme` and gives you `data/` on it. A
drive mounted at `/data` moves to `/mnt/nvme` with nothing erased, and its
top-level bags move into `data/bags`. Setup then offers to copy `~/data` to
the new root and leaves `~/data` in place. It points `~/jupyter_ws/data` and
`~/jupyter_ws/maps` at the new root, then asks for a reboot. It refuses the
system drive, a drive holding other data, and a blank-looking drive with
filesystem signatures; `racecar disk format` handles those. Format shows the
drive and its contents and asks you to type its serial. It refuses while
mapping or recording, and stops the services that write data. `setup_all.sh`
never formats; it prints `racecar disk setup` when an NVMe is present but not
set up. An existing car with the drive at `/data` runs `racecar disk setup`
once.

`racecar-data.service` works out the root once at boot and writes it to
`/run/racecar/data_root`. Services, login shells
(`/etc/profile.d/racecar-data.sh`) and the `racecar` shell tool set
`RACECAR_DATA_DIR` from it. Python code calls
`racecar_neo_ros2_driver.data_root.data_dir(sub)`; the student library has
`rc_utils.get_data_dir(subfolder)`.

A new session (`racecar log start`, `racecar map start`, `load`, `resume`)
refuses when the data root cannot be written, as when the drive drops out
after boot. `racecar status` has a `data root` row: WARN when data falls back
to the SD card while an NVMe is present, or at 80% used or under 10 GB free.
See [docs/troubleshooting.md](docs/troubleshooting.md), "Data drive missing at
boot".

## Bag recording

```bash
racecar log start lap3                     # bag: <timestamp>_lap3
racecar log start lap3 --topics /scan /odom
racecar log status                         # size, rate, time until full
racecar log stop                           # SIGINT, so rosbag2 finalizes
racecar log list
racecar log analyze                        # newest bag: duration, per-topic rates
racecar log config --storage=mcap        # saved defaults; --dir sets a fixed bag folder
```

Bags go to `bags/` under the [data root](#car-data). `--dir` and
`RACECAR_LOG_DIR` override it. `racecar log` names an NVMe that is present
but not set up rather than falling back without a word.

- For your own recordings, give a path under the data root:
  `ros2 bag record -o "$RACECAR_DATA_DIR/bags/my_run" /scan /odom`.
- To copy a recording to your computer, run this on your computer:
  `rsync -av racecar@<ip>:/mnt/nvme/data/bags/my_run .`

Recording every topic pulls in `/camera/color` and `/camera/depth`, roughly
73 MB/s against a sustained SD write near 30 MB/s. `start` refuses on an SD card
unless you pass `--force`, because rosbag2 drops messages rather than blocking:
the bag looks healthy until it is read. Name the topics you need, or
`--exclude '/camera/.*'`.

## Maps

`racecar map` builds a 2D map of a room or hallway from the lidar and saves it
under `maps/NAME/` in the [data root](#car-data). A saved map can be loaded later, and the car then
reports its position on it. Teleop must be running (`racecar service start
teleop`). You drive the car with the gamepad as usual. While mapping, the
speed is held at the firmware minimum of 0.5 m/s.
The same steps run from a phone or laptop in the [map viewer](#map-viewer).

```sh
racecar map start room_1                    # begin mapping
racecar map status                          # mode, map, localized, pose
racecar map save                            # returns when the files are written
racecar map stop
racecar map resume                          # start the last run again
racecar map list                            # maps; unsaved ones marked
racecar map info room_1                     # size, dates, poses
racecar map load room_1                     # localize; starts from the last pose on it
racecar map load room_1 --pose -1.5,2,1.57  # or from a pose: x, y in meters, heading in radians
racecar map load room_1 --global            # or search the whole map while driving
racecar map start room_2 --from room_1      # extend room_1 into a new map
racecar map recover room_1                  # keep the autosave of a run stopped without save
racecar map foxglove on                     # foxglove_bridge on port 8765 from the next start
```

To build a map:

1. Clear chairs, bags, and cables at about 20 cm above the floor (the lidar's
   height) out of the space, if you want clean walls.
2. Place the car where the map should begin.
3. Run `racecar map start NAME`. A name uses letters, digits, `-` and `_`.
4. Drive slowly through the space with wide, gentle turns. Sharp turns blur
   the walls.
5. Drive each area once or twice, not back and forth many times.
6. Drive back to where you started.
7. Run `racecar map save`.
8. Run `racecar map stop`.

The map is also autosaved every 60 s and on stop. A run stopped without
`save` is marked `unsaved` in `racecar map list`. `racecar map recover NAME`
keeps it.

To localize on a saved map:

1. Run `racecar map load NAME`.
2. Check that `racecar map status` reports `localized  true`.
3. If it reports `false`, drive a short distance and check again.

`localized` needs both a small pose spread from the localizer and the live
scan to fit the map (`scan_fit`, the share of lidar points within 0.1 m of a
wall). A car picked up and set down elsewhere reports `false` within a few
seconds; give its pose with `--pose`, or use `--global` and drive.
[docs/troubleshooting.md](docs/troubleshooting.md), "Localization state",
lists the causes of `false`.

Without `--pose` or `--global`, `load` starts from the car's last pose on that
map. A map that was never loaded starts from where mapping began. If the car
has moved since, give its pose with `--pose`, or use `--global` and drive
until `localized` turns true.

To extend a map, load it, drive to the edge of the mapped area, and run
`racecar map start NEW --from NAME`. `racecar map start NAME --from NAME`
extends it in place. A start on an existing name needs `--force`, which moves
the old folder to `maps/replaced/`. A save that replaces a map's files
moves the old ones to `NAME/previous/`. The tools never delete anything
under `maps/`.

Each map folder holds `NAME.pgm` (the image) and `NAME.yaml` for viewing and
AMCL, `NAME.posegraph` and `NAME.data` for slam_toolbox, and `meta.yaml`
(date, driver version, frame offsets, start pose). `RACECAR_MAP_ROOT`
overrides the folder. `scripts/setup_jupyter.sh` links `~/jupyter_ws/data` to
the data root and `~/jupyter_ws/maps` to its `maps/`, so JupyterLab shows
them.

### Map viewer

`racecar-mapview.service` serves a map page on port 8084 from boot,
independent of teleop: `http://<robot>:8084` on a phone or laptop on the
car's network. It shows the live map with the car's pose and path, the
lidar scan, mode, localized, scan fit, and a warning when the Pi is
throttled or hot. Its buttons run the same `racecar map` commands, and the
page shows their output:

| Button | Does |
|---|---|
| Start mapping | `racecar map start NAME` |
| Save, Stop | `racecar map save`, `racecar map stop` |
| Maps | Lists the saved maps with thumbnails; Load, Extend, and Keep (on an unsaved map) |
| Extend | Shows the saved map; drag from the car's position toward the way it faces, then `racecar map start NEW --from NAME --pose ...` |
| Set pose | Drag on the map from where the car is toward the way it faces |
| Find me | Searches the whole map (AMCL only); drive slowly until `localized` |
| Clear trail | Clears the car's path on every open map page |

The page never drives the car. Start, stop, and load restart the autonomy
service and ask first. With no browser open it holds no ROS subscriptions
and uses about 0.1% of a core; with one open, about 6%.

`racecar map foxglove on` adds `foxglove_bridge` for Lichtblick from the next
start or load; see [docs/advanced-settings.md](docs/advanced-settings.md),
"Foxglove bridge".

## Jupyter notebooks

`http://<robot>:8888/lab`: JupyterLab with `import rclpy` working out of the box. Notebooks land in `~/jupyter_ws/`. No token / password by default (the systemd unit assumes the robot's network is trusted).

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
racecar launch realsense                # individual nodes too: RealSense D435i (color + depth + IMU)
racecar launch imu_fusion
racecar launch lidar
racecar launch edgetpu
racecar launch dotmatrix
racecar launch autonomy                 # SLAM beside teleop: mapping by default
racecar launch autonomy mode:=localization map:=NAME   # on maps/NAME in the data root; localizer:=slam_toolbox, pose:=x,y,yaw
```

`autonomy.launch.py` needs teleop running.

RealSense topics, profiles, and known issues: see [docs/specifics/realsense_topics.md](docs/specifics/realsense_topics.md).

For boot-time startup, see [scripts/](./scripts/) for systemd units and the `setup_all.sh` idempotent installer.

## Sensor calibration

Bias and scale are per-board, so each car needs its own calibration run. The
committed `config/lsm9ds1_cal.yaml`, `config/lsm9ds1_mag_cal.yaml`,
`config/realsense_cal.yaml` and `config/frames.yaml` hold the defaults; a car's
own values go in `params/<name>.yaml` in the [car profile](#car-profile),
which the launch files load after the committed file.

```sh
racecar calibrate status      # profile path, what is calibrated, when, stored values
racecar calibrate             # every step but dynamics, in order, one guided session
racecar calibrate imu lidar   # chosen steps only
racecar calibrate check       # verify each calibration against the live sensors
```

Car prep is three commands: `racecar calibrate`, a teleop restart (offered at
the end), then `racecar calibrate check`. At each check's prompt, Enter runs
it, `s` skips it, `b` goes back to the previous check, and `q` stops. A summary at the end lists every criterion of every check with the
measured value, the limit, and PASS, WARN, or FAIL (green, yellow, red on a
terminal, as in `racecar status`; `NO_COLOR` turns it off), with what to do
under each WARN or FAIL; steps skipped or not reached are listed too. The command exits 1
if any check failed. One line per prompt on stdin scripts the whole run.

| Check | Car position | Passes when |
|---|---|---|
| `imu` | Still and level, 3 s | `/imu/lsm9ds1` gravity within 1% of 9.807 m/s^2; gyro under 0.01 rad/s |
| `mag` | One full turn flat on the floor, 25 s, away from large metal | `/mag` traces a circle centered on the turning axis (within 10% of its radius) and round to 5%. Field strength outside 20 to 70 uT is a WARN: a uniform scale error leaves a compass heading unchanged (see docs/troubleshooting.md on `imu.mag_scale`). `/mag/raw` is fitted the same way for contrast |
| `realsense` | Still and level, 3 s | the stored calibration applied to `/imu/realsense` gives gravity within 1%, gyro under 0.01 rad/s |
| `lidar` | Driven forward and back about 1.3 m, parked 2 s before, between, and after | the lidar yaw from the direction of travel within 0.5 deg of `laser_yaw_deg` (warn to 1); each scan match under 30 mm RMS with at least 30% of points matched |
| `odom` | Driven 2 circles and stopped anywhere near the start, then driven straight toward a wall and stopped anywhere short of it; parked 2 s before and after each run | `/odom/integrated` heading within 1% of the lidar's (warn to 3%); distance within 2% of the lidar's (warn to 5%); each scan match under 30 mm RMS with at least 30% of points matched; the scales `pit_node` runs with match `params/odom.yaml` |
| `steering` | Driven straight at steering 0 about 2 m, parked 2 s before and after | heading drift within 1 deg/m (warn to 5, naming servo and linkage play); the trim `throttle_node` runs with matches `params/throttle.yaml` |

`racecar calibrate` runs the tools in `scripts/calibration/` in this order:
the hand-held steps first (`imu`, `mag`, `realsense`), then `lidar`, `odom`
and `steering` with the car on the floor. `dynamics` measures the car for the
simulator and runs only when named (`racecar calibrate dynamics`); it has no
check.
Each step can be run, skipped (`s`), or gone back from (`b`, to run the previous
step again) at its prompt. Within a tool, every run, pose, or axis offers `b`
the same way to redo the previous one (after releasing RB too early, or
hitting something), and a run that was stopped or failed its measurement is
offered again. At the end it
prints old and new values and offers to restart teleop so the nodes load
them. Run on their own, the tools print the file written and the restart
command instead. Teleop must be running, since the tools read its topics. The tools
also run on their own:

```sh
ros2 run racecar_neo_ros2_driver calibrate_imu.py         # LSM9DS1 accel bias + scale, gyro bias
ros2 run racecar_neo_ros2_driver calibrate_mag.py         # LSM9DS1 hard/soft iron
ros2 run racecar_neo_ros2_driver calibrate_realsense_imu.py   # D435i accel bias + scale, gyro bias
ros2 run racecar_neo_ros2_driver calibrate_lidar.py       # lidar yaw from the direction of travel; the car drives itself while RB is held
ros2 run racecar_neo_ros2_driver calibrate_odom.py        # odometry against the lidar: circles, then a straight run
ros2 run racecar_neo_ros2_driver calibrate_steering.py    # steering trim against the lidar; the car drives itself while RB is held
ros2 run racecar_neo_ros2_driver calibrate_dynamics.py    # simulator dynamics; the car drives itself while RB is held
racecar service restart teleop                            # nodes read the values at start
```

| Utility | Reads | Writes | Consumed by |
|---|---|---|---|
| `calibrate_imu.py` | `/imu/lsm9ds1/raw` | `params/lsm9ds1_cal.yaml` | `pit_node` |
| `calibrate_mag.py` | `/mag/raw` | `params/lsm9ds1_mag_cal.yaml` | `pit_node` |
| `calibrate_realsense_imu.py` | `/imu/realsense` | `params/realsense_cal.yaml` | `imu_fusion_node` |
| `calibrate_lidar.py` | `/scan`, `/odom/integrated`, `/motor` (recorded); `/joy` (RB) | `params/frames.yaml` (`laser_yaw_deg`) | `frames.launch.py` |
| `calibrate_odom.py` | `/imu/lsm9ds1`, `/odom/integrated`, `/scan` | `params/odom.yaml` (`odometry.yaw_rate_scale`, `odometry.speed_scale`) | `pit_node` |
| `calibrate_steering.py` | `/scan`, `/odom/integrated`, `/motor` (recorded); `/joy` (RB) | `params/throttle.yaml` (`steering_trim`) | `throttle_node` |
| `calibrate_dynamics.py` | `/scan`, `/imu/lsm9ds1`, `/odom/integrated`, `/motor` (recorded); `/joy` (RB) | `params/dynamics.yaml` | the simulator (no node reads it) |

Each utility writes the file to the car profile; launch reads it at start,
so a teleop restart applies it with no rebuild. A replaced file is kept under
`history/`. It writes nothing and exits non-zero when the fit fails or it has too little
data.

`calibrate_lidar.py` measures the lidar yaw from the direction the car
travels. From each of three placements the car drives itself while RB is
held: forward 2 s, a pause, then back 2 s, with scans at rest before,
between, and after. `base_link` moves without sideslip, so its chord points
the way `/odom/integrated` says it went (the gyro heading along the path,
which also covers a path whose curvature changes); the scan match gives the
laser's motion in its own frame, and the one laser yaw that lines the two up
is the estimate. Forward legs scatter about 0.14 deg; reverse legs scatter
four times as much (reversing with front steering wanders), so they return
the car and are reported but not counted. The mean of the forward legs is
written as `laser_yaw_deg` when its standard error is 0.15 deg or less;
otherwise the tool asks for another placement, up to six. A leg that does
not move (no reverse from rest, or RB released) is reported. `--analyze
BAG ...` recomputes from recorded placements and writes the same way.
`--check` drives one placement and passes
within 0.5 deg of the configured yaw (warns to 1). `--wall` keeps the older
method: the car squared by hand to a flat wall 0.5 to 2 m ahead (both front
wheel centers the same distance from it), a line fit that should be
perpendicular to x in `base_link`; hand squaring is good to about 1 deg.
`--wall --check` reports that fit without writing, and `--wall-distance-mm`
takes a taped distance from `base_link` to the wall and reports the lidar x
error it implies.

`calibrate_odom.py` has two steps, each skippable, and uses the lidar as the
reference. Each run starts and ends with the car parked for 2 s while the
tool records a scan; scan matching (`scripts/calibration/scan_match.py`,
point-to-line ICP) gives the car's true motion between the two scans, so the
car may stop anywhere. Gyro scale: the car drives 3 circles and stops near
the start. The scan match gives the heading change modulo one turn and the
gyro gives the whole-turn count. A scale outside 0.8 to 1.25 is refused,
since it means `imu.gyro_scale` is wrong (a deg/s-for-rad/s mix-up reads
57x). Speed scale: the car drives at least 2 m straight toward a wall or
furniture 3 to 5 m away. Both steps compare `/odom/integrated` with the
lidar too, and warn when the odometry disagrees with the gyro it integrates.
The scene must stay still between the two scans, and a room works better
than a long corridor. Each step keeps the other's value in
`params/odom.yaml`.

`calibrate_steering.py` sets the steering trim, the offset `throttle_node`
adds to every steering command so the car drives straight at steering 0. It
drives seven short runs by itself while RB is held, one per steering command
(0, +/-0.25, +/-0.5, +/-1.0), each needing about 3 by 3 m of clear floor. In
each run the wheels are set with the car still and the steering never
changes, so the car moves on one arc; the lidar scans at rest before and
after give its chord and heading change, and curvature = 2 sin(heading / 2) /
chord, with no encoder distance. The command that drives straight is the
drift at 0 divided by the slope near center (from +/-0.25; at +/-0.5 the
curvature already flattens toward full lock). The new trim is the old one
plus that command; a change under 0.01 (about 5 us of servo pulse, below what
the servo resolves) is not written, and beyond +/-0.2 is refused. It also
prints the command-to-curvature table (radius and bicycle wheel angle per
command). The same runs give the encoder's over-read in turns: with no center
differential the encoder reads the mean of the rear and front axle speeds,
k (1/cos delta - 1) too high, delta the bicycle steering angle. The step fits
k (0.5 for a locked center shaft) against its own straight runs and prints
it; it does not write it. With `odometry.turn_gain` set to k in
`params/odom.yaml`, `pit_node` divides the encoder speed by
1 + k (1/cos delta - 1), with delta from the gyro (atan(wheelbase x yaw
rate / speed), `odometry.wheelbase_m`), above 0.1 m/s. The default 0 turns
the correction off.
`--check` drives straight at command 0: drift within 1 deg/m passes. Up to
5 deg/m warns, and the warning names servo and linkage play, which alone
moves the drift by up to about 3 deg/m between runs, so recalibrate only if
it stays. Beyond 5 deg/m fails as a mechanical fault (servo horn or
linkage). The trim `throttle_node` runs with must match the file.

`calibrate_dynamics.py` measures the car for the simulator. It drives five
fixed runs by itself while RB is held, and releasing RB while it drives stops
the car: straight at speed 0.25, 0.5 and 1.0 (about 3.5 m of clear floor),
then right-turn and left-turn circles at full lock at 0.25, each after 0.8 s
straight (about 2.3 by 1.6 m of clear floor). Each run is recorded with
`ros2 bag` under the `racecar log` directory and analyzed: steady speed, time
to 90% of it, and stop distance and time after the command goes to 0 (from
`/odom/integrated`, checked against the lidar between the rest scans); the
rear-axle turning radius per side from chained lidar scan matches, the
Ackermann inner and outer wheel angles (230 mm wheelbase, 178 mm track), the
outer front tire track diameter, and the encoder's over-read in turns; and the
center-to-lock steering time per side from the yaw rate, an upper bound on
the servo, summed for lock to lock. Speeds are normalized commands; their m/s
depends on the firmware ceiling (`config/throttle.yaml`). `--analyze BAG ...`
re-analyzes recorded runs.

`calibrate_imu.py` and `calibrate_realsense_imu.py` walk a 6-position sequence
(each axis up and down). For each accelerometer axis, the readings up and down
give its bias, (up + down) / 2, and its scale, 2 g / (up - down); the node
applies (raw - bias) * scale. A position held wrong shows as a scale outside
0.8 to 1.25, and the tool refuses to write. `calibrate_mag.py` needs rotation about
all three axes and fits an ellipsoid, then plots raw against corrected samples
so you can confirm the sphere closed up.

The values live in the car profile, so a driver update or re-clone keeps
them. Before a fresh image or SD card swap, save them with
`racecar profile export FILE`; see [Car profile](#car-profile).

A car that has never been calibrated runs with zero bias, unit scales and an
identity soft-iron matrix, and nothing is logged to say so. On every new car,
run the full `racecar calibrate`, restart teleop, then run
`racecar calibrate check`. ROS ignores a parameter file whose top-level key
names no running node, without warning; if a calibration appears to have no
effect, check that the key matches the node the launch file starts.

## Car profile

The car profile holds this car's settings outside the driver folder, so a
driver update, re-clone, or delete leaves them in place. It lives in
`~/.config/racecar` (`RACECAR_CONFIG_DIR`, else `$XDG_CONFIG_HOME/racecar`).

| File | Holds |
|---|---|
| `car.yaml` | car name, profile version |
| `params/<name>.yaml` | ROS parameter overrides for `config/<name>.yaml`; calibration writes here |
| `dashboards/<name>.yaml` | lab dashboard tuning |
| `autonomy.conf`, `features.conf`, `networking.env` | autonomy mode, camera and detections choice, network settings |
| `history/<stamp>/` | the previous copy of each file a write replaced |

Launch reads `params/` at start, so a new value applies on the next
`racecar service restart teleop` with no `racecar build`. Per-car YAML left in
the driver folder (`config/*.local.yaml`) is not read; `racecar status` and
the teleop log warn about it.

```sh
racecar profile               # path, car name, files and dates, unused driver-folder files
racecar profile path          # the profile folder only
racecar profile migrate       # move per-car files from the driver folder into the profile
racecar profile export FILE   # write the profile to one .tar.gz
racecar profile import FILE   # read it back; replaced files go to history/
racecar profile check         # warn about each unused driver-folder file
```

`migrate` keeps the driver-folder copy when both exist and moves the profile
copy to `history/`. A `calibration/` folder in the profile is reported where
it differs and moved to `history/`. `setup_all.sh` runs it after the build. An existing car
runs `racecar profile migrate` once after updating the driver. Use `export`
and `import` for an SD card swap or a fresh image.

## RTC backup cell

The Pi 5 keeps its clock running across power loss from a coin cell on the
board's RTC connector. Charging of that cell is **disabled by default**, so
without the setting below it drains until the clock resets on every power cut
and `racecar test` fails `TestRTC`.

`setup_raspi_config.sh` writes:

```
dtparam=rtc_bbat_vchg=3000000
```

3.0 V suits the official Raspberry Pi RTC Battery (an ML2032). The setting
takes effect on the next boot; check it with:

```sh
cat /sys/class/rtc/rtc0/charging_voltage    # expect 3000000
cat /sys/class/rtc/rtc0/battery_voltage     # climbs over the following days
```

Only enable charging for a **rechargeable** cell. Forcing charge current into a
primary CR2032 can make it vent or leak. If a car has a non-rechargeable cell
fitted, run `RTC_VCHG_UV=0 bash scripts/setup_raspi_config.sh`: it removes the
`dtparam=rtc_bbat_vchg=` line, so charging is off from the next boot. Swap the
cell before enabling it again.

## Bootloader EEPROM

`setup_raspi_config.sh` reconciles four bootloader settings, writing only when
one differs and leaving any other key untouched:

| Setting | Value | Reason |
|---|---|---|
| `PSU_MAX_CURRENT` | `5000` | Raises the total USB peripheral budget from 600 mA to 1.6 A |
| `POWER_OFF_ON_HALT` | `1` | `shutdown` cuts power instead of idling |
| `BOOT_UART` | `1` | Bootloader diagnostics on the UART |
| `BOOT_ORDER` | `0xf461` | SD, then NVMe, then USB, then repeat |

`PSU_MAX_CURRENT` is the one that matters most. The Pi 5 learns what its supply
can deliver by negotiating over USB-PD, and a car powered from a BEC on the 5 V
rail never negotiates at all. Left alone the firmware assumes a 3 A supply and
caps *total* USB peripheral current at 600 mA, which is not enough for the
RealSense D435i, the lidar, and the ALFA dongle together. Symptoms are
peripherals failing to enumerate or dropping out under load, which reads like a
hardware fault.

Confirm the negotiation actually came up empty on a given car with:

```sh
od -An -tx4 --endian=big /proc/device-tree/chosen/power/usbpd_power_data_objects
od -An -tu4 --endian=big /proc/device-tree/chosen/power/max_current
```

All-zero PD objects mean no negotiation happened; `max_current` should still
read `5000` because the EEPROM forced it.

Changes take effect on the next boot. `RACECAR_EEPROM=0` skips the step on a
car not yet configured; it does not undo keys already written (use
`sudo rpi-eeprom-config --edit` for that).

## ROS discovery scope

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` is set in `launch_teleop.sh`, the
dashboard / watchdog / jupyter units, the lab-dashboard units rendered by
`setup_dashboards.sh`, and the `.bashrc` block written by `setup_user_env.sh`.
Every node in this stack runs on the robot, so restricting discovery to the
loopback interface costs nothing on-board and keeps discovery chatter off the
ALFA dongle, where it was driving CPU spikes.

The tradeoff: a laptop cannot see the robot's topics. `rviz`,
`ros2 topic echo`, and remote nodes will find nothing. For a session where you
need them, widen the range on both machines:

```sh
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

## FlySky transmitter

A FlySky transmitter is an optional accessory. It can hold the drive gate
instead of the gamepad bumpers; that is off by default:

```yaml
# ~/.config/racecar/params/mux.yaml  (car profile, loaded after config/mux.yaml)
mux_node:
  ros__parameters:
    rc_authority_enable: true
```

With it enabled and a live transmitter, channel 6 (switch B) selects the mode:
middle idle, up manual (the USB gamepad still drives; the transmitter is a gate,
not a drive source), down autonomous. The bumpers are ignored while the
transmitter holds the gate. With no transmitter, nothing changes.

Authority is granted only when the link is fresh, every channel is inside the
valid pulse band, it has held continuously for `rc_link_hold_sec`, and the
switch has been seen at middle once since the grant. Any one of those failing
revokes it immediately and hands the gate back to the bumpers.

**Confirm on the bench before enabling.** Which physical switch lands on channel
6 depends on the transmitter's channel assignment, up and down may be mirrored
from what this assumes, and a receiver configured to emit failsafe values with
the transmitter off would look like a live link. Wheels off the ground.

**Mapping.** The mapping speed limit applies to the gamepad and the student
library. The transmitter drives the Teensy directly and is not limited, so
switch it to slow mode before mapping.

## Changelog

Full history in [docs/changelog.md](./docs/changelog.md). Most recent:

- **Unreleased**: low-battery warning and safe shutdown
  (`racecar-battery-guard.service`: time left, a red line on terminals,
  banners on 8080 and the app pages, LED strip and dot-matrix alerts, a
  60 s countdown at 7.1 V); battery time and power in the 8080 header; a
  Turn Off tile; `speed_cap` in `throttle.yaml` clips speed for every
  source (1.0, off, by default).
- **0.9.0** (2026-10-24): 2D SLAM beside teleop (`racecar map` to build, save,
  load, extend, and recover maps through `racecar-autonomy.service`;
  `racecar launch autonomy`: slam_toolbox mapping, AMCL or slam_toolbox
  localization; maps in `~/data/maps`); a web map viewer on port 8084
  (`racecar-mapview.service`: live map, pose, scan, the `racecar map`
  controls, set pose, find me); `localized` also needs the live scan to fit
  the map, so a car carried elsewhere reports not localized; autonomy in
  `racecar status`, `racecar service`, and the 8080 dashboard; a Lichtblick
  layout (`config/foxglove_layout.json`); a mapping steering cap, off by
  default; integrated odometry in `pit_node` (`odom -> base_link`,
  `/odom/integrated`); an apps page on 8080 with a live sensor overview
  (camera label back) and the map viewer in one shared style; a
  safety-controller override on 8080 to drive with no controller;
  Features on 8080 to turn the camera, detections (off at boot), and
  autonomy on and off; recordings kept on the NVMe; telemetry stamped with the Teensy
  sample time;
  `racecar calibrate` (guided calibration, status, backup,
  `check` against the live sensors including a magnetometer turn test and
  odometry calibration and check against the lidar (scan matching); rewritten
  step instructions;
  tools moved to `scripts/calibration/`); accelerometer scale for both IMUs;
  watchdog restart policy (startup grace, backoff, give-up,
  state file shown by `racecar status` and the dashboard); static frame tree (`base_link`, `laser`, `imu_link`,
  `camera_link`) from `urdf/racecar_neo.urdf.xacro` and `config/frames.yaml`,
  started by teleop; lidar yaw from the direction of travel
  (`calibrate_lidar.py`); steering trim against the lidar; simulator dynamics
  (`racecar calibrate dynamics`, run only when named); an encoder correction
  for turns (`odometry.turn_gain`, off by default);
  `racecar test` runs the pytest suite again (it ran none since 0.8.2);
  troubleshooting guide documents the 5 V brownout root cause
  (an intermittent buck to Pi 5 V wire, replaced 2026-10-03) and a
  post-repair check; the BEC bulk capacitor stays a known cause on other cars;
  American English spelling throughout.
- **0.8.3** (2026-09-22): `racecar status` exits non-zero only on a FAIL, with
  `--strict` for the old rule; slow-but-delivering streams and dual-mode eth0
  warn instead of failing; each sensor row shows a sample of its data; a
  CPU busy share and ARM clock replace the load average; OK, WARN and FAIL
  are colored on a terminal.
- **0.8.2** (2026-09-22): cleanup release. `racecar lint` with pinned ruff,
  black and mypy; per-car calibration in gitignored `config/*.local.yaml`, which
  `pit.launch.py` now loads; `calibrate_mag.py` no longer overwrites a good
  calibration on a failed fit; `RTC_VCHG_UV=0` turns RTC charging off;
  retired dashboard units removed; `racecar clear` removed.
- **0.8.1** (2026-09-10): dashboards moved to ports 8081 to 8083;
  EfficientDet-Lite0 on COCO as the default detection model, with inference
  capped at 15 Hz; a detections overlay and the depth
  stream on `webteleop`, whose lidar view was 180 degrees out; the dashboard
  palette led by orange and split by contrast; each dashboard checkout carries
  a `VERSION` this driver pins and `setup_dashboards.sh` verifies.
- **0.8.0** (2026-09-06): seven lab dashboards as `racecar-*` units, a
  transmitter-held autonomy gate, bag recording behind `racecar log`, `/odom`
  and `/rc/link` from `pit_node`.
- **0.7.4** (2026-09-06): eth0 holds one IPv4 addressing mode, never both, via
  `racecar eth`; new `racecar wifi` and `racecar desktop`; `racecar status` is a
  strict whole-car diagnostic; `racecar selftest` removed. See
  [docs/advanced-settings.md](docs/advanced-settings.md) for the non-defaults.
- **0.7.3** (2026-09-05): LSM9DS1 and RealSense calibration utilities plus raw
  `/imu/lsm9ds1/raw` and `/mag/raw` telemetry; dashboard CPU roughly halved via
  raw subscriptions and `/diagnostics`-sourced camera rates; ROS discovery
  restricted to localhost; RealSense defaults to 640x480 at 30 fps depth and
  60 fps color; ESC direction corrected; flake8 and pep257 backlog cleared.
- **0.7.2** (2026-07-07): eth0 static address reset loop fixed; the static is
  declared once via netplan `addresses:` with no gateway.
- **0.7.1** (2026-07-07): per-car SSID (`racecar-neo-<id>`) and an AP-disable
  reset for golden images.

## License

GPLv3; see [LICENSE](./LICENSE).
