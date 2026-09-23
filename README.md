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
- [Bag recording](#bag-recording)
- [Jupyter notebooks](#jupyter-notebooks)
- [Manual build](#manual-build)
- [Launch](#launch)
- [Sensor calibration](#sensor-calibration)
- [RTC backup cell](#rtc-backup-cell)
- [Bootloader EEPROM](#bootloader-eeprom)
- [ROS discovery scope](#ros-discovery-scope)
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
- `/imu/realsense` (sensor_msgs/Imu): RealSense D435i IMU, remapped from `/camera/imu`
- `/imu/lsm9ds1`, `/mag` (sensor_msgs/Imu, MagneticField): LSM9DS1 on the NEO-PIT board, republished from `pit_node` telemetry
- `/imu/lsm9ds1/raw`, `/mag/raw` (sensor_msgs/Imu, MagneticField): the same telemetry axis-remapped and sensitivity-scaled but with no bias applied; the calibration utilities fit against these
- `/imu/fused` (sensor_msgs/Imu): `imu_fusion_node` blends `/imu/realsense` and `/imu/lsm9ds1` (single-source passthrough when one is live)
- `/scan` (sensor_msgs/LaserScan)
- `/edgetpu/inference` (vision_msgs/Detection2DArray): `edgetpu_node` consumes `/camera/color`

Display node subscribes:
- `/dotmatrix/text` (std_msgs/String): renders user messages; falls back to a mode glyph (IDLE / MAN / AUTO) tied to the gamepad state

Safety and uptime layers:
- **Mux** enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- **Watchdog** (`scripts/watchdog.py`) supervises 7 nodes with two liveness signals (ROS topic + `pgrep` on the entry-point path) and restarts a node when either fails. 30 s restart cooldown, SIGTERM to SIGKILL escalation, FastRTPS SHM orphan sweep every 60 s, Pi 5 PMIC under-voltage alarm. Restart is skipped when the device is physically missing.
- **Four core systemd units** (`racecar-{teleop,watchdog,dashboard,jupyter}.service`) wired with `BindsTo=` so watchdog dies when teleop dies, and `Wants=` so watchdog auto-starts when teleop starts. Three more units carry the lab dashboards; they install disabled and are started one at a time.
- **Launch wrapper** (`scripts/launch_teleop.sh`) creates `~/logs/<timestamp>/`, updates `~/logs/latest` atomically, sweeps FastRTPS SHM orphans, and `exec`s `ros2 launch` so systemd tracks the launch PID directly.
- **Web dashboard** at `http://<robot>:8080`: 9 node cards, 9 topic-rate rows, System Health (RTC battery + Pi under-voltage alarm), watchdog log tail. Auto-refresh.
- **JupyterLab** at `http://<robot>:8888` with PYTHONPATH/AMENT_PREFIX_PATH pre-set so `import rclpy` works in notebooks.
- **`colcon test` suite** covering the nodes, scripts, setup phases and attached peripherals, with fix commands in the failure messages.

Node responsibilities, the full topic reference, launch composition, and the
calibration data flow are in [docs/architecture.md](./docs/architecture.md).
Failure modes that shaped the code, and the reasoning behind choices that look
arbitrary at the call site, are in
[docs/troubleshooting.md](./docs/troubleshooting.md).

## Quickstart (fresh Ubuntu 24.04 install)

Target: Raspberry Pi 5 running **Ubuntu Server 24.04 LTS for arm64** (Noble). ROS2 Jazzy is the only supported distro for this driver; older Ubuntu releases (22.04 Jammy) are **not** supported because Jazzy doesn't install there.

### 1. SD card or NVMe image

Use Raspberry Pi Imager -> *Other general-purpose OS* -> *Ubuntu* -> *Ubuntu Server 24.04 LTS (64-bit)*. Before writing, click the gear icon and pre-set:

- **Hostname**: `racecar-neo` (matches what the systemd services + dashboard expect)
- **Username**: `racecar` (the `racecar` shell tool, udev groups, and service unit `User=` are all hard-coded to this name; don't change it)
- **Password**: your choice
- **Wireless LAN**: your home/lab SSID. The Pi's built-in `wlan0` stays a client of this network; the AP set up later runs on a separate dongle.
- **SSH**: enabled, password auth

Boot the Pi, find its IP (`ip neigh` from another machine, or check your router), then `ssh racecar@<ip>`.

### 2. needrestart configuration

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

Largest single block of the install (~8-15 min on a fresh image at 10 MB/s). With needrestart silenced above, this runs hands-off.

### 4. Repository clone and setup run

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` is idempotent; re-running is safe (each phase checks for existing state and skips when already applied). Sudo password is prompted **once** at the top of the run and cached via a background keepalive for the remaining ~45 min; you can walk away after that prompt.

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

After reboot, `racecar-teleop.service` auto-starts and pulls the watchdog via `Wants=racecar-watchdog.service`. Verify:

```sh
racecar status              # full diagnostic; exits non-zero only on a FAIL
racecar service status      # the 4 core racecar-* units should be active+enabled
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

1. **`setup_ros2.sh`**: ROS2 Jazzy apt repo + message/driver packages
2. **`setup_dev_tools.sh`**: build tools, Python hardware libs (`smbus` / `serial` / `spidev`), and the pinned linters (`ruff`, `black`, `mypy`) per-user
3. **`setup_user_env.sh`**: joins `dialout` / `i2c` / `spi` / `gpio` / `video` groups; installs `/etc/polkit-1/rules.d/49-racecar-network.rules` so `racecar wifi` can control NetworkManager from a terminal; sources ROS2 + the `racecar` shell tool in `.bashrc` and puts `~/.local/bin` on `PATH`
4. **`setup_raspi_config.sh`**: boot-level configuration: enable I2C, enable SPI, disable serial console (frees the GPIO UART / `ttyAMA0` for the NEO-PIT link), enable RTC backup-cell trickle charging (`RTC_VCHG_UV=0` turns it off; see [RTC backup cell](#rtc-backup-cell)), and reconcile the bootloader EEPROM (`RACECAR_EEPROM=0` skips it; see [Bootloader EEPROM](#bootloader-eeprom))
5. **`setup_udev.sh`**: installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/neo-pit-pcb`, `/dev/lidar`)
6. **`setup_dotmatrix.sh`**: `pip install --user luma.led_matrix`, the font source for the `dotmatrix_node` rasterizer
7. **`setup_coral.sh`**: installs `libedgetpu1-std`, `tflite_runtime`, `pycoral` from vendored `depend/` artifacts; with the M.2 card present, also the gasket DKMS driver and the `coral-msi` overlay (reboot required)
8. **`setup_realsense.sh`**: installs `realsense2_camera` (apt) + the Pi 5 IMU IIO permission fix (script, udev rule, boot service)
9. **`setup_workspace.sh`**: clones `sllidar_ros2` and runs `colcon build --symlink-install`
10. **`setup_jupyter.sh`**: `pip install --user jupyterlab`, creates `~/jupyter_ws/`
11. **`setup_services.sh`**: installs and enables the four core systemd units (`racecar-{teleop,watchdog,dashboard,jupyter}.service`)
12. **`setup_dashboards.sh`**: clones or fast-forwards the three lab-dashboard checkouts into `scripts/dashboards/`, installs a stopped, disabled `racecar-*` unit for each, and removes units left by retired dashboards (`RACECAR_DASHBOARDS=0` skips it)

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

racecar service status      # active/enabled snapshot, core units and dashboards
racecar service install     # drop unit files in /etc/systemd/system/ + enable
racecar service start       # default: start teleop (watchdog follows via Wants=)
racecar service stop        # default: stop teleop (watchdog follows via BindsTo=)
racecar service logs teleop # journalctl -u racecar-teleop -f
racecar service start wallfollow  # a lab dashboard; stops the other /drive publishers
racecar service enable wallfollow # enable one unit; bare `enable` covers the core four

racecar setup all                       # run the 12-phase orchestrator
racecar setup networking --ssid=racecar-neo-2   # configure eth0 addressing + ALFA-dongle AP
racecar setup networking --show         # print persisted overrides
racecar setup dashboards                # clone the lab dashboards + install units
racecar setup realsense                 # offline D435i firmware flash

racecar udev                        # re-install the udev rules
racecar cleanup [--force]           # list / kill stale racecar processes + SHM orphans
racecar status                      # full diagnostic (devices, sensors, system, network)
racecar status --quick              # host checks only; skips the ROS sampling phase
racecar status --strict             # exit non-zero on any WARN or SKIP as well
racecar eth status                  # eth0 addressing mode + conflict checks
racecar wifi list                   # visible networks on wlan0
racecar desktop status              # GNOME on/off for the next boot
racecar log start lap3              # record a bag; racecar log stop to finalize
racecar log analyze                 # summarize the newest bag
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

Once `racecar-teleop.service` is running, browse to `http://<robot>:8080` for a live status page:

- **Nodes**: one card per monitored subsystem (9 total): green when the expected topic is being advertised, red when not. EdgeTPU and dot matrix, which the watchdog does not restart, show grey (`unsupervised`) instead of red.
- **System Health**: RTC backup battery voltage (green >= 3.0 V, yellow 2.7-3.0 V, red < 2.7 V) and the Pi 5 PMIC sticky under-voltage alarm.
- **Topic Rates**: live Hz for `/motor`, `/mux_out`, `/imu/fused`, `/imu/lsm9ds1`, `/scan`, `/edgetpu/inference`, `/camera/color`, `/camera/depth`, and `/imu/realsense`. Yellow when stale (< 0.5 Hz), red when missing. The three RealSense rows are read from the camera's own `/diagnostics` stream; the rest are counted from raw subscriptions, which keeps the dashboard's own CPU cost near 20%.
- **Watchdog Log**: tail of `~/logs/latest/watchdog.log` so you can see restart events.

Refreshes every 3 s; System Health refreshes every 60 s, since the cell voltage changes over weeks.

## Lab dashboards

Three browser-based labs, each a fork under the
[MITRacecarNeo](https://github.com/MITRacecarNeo) organization of the
corresponding [Neobotics
Foundation](https://github.com/Neobotics-Foundation-Inc) repository, installed
as `racecar-*` systemd units. The forks carry this platform's lidar convention,
ports and branding.

| Dashboard | Port | Reads | Publishes |
|---|---|---|---|
| `webteleop` | 8081 | `/camera/color`, `/camera/depth`, `/scan`, `/odom`, `/edgetpu/inference` | `/drive` |
| `linefollow` | 8082 | `/camera/color`, `/odom` | `/drive` |
| `wallfollow` | 8083 | `/scan`, `/odom` | `/drive` |

Install (also runs as phase 12 of `setup_all.sh`):

```bash
racecar setup dashboards          # clone or fast-forward, install units
racecar setup dashboards --update # pull only
racecar service update            # same, from the service subcommand
```

The install also stops, disables and removes the units of the four retired
dashboards (`camlabel`, `eps`, `pursuit`, `smartfollow`) if a car still has
them.

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
survive a reboot, and bare `racecar service enable` covers only the core four.

**Checkouts live in `scripts/dashboards/`**, gitignored, on the forks'
`racecar-neo` branch (`RACECAR_DASHBOARD_BRANCH` overrides). Updates are
`git pull --ff-only`, so a car's tuned `wallfollow.yaml` survives. Each unit is
rendered from its checkout's `.service.in` template.

**Versions.** Each checkout carries a `VERSION` tracking this driver's release.
`setup_dashboards.sh` pins the expected version (`RACECAR_DASHBOARD_VERSION`
overrides) and reports a mismatch without stopping the install:

```
==> Dashboard versions (driver pins 0.8.3)
  teleop_dashboard: 0.8.3
  linefollow_dashboard: 0.8.2, driver pins 0.8.3
  wallfollow_dashboard: 0.8.3
```

**Safety.** `mux_node` forwards `/drive` only while the RB bumper is held, and
zeroes the output when `/joy` or the active source goes stale. See
[Autonomy gate](#autonomy-gate) for the case where a transmitter holds the
gate.

**Tuning.** The shipped YAML is tuned for the NeoRacer's chassis and lidar.
Expect to retune `max_mps`, `kp`, `kd`, `lookahead` and the `linefollow` HSV
thresholds per car.

The lidar mounting convention (`LIDAR_MOUNT_YAW_DEG` for this car's aft-facing
RPLIDAR) is documented in the
[wallfollow architecture notes](https://github.com/MITRacecarNeo/wallfollow_dashboard/blob/racecar-neo/docs/architecture.md),
and the shared palette in the
[webteleop architecture notes](https://github.com/MITRacecarNeo/teleop_dashboard/blob/racecar-neo/docs/architecture.md).
Where lidar normalization belongs for every consumer is still open with the
Neobotics Foundation; the preferred direction is for the dashboards to consume
the `racecar-neo-library` API rather than raw `/scan`.

## Autonomy gate

By default the gamepad bumpers govern: LB for manual, RB for autonomy, neither
or both for idle. A FlySky transmitter can hold that gate instead, which is off
by default because not every car has one:

```yaml
# config/mux.local.yaml  (gitignored, loaded after mux.yaml)
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

## Bag recording

```bash
racecar log start lap3                     # bag: <timestamp>_lap3
racecar log start lap3 --topics /scan /odom
racecar log status                         # size, rate, time until full
racecar log stop                           # SIGINT, so rosbag2 finalizes
racecar log list
racecar log analyze                        # newest bag: duration, per-topic rates
racecar log config --dir=/data --storage=mcap
```

Bags go to `/data` when an NVMe is mounted there, and `~/logs/bags` otherwise.
`racecar log` names an NVMe that is present but unmounted rather than silently
falling back.

A drive shipped raw has no `/data` to fall back from; `bash scripts/setup_nvme.sh`
partitions, formats and mounts it. That script erases the target disk, so it is
not part of `setup_all.sh`.

Recording every topic pulls in `/camera/color` and `/camera/depth`, roughly
73 MB/s against a sustained SD write near 30 MB/s. `start` refuses on an SD card
unless you pass `--force`, because rosbag2 drops messages rather than blocking:
the bag looks healthy until it is read. Name the topics you need, or
`--exclude '/camera/.*'`.

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
```

RealSense topics, profiles, and known issues: see [docs/specifics/realsense_topics.md](docs/specifics/realsense_topics.md).

For boot-time startup, see [scripts/](./scripts/) for systemd units and the `setup_all.sh` idempotent installer.

## Sensor calibration

Bias and scale are per-board, so each car needs its own calibration run. The
committed `config/lsm9ds1_cal.yaml`, `config/lsm9ds1_mag_cal.yaml` and
`config/realsense_cal.yaml` hold zero defaults; a car's own values go in a
gitignored `config/<name>.local.yaml` beside each, which the launch files load
after the committed file.

```sh
ros2 run racecar_neo_ros2_driver calibrate_imu.py         # LSM9DS1 accel + gyro bias
ros2 run racecar_neo_ros2_driver calibrate_mag.py         # LSM9DS1 hard/soft iron
ros2 run racecar_neo_ros2_driver calibrate_realsense_imu.py   # D435i accel + gyro bias
racecar build                                             # installs the new .local.yaml
racecar service restart teleop                            # nodes read it at start
```

| Utility | Reads | Writes | Consumed by |
|---|---|---|---|
| `calibrate_imu.py` | `/imu/lsm9ds1/raw` | `config/lsm9ds1_cal.local.yaml` | `pit_node` |
| `calibrate_mag.py` | `/mag/raw` | `config/lsm9ds1_mag_cal.local.yaml` | `pit_node` |
| `calibrate_realsense_imu.py` | `/imu/realsense` | `config/realsense_cal.local.yaml` | `imu_fusion_node` |

Each utility writes the file to the source `config/` directory and, when it
exists, the install share `config/` directory. It writes nothing and exits
non-zero when the fit fails or it has too little data.

`calibrate_imu.py` walks a 6-position sequence (each axis up and down) and
averages the gravity vector per pose. `calibrate_mag.py` needs rotation about
all three axes and fits an ellipsoid, then plots raw against corrected samples
so you can confirm the sphere closed up.

The `.local.yaml` files are untracked, so a fresh clone or reimage loses them.
Keep a copy in `~/.config/racecar/calibration/`; each utility prints a reminder.

A car that has never been calibrated runs with zero bias and an identity
soft-iron matrix, and nothing is logged to say so, so run the two LSM9DS1
utilities on every new car. ROS ignores a parameter file whose top-level key
names no running node, without warning; if a calibration appears to have no
effect, check that the key matches the node the launch file starts.

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

## Changelog

Full history in [docs/changelog.md](./docs/changelog.md). Most recent:

- **0.8.3** (unreleased): `racecar status` exits non-zero only on a FAIL, with
  `--strict` for the old rule; slow-but-delivering streams and dual-mode eth0
  warn instead of failing; each sensor row shows a sample of its data; a
  CPU busy share and ARM clock replace the load average; OK, WARN and FAIL
  are coloured on a terminal.
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
