# racecar_neo_ros2_driver

ROS2 driver for the **MIT RACECAR Neo v2**: a 1:14-scale autonomous Ackermann-steering racing robot.

This package is the v2 successor to [`racecar-neo-ros2-backend`](https://github.com/MITRacecarNeo/racecar-neo-ros2-backend), with the safety, uptime, and recovery infrastructure ported from [`uav_neo_ros2_driver`](https://github.com/MITUavNeo/uav_neo_ros2_driver). For the full feature catalog of the patterns being inherited, see [docs/features.md](https://github.com/MITUavNeo/uav_neo_ros2_driver/blob/main/docs/features.md) in the UAV Neo repo.

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Quickstart (fresh Ubuntu 24.04 install)](#quickstart-fresh-ubuntu-2404-install)
- [The `racecar` shell tool](#the-racecar-shell-tool)
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
- `/dotmatrix/text` (std_msgs/String): renders user messages; falls back to a mode glyph (IDLE / TELEOP / AUTO) tied to the gamepad state

Safety/uptime layers (inherited from UAV Neo, shipped in v0.0.4):
- **Mux** enforces speed/steer limits and gates commands behind controller bumpers; zeroes output on joystick disconnect (500 ms timeout).
- **Watchdog** (`scripts/watchdog.py`) supervises 7 nodes with two-signal liveness (ROS topic + `pgrep` on the entry-point path), 30 s restart cooldown, SIGTERM to SIGKILL escalation, FastRTPS SHM orphan sweep every 60 s, Pi 5 PMIC under-voltage alarm. Hardware-aware: skips restart when the device is physically missing.
- **Four core systemd units** (`racecar-{teleop,watchdog,dashboard,jupyter}.service`) wired with `BindsTo=` so watchdog dies when teleop dies, and `Wants=` so watchdog auto-starts when teleop starts. Seven more units carry the lab dashboards; they install disabled and are started one at a time.
- **Launch wrapper** (`scripts/launch_teleop.sh`) creates `~/logs/<timestamp>/`, updates `~/logs/latest` atomically, sweeps FastRTPS SHM orphans, and `exec`s `ros2 launch` so systemd tracks the launch PID directly.
- **Web dashboard** at `http://<robot>:8080`: 9 node cards, 9 topic-rate rows, System Health (RTC battery + Pi under-voltage alarm), watchdog log tail. Auto-refresh.
- **JupyterLab** at `http://<robot>:8888` with PYTHONPATH/AMENT_PREFIX_PATH pre-set so `import rclpy` works in notebooks.
- **Pre-flight `colcon test` suite** (365 tests) asserting every peripheral, embedding fix commands in failure messages.

Node responsibilities, the full topic reference, launch composition, and the
calibration data flow are in [docs/architecture.md](./docs/architecture.md).

## Quickstart (fresh Ubuntu 24.04 install)

Target: Raspberry Pi 5 running **Ubuntu Server 24.04 LTS for arm64** (Noble). ROS2 Jazzy is the only supported distro for this driver; older Ubuntu releases (22.04 Jammy) are **not** supported because Jazzy doesn't install there.

### 1. Image the SD card / NVMe

Use Raspberry Pi Imager -> *Other general-purpose OS* -> *Ubuntu* -> *Ubuntu Server 24.04 LTS (64-bit)*. Before writing, click the gear icon and pre-set:

- **Hostname**: `racecar-neo` (matches what the systemd services + dashboard expect)
- **Username**: `racecar` (the `racecar` shell tool, udev groups, and service unit `User=` are all hard-coded to this name; don't change it)
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

Largest single block of the install (~8-15 min on a fresh image at 10 MB/s). With needrestart silenced above, this runs hands-off.

### 4. Clone and run the orchestrator

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` is idempotent; re-running is safe (each phase checks for existing state and skips when already applied). Sudo password is prompted **once** at the top of the run and cached via a background keepalive for the remaining ~45 min; you can walk away after that prompt.

### 5. Apply group memberships

The setup adds your user to `dialout`, `i2c`, `spi`, `gpio`, and `video`. Group membership applies to **new login sessions only**, so:

```sh
exit                     # close SSH
ssh racecar@<ip>         # back in — groups now active
groups                   # verify: dialout i2c spi gpio video should appear
```

### 6. Plug in the hardware and reboot

With the Pi powered off: connect the NEO-PIT PCB (motor, steering, IMU, and the dot matrix chain all hang off it), the RealSense camera, the lidar, the Coral Edge TPU M.2 card, and the EasySMX gamepad's USB dongle. Power on and:

```sh
sudo reboot
```

After reboot, `racecar-teleop.service` auto-starts and pulls the watchdog via `Wants=racecar-watchdog.service`. Verify:

```sh
racecar status              # full diagnostic; exits non-zero unless everything passed
racecar service status      # all 4 racecar-* units should be active+enabled
```

Browse to `http://racecar-neo.local:8080` for the live dashboard.

### 7. (Optional) Switch to AP-mode networking

Once the wired setup works, you can untether the robot from your home WiFi by running:

```sh
racecar setup networking --ssid=racecar-neo-1 --psk='your-password'
```

This brings up an isolated AP on the ALFA dongle (`wlan1`) and configures eth0 with both a static IP and DHCP. See [Networking (optional)](#networking-optional). **Run this from a wired (eth0) session or directly on the console**; it reconfigures the AP interface and will drop SSH-over-WiFi.

### What `setup_all.sh` actually does

Eleven phases, all under `scripts/`:

1. **`setup_ros2.sh`**: ROS2 Jazzy apt repo + message/driver packages
2. **`setup_dev_tools.sh`**: build tools, Python hardware libs (`smbus` / `serial` / `spidev`)
3. **`setup_user_env.sh`**: joins `dialout` / `i2c` / `spi` / `gpio` / `video` groups; sources ROS2 + the `racecar` shell tool in `.bashrc`
4. **`setup_raspi_config.sh`**: boot-level configuration: enable I2C, enable SPI, disable serial console (frees the GPIO UART / `ttyAMA0` for the NEO-PIT link), enable RTC backup-cell trickle charging (`RTC_VCHG_UV=0` skips it; see [RTC backup cell](#rtc-backup-cell)), and reconcile the bootloader EEPROM (`RACECAR_EEPROM=0` skips it; see [Bootloader EEPROM](#bootloader-eeprom))
5. **`setup_udev.sh`**: installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/neo-pit-pcb`, `/dev/lidar`)
6. **`setup_dotmatrix.sh`**: `pip install --user luma.led_matrix`
7. **`setup_coral.sh`**: installs `libedgetpu1-std`, `tflite_runtime`, `pycoral` from vendored `depend/` artifacts
8. **`setup_realsense.sh`**: installs `realsense2_camera` (apt) + the Pi 5 IMU IIO permission fix (script, udev rule, boot service)
9. **`setup_workspace.sh`**: clones `sllidar_ros2` and runs `colcon build --symlink-install`
10. **`setup_jupyter.sh`**: `pip install --user jupyterlab`, creates `~/jupyter_ws/`
11. **`setup_services.sh`**: installs and enables the four core systemd units (`racecar-{teleop,watchdog,dashboard,jupyter}.service`)
12. **`setup_dashboards.sh`**: clones the seven lab dashboards and installs their units, stopped and disabled

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

racecar service status      # active/enabled snapshot, core units and dashboards
racecar service install     # drop unit files in /etc/systemd/system/ + enable
racecar service start       # default: start teleop (watchdog follows via Wants=)
racecar service stop        # default: stop teleop (watchdog follows via BindsTo=)
racecar service logs teleop # journalctl -u racecar-teleop -f
racecar service start wallfollow  # a lab dashboard; stops the other /drive publishers

racecar setup all                       # run the 12-phase orchestrator
racecar setup networking --ssid=foo     # configure eth0 addressing + ALFA-dongle AP
racecar setup networking --show         # print persisted overrides
racecar setup dashboards                # clone the lab dashboards + install units

racecar clear --dmatrix             # flash + clear the MAX7219 display
racecar udev                        # re-install the udev rules
racecar cleanup [--force]           # list / kill stale racecar processes + SHM orphans
racecar status                      # full diagnostic (devices, sensors, system, network)
racecar status --quick              # host checks only; skips the ROS sampling phase
racecar eth status                  # eth0 addressing mode + conflict checks
racecar wifi list                   # visible networks on wlan0
racecar desktop status              # GNOME on/off for the next boot
racecar log start lap3              # record a bag; racecar log stop to finalize
racecar log analyze                 # summarize the newest bag
racecar help                        # full usage
```

Tab completion is registered for subcommands; `racecar launch <TAB>` discovers launch files dynamically, `racecar service <TAB>` offers actions, etc.

The dot matrix pattern sweep is no longer wrapped by a subcommand. Run it directly when bringing up the display hardware, with `dotmatrix_node` already running:

```sh
racecar launch dotmatrix                                    # in another shell
python3 ~/ros2_ws/src/racecar_neo_ros2_driver/scripts/dmatrix_patterns.py all
```

Patterns: `all` (default), `checkerboard`, `all-on`, `sweep`, `module-id`, `font`.

## Networking (optional)

`scripts/setup_networking.sh` configures two things and is **not** invoked by `setup_all.sh`; it's a separate step because it reconfigures the AP interface and would drop SSH-over-WiFi sessions during a fresh install. Run it from a wired (eth0) session or directly on the console:

```sh
racecar setup networking --psk='your-password'
```

With no `--ssid` or saved car ID, it prompts for this car's ID and sets the SSID to `racecar-neo-<id>`, so multiple cars on the same network don't collide. The ID is persisted and reused on later runs.

What it does:

1. **eth0 addressing** via `setup_eth.sh`; eth0 is put in exactly one IPv4 mode, static by default at `192.168.52.200/24`, so the robot is reachable at a known IP on a bare switch. See [Ethernet addressing](#ethernet-addressing).
2. **ALFA-dongle isolated AP** via NetworkManager; the AP runs on the ALFA MT7612U dongle (pinned to `wlan1` by the udev rule), hosting its own 2.4 GHz WiFi network. Clients can SSH / browse the dashboard / use jupyter, but a NetworkManager dispatcher installs `iptables FORWARD REJECT` rules so AP clients **cannot** route through the Pi to the internet (intentional isolation; it keeps the robot's WiFi from becoming a janky general-purpose gateway). The Pi's built-in `wlan0` is left in default client mode.

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
sudo iptables -L FORWARD -n     # two REJECT rules for wlan1
```

## Ethernet addressing

eth0 holds exactly one IPv4 addressing mode. Carrying a static address and a DHCP lease at the same time is what made the static drop periodically: NetworkManager re-applies the whole IPv4 config on every lease event, and in the field the link would come back only after the cable was reseated.

```sh
racecar eth                 # or: racecar eth status
racecar eth static          # 192.168.52.200/24, no gateway (the default)
racecar eth dynamic         # address and default route from DHCP
racecar eth static --addr=10.0.0.50/24
```

Static is the default because a known address is what makes a car debuggable on a bare switch. It carries no gateway or DNS, so a static car reaches the internet over `wlan0` or not at all; switch to `dynamic` when you need `apt` over the wire.

`status` reports the configured mode, the live addresses, both default routes, and fails when it finds more than one global IPv4 address on eth0. The conflict check is IPv4-scoped: the link-local `fe80::` address is always present and SLAAC may add more, so counting every address would report a conflict on a healthy car.

In static mode eth0 keeps its IPv6 addresses but never a default route (`ipv6.never-default`). Router advertisements would otherwise hand eth0 a v6 default route despite the absence of a v4 gateway, and since most large destinations are dual-stack a "gateway-less" car would still send most of its traffic out the wire.

**Switching modes drops an SSH session arriving over eth0.** The command detects that and asks first; use the AP, `wlan0`, or an HDMI console, or pass `--force`.

### Confirming the fix

Making the modes mutually exclusive removes the structural cause of the drop, but that reasoning is not the same as evidence: the symptom is periodic and recovers only when the cable is reseated, so a quiet afternoon proves nothing. `racecar eth monitor` records the addresses, both default routes, carrier, operstate and NetworkManager state, logging a line whenever any of them changes plus a heartbeat every 15 minutes.

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

A clean run is `START` followed by heartbeats. Any `ADDR_LOST`, `CARRIER` or `OPERSTATE` line is the event worth reading. Disable the unit once the question is settled; it is a diagnostic, not part of the running car.

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

`connect` brings up a saved profile as-is, whatever its security type. For a new network it asks for a passphrase, or for an identity and password on an enterprise (802.1X) network, and nothing else. Server validation is not optional: every enterprise profile gets system CA certificates plus a `domain-suffix-match` derived from the identity's realm, so credentials are never offered to an access point that cannot prove who it is. Use `--ca-cert=` and `--domain-suffix-match=` where that derivation does not fit.

`disconnect` puts the device into NetworkManager's manually-disconnected state, so it will not rejoin on its own until the next `connect`.

## Desktop toggle

The GNOME desktop ships enabled. Headless users turn it off without removing anything:

```sh
racecar desktop             # or: racecar desktop status
racecar desktop disable     # boot to multi-user.target
racecar desktop enable      # boot to graphical.target
```

The boot target is the only lever and is sufficient on its own: the display manager unit is `static` (no `[Install]` section), so `systemctl enable`/`disable` on it cannot work, and `graphical.target` is what pulls it in.

Changes apply on the **next boot**. There is deliberately no immediate variant, so the command can never tear down a desktop session someone is using; `status` reports a pending change when the default and active targets disagree. Packages are never removed, so the toggle works on a car with no network.

## Web dashboard

Once `racecar-teleop.service` is running, browse to `http://<robot>:8080` for a live status page:

- **Nodes**: one card per monitored subsystem (9 total): green when the expected topic is being advertised, red when not.
- **System Health**: RTC backup battery voltage (green >= 3.0 V, yellow 2.7-3.0 V, red < 2.7 V) and the Pi 5 PMIC sticky under-voltage alarm.
- **Topic Rates**: live Hz for `/motor`, `/mux_out`, `/imu/fused`, `/imu/lsm9ds1`, `/scan`, `/edgetpu/inference`, `/camera/color`, `/camera/depth`, and `/imu/realsense`. Yellow when stale (< 0.5 Hz), red when missing. The three RealSense rows are read from the camera's own `/diagnostics` stream; the rest are counted from raw subscriptions, which keeps the dashboard's own CPU cost near 20%.
- **Watchdog Log**: tail of `~/logs/latest/watchdog.log` so you can see restart events.

Refreshes every 3 s; System Health refreshes on a slower 60 s cadence (RTC drifts on the order of weeks, not seconds).

## Lab dashboards

Seven browser-based labs, each its own upstream repository under the [Neobotics
Foundation](https://github.com/Neobotics-Foundation-Inc) organization, installed
as `racecar-*` systemd units.

> **The lidar-based dashboards do not steer correctly on this platform.**
> `wallfollow`, `eps`, and `smartfollow` derive steering from `/scan` using their
> own angle arithmetic, and that arithmetic assumes a lidar mount this chassis
> does not have. They will drive into the wrong half of the world. `webteleop`
> still drives correctly (its input is manual) but its lidar view is rotated.
> The camera-only dashboards are unaffected. See
> [Lidar convention mismatch](#lidar-convention-mismatch).


| Dashboard | Port | Reads | Publishes |
|---|---|---|---|
| `wallfollow` | 8081 | `/scan`, `/odom` | `/drive` |
| `camlabel` | 8082 | `/camera/color` | none |
| `pursuit` | 8083 | `/camera/color`, `/edgetpu/inference` | `/drive` |
| `eps` | 8084 | `/scan`, `/odom` | `/drive` |
| `smartfollow` | 8085 | `/scan`, `/odom`, `/camera/color`, `/edgetpu/inference` | `/drive` |
| `linefollow` | 8086 | `/camera/color`, `/odom` | `/drive` |
| `webteleop` | 8087 | `/camera/color`, `/scan`, `/odom` | `/drive` |

Install (also runs as phase 12 of `setup_all.sh`):

```bash
racecar setup dashboards          # clone or fast-forward, install units
racecar setup dashboards --update # pull only
racecar service update            # same, from the service subcommand
```

Run one:

```bash
racecar service start wallfollow   # stops the other /drive publishers first
racecar service status             # core and dashboards, with URLs
racecar service logs wallfollow
racecar service stop wallfollow
```

**Usable today:**

| Dashboard | Reads `/scan` | Status |
|---|---|---|
| `camlabel` (8082) | no | works |
| `pursuit` (8083) | no | works |
| `linefollow` (8086) | no | works |
| `webteleop` (8087) | view only | drives correctly; lidar view rotated |
| `wallfollow` (8081) | yes | **steers wrongly** |
| `eps` (8084) | yes | **steers wrongly** |
| `smartfollow` (8085) | yes | **steers wrongly** |

**One at a time.** Six of the seven publish `/drive`, and a second publisher
fights the mux, so `racecar service start` stops the others before starting the
one you asked for. `camlabel` only reads the camera and can run alongside any of
them. Units install disabled; `racecar service enable <name>` makes one survive
a reboot, and bare `racecar service enable` deliberately covers only the core
four.

**Checkouts live in `scripts/dashboards/`**, gitignored. Updates are
`git pull --ff-only` and never `reset --hard`, so a car's tuned `wallfollow.yaml`
survives. Nothing in a checkout is modified: the ROS distribution, unit naming
and discovery scope are handled by rendering our own unit from the upstream
template.

**The upstream safety text does not describe this car.** Every dashboard README
says the mux forwards `/drive` with no software deadman and the transmitter's
SWB switch is the only gate. Here, `mux_node` requires the RB bumper held, and
zeroes the output when `/joy` or the active source goes stale. See
[Autonomy gate](#autonomy-gate) for the case where a transmitter does hold the
gate.

### Lidar convention mismatch

The dashboards read `/scan` directly and do their own angle arithmetic, taking
0 as the car's nose and positive as its right. On this chassis that assumption
does not hold: measured on hardware, an object placed to the car's right is
reported by their arithmetic at -90 degrees (their left), and an object placed
in front at 180 degrees (behind them). A mirrored scan would have put the front
object at 0, so the difference is a 180 degree yaw, not a handedness flip.

This is not a bug in either codebase. It is a missing convention: nothing
defines whose job it is to normalize lidar orientation, so the driver and the
dashboards each assume the other has done it. `racecar-neo-library` resolves the
same question for student code, in `real/lidar_real.py`, by normalizing on the
way in.

Fixing it needs agreement with the Neobotics Foundation on where normalization
belongs. The preferred direction is for the dashboards to consume the library
API rather than raw `/scan`, so orientation and units are normalized once for
every consumer. Correcting `/scan` inside this driver was prototyped and
reverted: it works, but it settles a shared convention unilaterally, and any car
running an unaware consumer would then be wrong in the other direction.

Until then, treat `wallfollow`, `eps`, and `smartfollow` as installed but not
functional on this platform.

**These are NeoRacer numbers.** The shipped YAML is tuned for a different
chassis and lidar. Expect to retune `max_mps`, `kp`, `kd`, `lookahead` and the
`linefollow` HSV thresholds per car.

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
racecar launch imu
racecar launch lidar
racecar launch edgetpu
racecar launch dotmatrix
```

RealSense topics, profiles, and known issues: see [docs/realsense_topics.md](docs/realsense_topics.md).


For boot-time startup, see [scripts/](./scripts/) for systemd units and the `setup_all.sh` idempotent installer.

## Sensor calibration

Bias and scale are per-board, so each car needs its own calibration run. All
three utilities write their YAML to both the install tree and the source tree,
so a later `colcon build` does not discard the result.

```sh
ros2 run racecar_neo_ros2_driver calibrate_imu.py         # LSM9DS1 accel + gyro bias
ros2 run racecar_neo_ros2_driver calibrate_mag.py         # LSM9DS1 hard/soft iron
ros2 run racecar_neo_ros2_driver calibrate_realsense_imu.py   # D435i accel + gyro bias
```

| Utility | Reads | Writes | Consumed by |
|---|---|---|---|
| `calibrate_imu.py` | `/imu/lsm9ds1/raw` | `config/lsm9ds1_cal.yaml` | `pit_node` |
| `calibrate_mag.py` | `/mag/raw` | `config/lsm9ds1_mag_cal.yaml` | `pit_node` |
| `calibrate_realsense_imu.py` | `/imu/realsense` | `config/realsense_cal.yaml` | `imu_fusion_node` |

`calibrate_imu.py` walks a 6-position sequence (each axis up and down) and
averages the gravity vector per pose. `calibrate_mag.py` needs rotation about
all three axes and fits an ellipsoid, then plots raw against corrected samples
so you can confirm the sphere closed up.

A car that has never been calibrated runs with zero bias and an identity
soft-iron matrix, and nothing is logged to say so, so run the two LSM9DS1
utilities on every new car. Note that ROS ignores a parameter file whose
top-level key names no running node, without warning; if a calibration appears
to have no effect, check that the key matches the node the launch file starts.

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
fitted, run the phase with `RTC_VCHG_UV=0 bash scripts/setup_raspi_config.sh`
and swap the cell before enabling it.

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

Changes take effect on the next boot. `RACECAR_EEPROM=0` skips the whole step.

## ROS discovery scope

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` is set in `launch_teleop.sh`, the
dashboard / watchdog / jupyter units, and the `.bashrc` block written by
`setup_user_env.sh`. Every node in this stack runs on the robot, so restricting
discovery to the loopback interface costs nothing on-board and keeps discovery
chatter off the ALFA dongle, where it was driving CPU spikes.

The tradeoff: a laptop cannot see the robot's topics. `rviz`,
`ros2 topic echo`, and remote nodes will find nothing. For a session where you
need them, widen the range on both machines:

```sh
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

## Changelog

Full history in [docs/changelog.md](./docs/changelog.md). Most recent:

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
