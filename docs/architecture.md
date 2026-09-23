# Architecture

Structure of `racecar_neo_ros2_driver`: the nodes it ships, how they connect,
and what supervises them. Companion to the operational instructions in
[../README.md](../README.md).

## Contents

- [Platform](#platform)
- [Hardware topology](#hardware-topology)
- [Node inventory](#node-inventory)
- [Control pipeline](#control-pipeline)
- [Sensor and perception nodes](#sensor-and-perception-nodes)
- [Display path](#display-path)
- [Topic reference](#topic-reference)
- [Launch structure](#launch-structure)
- [Process model](#process-model)
- [Supervision and observability](#supervision-and-observability)
- [Configuration](#configuration)
- [Calibration data flow](#calibration-data-flow)
- [Repository layout](#repository-layout)
- [Known structural issues](#known-structural-issues)

## Platform

Raspberry Pi 5 (BCM2712, aarch64, 4 cores) on Ubuntu Server 24.04 LTS, ROS 2
Jazzy. The package is `ament_python`; the workspace overlay lives at
`~/ros2_ws` and is built with `colcon build --symlink-install`, so edits to
existing YAML, launch, and Python files take effect without a rebuild. A new
file under `config/` needs one `racecar build` to appear in the install tree.

`sllidar_ros2` is cloned as a sibling package by `scripts/setup_workspace.sh`
rather than vendored.

Boot-level state is reconciled by `scripts/setup_raspi_config.sh`: I2C and SPI,
the serial console, the RTC backup-cell trickle charge, and four bootloader
EEPROM keys. `PSU_MAX_CURRENT=5000` lifts the 600 mA USB peripheral cap that a
BEC-fed car (no USB-PD negotiation) otherwise gets; see README "Bootloader
EEPROM".

Storage: root runs from the SD card. The PCIe lane is split by an ASMedia
ASM1182e switch between the Coral Edge TPU and an NVMe drive. The NVMe holds
`/data` (ext4, made by `scripts/setup_nvme.sh`, which `setup_all.sh` does not
run because it erases the disk); `racecar log` records bags there when it is
mounted.

## Hardware topology

| Subsystem | Component | Interface | Owning node |
|---|---|---|---|
| Motor, steering, IMU, power, display bridge | NEO-PIT PCB (Teensy 4.1; LSM9DS1 + INA226 + hall encoder) | UART `/dev/neo-pit-pcb` | `pit_node` |
| Camera (color, depth, IMU) | Intel RealSense D435i | USB 3.x (`8086:0b3a`) | `realsense2_camera_node` |
| 2D LIDAR | RPLIDAR A3-class | UART `/dev/lidar` | `sllidar_node` |
| Gamepad | Switch Pro / EasySMX | USB HID `/dev/input/*` | `joy_node` |
| ML inference | Coral Edge TPU M.2 (Apex, PCIe) | `/dev/apex_0` | `edgetpu_node` |
| Dot matrix display | MAX7219, 3 cascaded | driven by the Teensy, fed over UART | `dotmatrix_node` plus `pit_node` |
| AP radio | ALFA MT7612U | USB, pinned to `wlan1` | NetworkManager |

Every `/dev/*` path above is a udev symlink installed by
`scripts/setup_udev.sh`, so device numbering does not shift across reboots.

The two radios have separate roles and are never interchangeable. `wlan0` is
the Pi's built-in Broadcom part and is the only client interface; `wlan1` is
the ALFA dongle carrying the isolated AP. Anything that reconfigures a radio is
pinned to one of them by name, so joining a network cannot disturb the AP.

NetworkManager gates activation and profile edits behind polkit.
`scripts/polkit/49-racecar-network.rules`, installed by
`scripts/setup_user_env.sh`, grants the five actions `racecar wifi` needs to the
`sudo` group; see docs/troubleshooting.md, "NetworkManager authorization".

eth0 carries exactly one IPv4 addressing mode, written by `scripts/setup_eth.sh`
as the single owner of `/etc/netplan/99-racecar-eth0.yaml`. Static is the
default and carries no gateway and no IPv6 default route; dynamic takes both
from DHCP. `setup_networking.sh` delegates to the same writer, and the eth0-SSH
guard in `setup_eth.sh` applies on both paths. Rationale: docs/troubleshooting.md,
"eth0 addressing".

The Pi drives no display bus directly. `dotmatrix_node` rasterises frames in
software and hands them to `pit_node`, which forwards them to the Teensy; the
Teensy owns the MAX7219 chain.

## Node inventory

Seven nodes ship in this package (`setup.py` console scripts):

| Node | Responsibility |
|---|---|
| `pit_node` | Sole owner of the Teensy UART. Encodes drive commands, decodes telemetry, forwards display and LED frames. |
| `mux_node` | Arbitrates teleop against autonomy, enforces speed and steering limits, gates on the controller or on a live FlySky transmitter, zeroes output when the joystick drops. |
| `throttle_node` | Clamps `/mux_out` to `[-1, 1]` and applies per-direction speed and steering caps onto `/motor`. The Teensy maps the normalised command to PWM. |
| `gamepad_node` | Maps `/joy` axes into an Ackermann drive command. |
| `imu_fusion_node` | Merges the RealSense and LSM9DS1 IMUs into one stream; passes through when only one is live. |
| `dotmatrix_node` | Renders text, student pixel buffers, a startup splash, or a mode glyph into a display frame. |
| `edgetpu_node` | Runs object detection on the color stream via the Coral Apex. |

Three external nodes complete the graph: `joy_node` (`joy`), `sllidar_node`
(`sllidar_ros2`), and `realsense2_camera_node` (`realsense2_camera`).

## Control pipeline

Commands flow one way, and every path converges on `mux_node` before reaching
hardware. `mux_node` is the single safety gate.

```
  EasySMX / Switch Pro
          │ USB HID
          ▼
      joy_node ──────── /joy ──────────┬──────────────┬───────────────┐
          │                            │              │               │
          │                            ▼              ▼               ▼
          └─ /joy ─▶ gamepad_node   mux_node     dotmatrix_node   pit_node
                          │        (arming +      (mode glyph)   (button state)
                          │         deadman)
                          │              ▲
              /gamepad_drive             │
                          │              │
                          └──────────────┤
                                         │
   autonomy (labs, notebooks) ─ /drive ──┘
                                         │
                                    /mux_out
                                         │
                                         ▼
                                  throttle_node
                                         │
                                     /motor
                                         │
                                         ▼
                                     pit_node
                                         │ UART framing
                                         ▼
                                  NEO-PIT PCB (Teensy 4.1)
                                         │
                              ┌──────────┴──────────┐
                              ▼                     ▼
                         ESC / motor         steering servo
```

`mux_node` subscribes to `/joy` directly rather than trusting `gamepad_node`,
so the arming gate and the 500 ms disconnect timeout stay independent of the
mapping layer. A stale or missing `/joy` zeroes `/mux_out` regardless of what
`/drive` is publishing.

### Drive authority

Two authorities can hold the gate. The gamepad bumpers hold it by default. A
FlySky transmitter holds it instead when `rc_authority_enable` is set and the
link is live, in which case channel 6 selects the mode and the bumpers are
ignored; the transmitter is a gate, not a drive source, so manual mode still
routes `/gamepad_drive`.

Presence of a transmitter cannot be read from `/rc/channels`. `rc_normalized`
clamps pulse widths into `[-1, 1]`, which puts a dead channel (near 0 us) on
exactly `-1.0`, the same value a switch held low produces. `pit_node` therefore
publishes `/rc/link` from the raw widths, and `mux_node` gates on that plus
freshness, a sustained hold, and the switch having been seen at middle. Granting
is slow and revoking immediate: one bad frame hands the gate back.

### Lab dashboards

Three dashboards run as `racecar-*` units, installed by
`scripts/setup_dashboards.sh` from gitignored checkouts under
`scripts/dashboards/`: `webteleop` on 8081, `linefollow` on 8082 and
`wallfollow` on 8083, continuing from the driver's own dashboard on 8080. All
three publish `/drive` and so are mutually exclusive; a second publisher fights
the mux, and `racecar service start` enforces one at a time.
`setup_dashboards.sh` also stops, disables and removes units left by the four
retired dashboards (`camlabel`, `eps`, `pursuit`, `smartfollow`).

Each is a fork under MITRacecarNeo of the corresponding Neobotics Foundation
repository, checked out on its `racecar-neo` branch, so the lidar convention,
ports and branding are corrected at the source. Each checkout carries
`VERSION`, `docs/changelog.md` and `docs/architecture.md`; the version tracks
this driver's release, and `setup_dashboards.sh` reports a checkout that does
not match the version it pins. Dashboard internals (depth preview, lidar
convention, palette) are documented in each fork's `docs/architecture.md`.

Units are rendered from each checkout's `.service.in` rather than copied, so a
fork synced from Neobotics upstream keeps working: the ROS distribution, unit
prefix and discovery scope are absorbed on this side either way.

## Sensor and perception nodes

Sensor nodes publish independently of the control chain; nothing in this
section can block a drive command.

```
  NEO-PIT PCB ─ UART ─▶ pit_node ─┬─▶ /imu/lsm9ds1      /imu/lsm9ds1/raw
                                  ├─▶ /mag              /mag/raw
                                  ├─▶ /encoder/speed
                                  ├─▶ /battery/voltage  /battery/current
                                  ├─▶ /rc/channels  /rc/link
                                  └─▶ /odom

  RealSense D435i ─ USB3 ─▶ realsense2_camera_node ─┬─▶ /camera/color
                                                    ├─▶ /camera/depth
                                                    ├─▶ /imu/realsense
                                                    └─▶ /diagnostics

  RPLIDAR ─ UART ─▶ sllidar_node ─▶ /scan

  /imu/realsense ─┐
                  ├─▶ imu_fusion_node ─▶ /imu/fused
  /imu/lsm9ds1 ───┘

  /camera/color ─▶ edgetpu_node ─┬─▶ /edgetpu/inference
                                 └─▶ /diagnostics
```

`pit_node` publishes each inertial channel twice. The plain topic carries the
bias-corrected value; the `/raw` topic carries the same reading axis-remapped
and sensitivity-scaled with zero bias applied. The calibration utilities fit
against the raw topics.

A parameter file whose top-level key names no running node is ignored without
warning. When adding a calibration file, confirm the key matches the node the
launch file starts.

`imu_fusion_node` averages accelerometer and gyroscope vectors when both
sources are fresh within `source_timeout_sec`, and passes a single source
through untouched when only one is. RealSense bias correction is applied in the
subscription callback, once per message.

`edgetpu_node` infers at `inference_rate_hz` (15) rather than at the camera's
60 fps. Frames above the rate are dropped before the decode, and the
stale-input watchdog still stamps every frame, so it reports on the camera
rather than on the rate gate. `diagnose.py`'s nominal for
`/edgetpu/inference` tracks the same number.

## Display path

```
  /dotmatrix/text ──┐
  /dotmatrix/pixels ├─▶ dotmatrix_node ─ /dotmatrix/frame ─▶ pit_node ─▶ Teensy ─▶ MAX7219 x3
  /joy ─────────────┘   (glyph fallback)
```

`dotmatrix_node` uses `luma` only for font rasterisation. Text times out and
reverts to a mode glyph and label (IDLE, MAN, AUTO) derived from gamepad
state. `scripts/dmatrix_patterns.py` is the display self-test; it publishes
patterns on `/dotmatrix/pixels`.

## Topic reference

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `gamepad_node`, `mux_node`, `dotmatrix_node`, `pit_node` |
| `/gamepad_drive` | `ackermann_msgs/AckermannDriveStamped` | `gamepad_node` | `mux_node` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | user code, lab dashboards | `mux_node` |
| `/mux_out` | `ackermann_msgs/AckermannDriveStamped` | `mux_node` | `throttle_node` |
| `/motor` | `ackermann_msgs/AckermannDriveStamped` | `throttle_node` | `pit_node` |
| `/imu/lsm9ds1`, `/imu/lsm9ds1/raw` | `sensor_msgs/Imu` | `pit_node` | `imu_fusion_node`, `calibrate_imu.py` |
| `/mag`, `/mag/raw` | `sensor_msgs/MagneticField` | `pit_node` | `calibrate_mag.py` |
| `/imu/realsense` | `sensor_msgs/Imu` | `realsense2_camera_node` | `imu_fusion_node`, `calibrate_realsense_imu.py` |
| `/imu/fused` | `sensor_msgs/Imu` | `imu_fusion_node` | user code |
| `/encoder/speed` | `std_msgs/Float32` | `pit_node` | user code |
| `/battery/voltage`, `/battery/current` | `std_msgs/Float32` | `pit_node` | dashboard, user code |
| `/rc/channels` | `std_msgs/Float32MultiArray` | `pit_node` | `mux_node` (when `rc_authority_enable`), user code |
| `/rc/link` | `std_msgs/Bool` | `pit_node` | `mux_node` (when `rc_authority_enable`) |
| `/odom` | `nav_msgs/Odometry` | `pit_node` | lab dashboards, user code |
| `/camera/color` | `sensor_msgs/Image` | `realsense2_camera_node` | `edgetpu_node`, `webteleop`, `linefollow`, user code |
| `/camera/depth` | `sensor_msgs/Image` | `realsense2_camera_node` | `webteleop`, user code |
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` | `webteleop`, `wallfollow`, user code |
| `/edgetpu/inference` | `vision_msgs/Detection2DArray` | `edgetpu_node` | `webteleop`, user code |
| `/dotmatrix/text`, `/dotmatrix/pixels` | `std_msgs/String`, `UInt8MultiArray` | user code | `dotmatrix_node` |
| `/dotmatrix/frame`, `/led/pixels` | `std_msgs/UInt8MultiArray` | `dotmatrix_node`, user code | `pit_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | camera, `edgetpu_node` | dashboard, `diagnose.py` |

Sensor and control topics use BEST_EFFORT, VOLATILE, KEEP_LAST QoS. A
subscriber built with default RELIABLE QoS will not receive them.
`/edgetpu/inference` and `edgetpu_node`'s `/diagnostics` are published
RELIABLE (depth 10), so either QoS receives them.

## Launch structure

`teleop.launch.py` is the composite entry point and includes one launch file
per subsystem. Every per-node launch file is standalone; the watchdog uses the
seven files for the nodes it supervises to restart one node without disturbing
the rest.

```
teleop.launch.py
├── joy_node                     (inline Node)
├── gamepad.launch.py            ─┐
├── mux.launch.py                 │ control chain, started first,
├── throttle.launch.py            │ no stagger
├── pit.launch.py                ─┘
├── imu_fusion.launch.py         ─┐
├── lidar.launch.py               │ gated by <name>_enable launch args
├── realsense.launch.py           │
├── edgetpu.launch.py  (+3.0 s)   │ stagger lets camera topics appear first
└── dotmatrix.launch.py          ─┘
```

Per-node files build on `single_node_launch()` in
`racecar_neo_ros2_driver/launch_common.py`, which declares a `<name>_config`
launch argument for the node's YAML and appends `config/<name>.local.yaml`
when it exists. Two load extra files: `pit.launch.py` adds
`lsm9ds1_cal.yaml` and `lsm9ds1_mag_cal.yaml`, and `imu_fusion.launch.py` adds
`realsense_cal.yaml`, each followed by its `.local.yaml` when present.
`realsense.launch.py` wraps the vendor launch and applies the topic remaps.

## Process model

Four core systemd units, all `User=racecar`:

```
racecar-teleop.service ──── Wants= ───▶ racecar-watchdog.service
        ▲                                        │
        └────────────── BindsTo= ────────────────┘

racecar-dashboard.service   (independent, port 8080)
racecar-jupyter.service     (independent, port 8888)
```

`Wants=` pulls the watchdog up when teleop starts; `BindsTo=` takes it down
when teleop stops, so the supervisor never outlives the thing it supervises.

Beside them: the three lab-dashboard units (`racecar-webteleop`,
`racecar-linefollow`, `racecar-wallfollow`), rendered by
`setup_dashboards.sh` and installed disabled, and
`scripts/racecar-eth-monitor.service`, a diagnostic that no setup phase
installs.

The default systemd target selects whether the GNOME session starts. It ships
as `graphical.target`, and `racecar desktop` switches it to `multi-user.target`
for headless operation. The change is reboot-scoped; see
docs/troubleshooting.md, "Desktop toggle scope".

`racecar-teleop.service` executes `scripts/launch_teleop.sh`, which creates
`~/logs/<timestamp>/`, repoints the `~/logs/latest` symlink atomically, sweeps
FastRTPS shared-memory orphans, sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
and `exec`s `ros2 launch` so systemd tracks the launch PID directly rather than
a wrapper shell.

## Supervision and observability

`scripts/watchdog.py` supervises seven nodes (pit, throttle, mux, gamepad,
imu_fusion, lidar, realsense) using two liveness signals: the expected ROS
topic advertising, and `pgrep` against the node's entry-point path. A node is
restarted when either signal fails. `/scan` also carries a freshness window
(`freshness_sec`, 5 s), so a lidar that stays advertised but stops publishing
is restarted too. Additional behavior:

- 30 s restart cooldown per node, SIGTERM then SIGKILL escalation
- FastRTPS shared-memory orphan sweep every 60 s
- Pi 5 PMIC under-voltage alarm via the `rpi_volt` hwmon's `in0_lcrit_alarm`,
  found by driver name because hwmon numbering is not stable
- Hardware awareness: restart is skipped when the underlying device is absent,
  so an unplugged lidar does not produce a restart loop

`edgetpu_node` and `dotmatrix_node` are not supervised; the dashboard shows
their cards as `unsupervised` rather than dead when they are down.

`scripts/dashboard.py` serves `/` and `/api/status` on port 8080. Node cards
resolve from topic advertisement; rate rows come from raw subscriptions
(counted without deserialising) except the three RealSense streams, which are
read from the camera's `/diagnostics` output. System Health reports the RTC
backup cell voltage and the sticky PMIC under-voltage alarm.

`scripts/diagnose.py` is the one-shot counterpart behind `racecar status`,
where the dashboard is the continuous view. It opens every subscription at
once and shares a single 5 s sample window rather than measuring topics in
sequence, and runs the host checks on a worker thread beside it. Checks are
grouped as devices, sensors, actuators, system, services and network; rate
checks compare observed Hz against a per-topic floor rather than testing for
presence. Each rate row also carries one decoded sample from its topic in a
tab-separated column, and a payload outside its expected range (gravity,
pack voltage, channel count) sets the row's status. The RealSense streams
are read once after the window closes, so their decode costs no rate. It is
read-only and never commands the hardware. The measurement
cost of that window, and the floors, are covered in docs/troubleshooting.md,
"Diagnostic rate checks".

The exit code is 1 only when a check fails; `WARN` and `SKIP` leave the car
usable and exit 0. `--strict` makes anything other than `OK` fail, for
scripts that need every check to have run and passed. Deselecting a section
with `--quick` or `--section` is distinct from a check failing to run and
affects neither mode. A car with teleop stopped still fails: its topics are
absent from the graph, and an absent topic is a `FAIL`, not a `SKIP`.

`scripts/sysinfo.py` holds the host readings the dashboard, `diagnose.py`,
`watchdog.py` and `eth_monitor.py` share: RTC thresholds and classification,
the under-voltage alarm path, SoC temperature, throttling flags, CPU time,
the ARM clock, memory, disk, uptime, clock sync, the RealSense `/diagnostics`
rate parser and a subprocess helper. Each reading has one definition.

## Configuration

One YAML per node under `config/`, keyed by node name and loaded through the
node's `<name>_config` launch argument.

| File | Node | Contents |
|---|---|---|
| `pit.yaml` | `pit_node` | serial device, axis order and polarity, topic names, CRC toggle |
| `mux.yaml` | `mux_node` | speed and steering limits, arming buttons, joystick timeout, RC gate |
| `throttle.yaml` | `throttle_node` | forward, reverse, and steering caps as fractions of full scale |
| `gamepad.yaml` | `gamepad_node` | axis indices and signs |
| `imu_fusion.yaml` | `imu_fusion_node` | source topics, output topic, rate, staleness timeout |
| `lidar.yaml` | `sllidar_node` | serial device, baud, scan mode |
| `dotmatrix.yaml` | `dotmatrix_node` | module count, refresh rate, scroll and pixel timeouts, splash, mode buttons |
| `edgetpu.yaml` | `edgetpu_node` | model path, labels, score threshold, image topic, inference rate |
| `lsm9ds1_cal.yaml`, `lsm9ds1_mag_cal.yaml` | `pit_node` | IMU bias and iron correction, zero defaults |
| `realsense_cal.yaml` | `imu_fusion_node` | RealSense IMU bias, zero defaults |

`config/*.local.yaml` is gitignored. Each launch file loads `<name>.local.yaml`
after the committed file of the same name, so per-car values (calibration, an
enabled RC gate) override the shipped defaults without dirtying the tree.

## Calibration data flow

The committed calibration files hold zero defaults. Each utility subscribes to
a raw topic, fits, and writes the per-car result to the gitignored
`.local.yaml` counterpart in the source `config/` directory, and in the install
share `config/` directory when it exists. It writes only keys the node
declares, and writes nothing (exit non-zero) when the fit fails or there is too
little data.

```
  /imu/lsm9ds1/raw ─▶ calibrate_imu.py ──────────▶ lsm9ds1_cal.local.yaml ──────┐
  /mag/raw ────────▶ calibrate_mag.py ──────────▶ lsm9ds1_mag_cal.local.yaml ──┴─▶ pit_node
  /imu/realsense ──▶ calibrate_realsense_imu.py ─▶ realsense_cal.local.yaml ─────▶ imu_fusion_node
```

After a run: `racecar build` (installs the new file), then restart teleop. The
`.local.yaml` files are not in git; keep a backup in
`~/.config/racecar/calibration/`.

## Repository layout

```
racecar_neo_ros2_driver/
├── racecar_neo_ros2_driver/   node implementations, launch_common, pit_protocol,
│                              limits (shared clamp)
├── launch/                    one file per subsystem plus teleop composite
├── config/                    one YAML per node, keyed by node name;
│                              *.local.yaml per-car overrides (gitignored)
├── scripts/                   setup phases, racecar-tool.sh, watchdog, dashboard,
│                              diagnose, sysinfo, eth_monitor, wifi_scan,
│                              racecar_log, calibration, dmatrix_patterns,
│                              systemd units, udev, polkit and modprobe rules,
│   │                          Coral overlay and gasket patch
│   └── dashboards/            lab-dashboard checkouts (gitignored)
├── test/                      pytest suite (ament convention; not tests/)
├── models/                    EdgeTPU models: COCO EfficientDet-Lite0 (default)
│                              and the single-class generic model, with labels
├── depend/                    vendored Coral debs and wheels
├── resource/                  ament index marker
├── docs/                      this file, changelog, troubleshooting,
│   │                          advanced settings
│   ├── img/                   RACECAR Neo logo set
│   └── specifics/             RealSense topics, Coral M.2 migration
├── package.xml, setup.py, setup.cfg
└── pyproject.toml             ruff, black and mypy settings
```

`pit_protocol.py` holds the wire format for the Teensy link and carries no ROS
dependency, so it is unit-testable without a running graph. `sysinfo.py` and
`wifi_scan.py` are separated from their callers for the same reason: both are
pure enough to test against fixtures without hardware.

## Known structural issues

- The layout follows the ament and `colcon test` conventions rather than the
  `src/`, `tests/`, `data/` scaffold used elsewhere on this machine: the Python
  package sits at the top level, the suite lives in `test/`, and there is no
  data directory.
- `edgetpu_node` and `dotmatrix_node` run without watchdog supervision.
