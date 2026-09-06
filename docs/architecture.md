# Architecture

Structure of `racecar_neo_ros2_driver`: the nodes it ships, how they connect,
and what supervises them. Companion to the operational instructions in
[../README.md](../README.md).

## Contents

- [Platform](#platform)
- [Hardware topology](#hardware-topology)
- [Node inventory](#node-inventory)
- [Control pipeline](#control-pipeline)
- [Sensing and perception](#sensing-and-perception)
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
`~/ros2_ws` and is built with `colcon build --symlink-install`, so YAML, launch,
and Python edits take effect without a rebuild.

`sllidar_ros2` is cloned as a sibling package by `scripts/setup_workspace.sh`
rather than vendored.

Boot-level state is reconciled by `scripts/setup_raspi_config.sh`: I2C and SPI,
the serial console, the RTC backup-cell trickle charge, and four bootloader
EEPROM keys. `PSU_MAX_CURRENT=5000` is load-bearing; a BEC-fed car never
negotiates USB-PD, so without it the firmware caps total USB peripheral current
at 600 mA and the camera, lidar, and dongle cannot all run.

Storage: root runs from the SD card. The PCIe lane is split by an ASMedia
ASM1182e switch between the Coral Edge TPU and an NVMe drive; the NVMe is
present but deliberately unpartitioned, reserved for a planned migration of root
off the SD card. An empty drive here is expected, not a fault.

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
the ALFA dongle carrying the isolated AP that operators connect over. Anything
that reconfigures a radio is pinned to one of them by name, so joining a
network cannot disturb the AP.

eth0 carries exactly one IPv4 addressing mode, written by `scripts/setup_eth.sh`
as the single owner of `/etc/netplan/99-racecar-eth0.yaml`. Static is the
default and carries no gateway and no IPv6 default route; dynamic takes both
from DHCP. The modes are mutually exclusive because holding a static address
and a DHCP lease together is what made the static drop: NetworkManager
re-applies the whole IPv4 config for an interface on every lease event.
`setup_networking.sh` delegates to the same writer, so the two paths cannot
disagree about the file.

The Pi drives no display bus directly. `dotmatrix_node` rasterises frames in
software and hands them to `pit_node`, which forwards them to the Teensy; the
Teensy owns the MAX7219 chain.

## Node inventory

Seven nodes ship in this package (`setup.py` console scripts):

| Node | Responsibility |
|---|---|
| `pit_node` | Sole owner of the Teensy UART. Encodes drive commands, decodes telemetry, forwards display and LED frames. |
| `mux_node` | Arbitrates teleop against autonomy, enforces speed and steering limits, gates on the controller or on a live FlySky transmitter, zeroes output when the joystick drops. |
| `throttle_node` | Scales normalised drive commands into the PWM range the hardware expects. |
| `gamepad_node` | Maps `/joy` axes into an Ackermann drive command. |
| `imu_fusion_node` | Merges the RealSense and LSM9DS1 IMUs into one stream; passes through when only one is live. |
| `dotmatrix_node` | Renders text, student pixel buffers, or a mode glyph into a display frame. |
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

Seven dashboards run as `racecar-*` units on ports 8081 to 8087, installed by
`scripts/setup_dashboards.sh` from gitignored checkouts under
`scripts/dashboards/`. Six publish `/drive` and so are mutually exclusive; a
second publisher fights the mux, and `racecar service start` enforces one at a
time. `camlabel` only reads `/camera/color` and can run alongside any of them.

Units are rendered from each upstream `.service.in` rather than copied, which
absorbs the ROS distribution, unit prefix and discovery scope on this side and
leaves every checkout byte-identical to upstream.

## Sensing and perception

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
against the raw topics, which is why they must exist before a car can be
calibrated.

A parameter file whose top-level key names no running node is ignored without
warning. Both LSM9DS1 files were keyed on `imu_node` from v0.3.0 until v0.7.3
and silently contributed nothing; they are keyed on `pit_node` now. When adding
a calibration file, confirm the key matches the node the launch file starts.

`imu_fusion_node` averages accelerometer and gyroscope vectors when both
sources are fresh within `source_timeout_sec`, and passes a single source
through untouched when only one is. RealSense bias correction is applied in the
subscription callback, once per message.

## Display path

```
  /dotmatrix/text ──┐
  /dotmatrix/pixels ├─▶ dotmatrix_node ─ /dotmatrix/frame ─▶ pit_node ─▶ Teensy ─▶ MAX7219 x3
  /joy ─────────────┘   (glyph fallback)
```

`dotmatrix_node` uses `luma` only for font rasterisation. Text times out and
reverts to a mode glyph (IDLE, TELEOP, AUTO) derived from gamepad state.

## Topic reference

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `gamepad_node`, `mux_node`, `dotmatrix_node`, `pit_node` |
| `/gamepad_drive` | `ackermann_msgs/AckermannDriveStamped` | `gamepad_node` | `mux_node` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | user code | `mux_node` |
| `/mux_out` | `ackermann_msgs/AckermannDriveStamped` | `mux_node` | `throttle_node` |
| `/motor` | `ackermann_msgs/AckermannDriveStamped` | `throttle_node` | `pit_node` |
| `/imu/lsm9ds1`, `/imu/lsm9ds1/raw` | `sensor_msgs/Imu` | `pit_node` | `imu_fusion_node`, `calibrate_imu.py` |
| `/mag`, `/mag/raw` | `sensor_msgs/MagneticField` | `pit_node` | `calibrate_mag.py` |
| `/imu/realsense` | `sensor_msgs/Imu` | `realsense2_camera_node` | `imu_fusion_node`, `calibrate_realsense_imu.py` |
| `/imu/fused` | `sensor_msgs/Imu` | `imu_fusion_node` | user code |
| `/encoder/speed` | `std_msgs/Float32` | `pit_node` | user code |
| `/battery/voltage`, `/battery/current` | `std_msgs/Float32` | `pit_node` | dashboard, user code |
| `/rc/channels` | `std_msgs/Float32MultiArray` | `pit_node` | user code |
| `/rc/link` | `std_msgs/Bool` | `pit_node` | `mux_node` |
| `/odom` | `nav_msgs/Odometry` | `pit_node` | lab dashboards, user code |
| `/camera/color`, `/camera/depth` | `sensor_msgs/Image` | `realsense2_camera_node` | `edgetpu_node`, user code |
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` | user code |
| `/edgetpu/inference` | `vision_msgs/Detection2DArray` | `edgetpu_node` | user code |
| `/dotmatrix/text`, `/dotmatrix/pixels` | `std_msgs/String`, `UInt8MultiArray` | user code | `dotmatrix_node` |
| `/dotmatrix/frame`, `/led/pixels` | `std_msgs/UInt8MultiArray` | `dotmatrix_node`, user code | `pit_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | camera, `edgetpu_node` | dashboard |

Sensor topics use BEST_EFFORT, VOLATILE, KEEP_LAST QoS. A subscriber built with
default RELIABLE QoS will not receive them.

## Launch structure

`teleop.launch.py` is the composite entry point and includes one launch file
per subsystem. Every per-node launch file is standalone so the watchdog can
restart a single node without disturbing the rest.

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

Most per-node files delegate to `single_node_launch()` in
`racecar_neo_ros2_driver/launch_common.py`, which declares a
`<name>_config` launch argument and wires one YAML file. Three are explicit
because they need more: `pit.launch.py` loads three YAML files,
`imu_fusion.launch.py` loads two, and `realsense.launch.py` wraps the vendor
launch and applies the topic remaps.

## Process model

Four systemd units, all `User=racecar`:

```
racecar-teleop.service ──── Wants= ───▶ racecar-watchdog.service
        ▲                                        │
        └────────────── BindsTo= ────────────────┘

racecar-dashboard.service   (independent, port 8080)
racecar-jupyter.service     (independent, port 8888)
```

`Wants=` pulls the watchdog up when teleop starts; `BindsTo=` takes it down
when teleop stops, so the supervisor never outlives the thing it supervises.

The default systemd target selects whether the GNOME session starts. It ships
as `graphical.target` and `racecar desktop` switches it to `multi-user.target`
for headless operation. The target is the only lever: the display manager unit
is `static`, so it cannot be enabled or disabled, and `graphical.target` is
what pulls it in. The change is reboot-scoped by design, so it can never tear
down a session in use, and no packages are removed.

`racecar-teleop.service` executes `scripts/launch_teleop.sh`, which creates
`~/logs/<timestamp>/`, repoints the `~/logs/latest` symlink atomically, sweeps
FastRTPS shared-memory orphans, sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
and `exec`s `ros2 launch` so systemd tracks the launch PID directly rather than
a wrapper shell.

## Supervision and observability

`scripts/watchdog.py` supervises seven nodes using two independent liveness
signals: the expected ROS topic advertising, and `pgrep` against the node's
entry-point path. A node is restarted only when both fail, which avoids
restarting a healthy node whose topic went briefly quiet. Additional behavior:

- 30 s restart cooldown per node, SIGTERM then SIGKILL escalation
- FastRTPS shared-memory orphan sweep every 60 s
- Pi 5 PMIC under-voltage alarm via `/sys/class/hwmon/hwmon4/in0_lcrit_alarm`
- Hardware awareness: restart is skipped when the underlying device is absent,
  so an unplugged lidar does not produce a restart loop

`scripts/dashboard.py` serves `/` and `/api/status` on port 8080. Node cards
resolve from topic advertisement; rate rows come from raw subscriptions
(counted without deserialising) except the three RealSense streams, which are
read from the camera's `/diagnostics` output. System Health reports the RTC
backup cell voltage and the sticky PMIC under-voltage alarm.

`scripts/diagnose.py` is the one-shot counterpart behind `racecar status`,
where the dashboard is the continuous view. It opens every subscription at
once and shares a single sample window rather than measuring topics in
sequence, and runs the host checks on a worker thread beside it, so a full
pass costs one discovery plus one window. Checks are grouped as devices,
sensors, actuators, system, services and network; rate checks compare observed
Hz against a per-topic floor rather than testing for presence. It is read-only
and never commands the hardware.

The exit code is strict: 0 only when every requested check passed, so `WARN`
and `SKIP` both count against it. Deselecting a section with `--quick` or
`--section` is distinct from a check failing to run, and does not affect the
result; without that distinction a car with teleop stopped would skip its
sensor checks and still report success.

`scripts/sysinfo.py` holds the host readings both consumers need: RTC
thresholds and classification, the under-voltage alarm, SoC temperature,
throttling flags, load, memory, disk, uptime and clock sync. It exists because
the RTC thresholds were the kind of constant that goes wrong quietly when
duplicated, since a drifted copy still passes its own tests.

## Configuration

One YAML per node under `config/`, keyed by node name and loaded through the
launch argument that `single_node_launch()` declares.

| File | Node | Contents |
|---|---|---|
| `pit.yaml` | `pit_node` | serial device, axis order and polarity, topic names, CRC toggle |
| `mux.yaml` | `mux_node` | speed and steering limits, arming buttons, joystick timeout |
| `throttle.yaml` | `throttle_node` | forward, reverse, and steering PWM fractions |
| `gamepad.yaml` | `gamepad_node` | axis indices and signs |
| `imu_fusion.yaml` | `imu_fusion_node` | source topics, output topic, rate, staleness timeout |
| `lidar.yaml` | `sllidar_node` | serial device, baud, scan mode |
| `dotmatrix.yaml` | `dotmatrix_node` | refresh rate, brightness, text timeout |
| `edgetpu.yaml` | `edgetpu_node` | model path, labels, score threshold, image topic |
| `lsm9ds1_cal.yaml`, `lsm9ds1_mag_cal.yaml` | `pit_node` | generated bias and iron correction |
| `realsense_cal.yaml` | `imu_fusion_node` | generated RealSense IMU bias |

`config/*.local.yaml` is gitignored for per-car overrides.

## Calibration data flow

Calibration values are per-board, so they are generated on each car rather than
committed. Each utility subscribes to a raw topic, fits, and writes YAML to
both the install tree and the source tree so a later `colcon build` does not
discard the result.

```
  /imu/lsm9ds1/raw ─▶ calibrate_imu.py ──────────▶ lsm9ds1_cal.yaml ──────┐
  /mag/raw ────────▶ calibrate_mag.py ──────────▶ lsm9ds1_mag_cal.yaml ──┴─▶ pit_node
  /imu/realsense ──▶ calibrate_realsense_imu.py ─▶ realsense_cal.yaml ─────▶ imu_fusion_node
```

## Repository layout

```
racecar_neo_ros2_driver/
├── racecar_neo_ros2_driver/   node implementations, launch_common, pit_protocol
├── launch/                    one file per subsystem plus teleop composite
├── config/                    one YAML per node, keyed by node name
├── scripts/                   setup phases, watchdog, dashboard, diagnostic,
│                              calibration, systemd units, udev rules
├── test/                      pytest suite (ament convention; not tests/)
├── models/                    EdgeTPU tflite model and labels
├── depend/                    vendored Coral debs and wheels
└── docs/                      this file, changelog, advanced settings,
                               migration and test notes
```

`pit_protocol.py` holds the wire format for the Teensy link and carries no ROS
dependency, so it is unit-testable without a running graph. `sysinfo.py` and
`wifi_scan.py` are separated from their callers for the same reason: both are
pure enough to test against fixtures without hardware.

## Known structural issues

- The test suite lives in `test/`, not `tests/`. This follows the ament and
  `colcon test` convention and is a deliberate exception to the layout used
  elsewhere on this machine.
