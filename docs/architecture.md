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
- [Frame tree](#frame-tree)
- [Process model](#process-model)
- [Supervision and observability](#supervision-and-observability)
- [Battery guard](#battery-guard)
- [Terminal app](#terminal-app)
- [Configuration](#configuration)
- [Data root](#data-root)
- [Calibration data flow](#calibration-data-flow)
- [Repository layout](#repository-layout)
- [Known structural issues](#known-structural-issues)

## Platform

Raspberry Pi 5 (BCM2712, aarch64, 4 cores) on Ubuntu Desktop 24.04 LTS, ROS 2
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
ASM1182e switch between the Coral Edge TPU and an NVMe drive. The NVMe, ext4
labeled `racecar-data` and mounted at `/mnt/nvme` by `racecar disk setup`,
holds the data root; see [Data root](#data-root).

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

The Pi drives no display bus directly. `dotmatrix_node` rasterizes frames in
software and hands them to `pit_node`, which forwards them to the Teensy; the
Teensy owns the MAX7219 chain.

## Node inventory

Eight nodes ship in this package (`setup.py` console scripts):

| Node | Responsibility |
|---|---|
| `pit_node` | Sole owner of the Teensy UART. Encodes drive commands, decodes telemetry, forwards display and LED frames (a fresh low-battery alert frame first, then the user's frame, then the override flash), and integrates odometry (`odometry.py`) into `odom -> base_link`. |
| `mux_node` | Arbitrates teleop against autonomy, enforces speed and steering limits, gates on the controller or on a live FlySky transmitter, zeroes output when the joystick drops. |
| `throttle_node` | Clamps `/mux_out` to `[-1, 1]` and applies per-direction speed and steering caps onto `/motor`; while `/autonomy/status` reports mapping, clips speed to `mapping_speed_cap` and steering to `mapping_steering_cap`; clips speed to `speed_cap` at all times (1.0, off). The Teensy maps the normalized command to PWM. |
| `gamepad_node` | Maps `/joy` axes into an Ackermann drive command. |
| `imu_fusion_node` | Merges the RealSense and LSM9DS1 IMUs into one stream; passes through when only one is live. |
| `dotmatrix_node` | Renders text, student pixel buffers, a startup splash, or a mode glyph into a display frame. |
| `edgetpu_node` | Runs object detection on the color stream via the Coral Apex. |
| `autonomy_node` | The SLAM interface: `/autonomy/pose`, `/autonomy/status`, `/autonomy/save_map`, `/autonomy/autosave`, `/autonomy/global_localization`; autosaves while mapping, keeps the last pose while localizing. Started by `autonomy.launch.py`, not by teleop. |

Four external nodes complete the graph: `joy_node` (`joy`), `sllidar_node`
(`sllidar_ros2`), `realsense2_camera_node` (`realsense2_camera`), and
`robot_state_publisher`, which publishes the static frame tree (see
[Frame tree](#frame-tree)).

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

### Safety-controller override

The 8080 dashboard can hold the gate with no controller. Its toggle starts a
session; the 8080 page and the three driving lab pages (`scripts/web/override.js`)
each post a keepalive to `/api/override` every second, which never starts one.
The lab pages call 8080 from their own ports, so the reply carries a CORS
header for those origins only (`override_origin`). `OverrideSession` in
`scripts/dashboard.py` publishes a random nonzero session number on
`/mux/override` (`std_msgs/UInt32`) at 5 Hz; 0 means off. The session lapses
2 s after the last keepalive, so closing every page ends it.
`SafetyOverride` in `mux_node` forwards `/drive` while a session number is
under `override_timeout_sec` old. LB held selects `/gamepad_drive`; a press of
any other button cancels that session number, and only a new one turns the
override back on. `mux_node` reports the state on `/mux/override_active`
(latched). The dashboard ends its side when the mux reports off, and when the
mux never confirms within 3 s. `dotmatrix_node` shows the AUTO glyph and
`pit_node` sets the auto mode bits and flashes the LED strip amber at 2 Hz
unless a program frame is fresh. Nothing is persisted; a teleop restart ends
it. No extra speed cap applies.

### Features

`scripts/features.py` holds the choice of teleop subsystems the 8080 Features
tiles control: camera (`realsense`) and detections (`edgetpu`, which needs the
camera). It is written to `~/.config/racecar/features.conf` with the boot id,
and ignored after a reboot, so each boot starts with the camera on and
detections off. `scripts/launch_teleop.sh` appends the matching
`<subsystem>_enable` arguments unless they were given, and records them in
`launch_args`, so the watchdog and the status cards treat an off subsystem as
turned off. A change on the page (`apply_features` in `scripts/dashboard.py`)
stops a running autonomy run, restarts teleop (allowed by
`scripts/polkit/49-racecar-teleop.rules`), waits for the sensors, and runs
`racecar map resume`. Resume continues a mapping run from its autosave and
loads a localization map from the last pose. The autonomy tile runs the same
`racecar map` commands: resume, stop, and save then stop.

### Lab dashboards

Four dashboards in `scripts/dashboards/` run as `racecar-*` units, installed
by `scripts/setup_dashboards.sh`: `webteleop` on 8081, `linefollow` on 8082,
`wallfollow` on 8083 and `camlabel` on 8085, beside the driver's own dashboard
on 8080 and the map viewer on 8084. The first three publish `/drive` and so
are mutually exclusive; a second publisher fights the mux, and both
`racecar service start` and the 8080 apps page (`open_app` in
`scripts/dashboard.py`) keep one at a time. Autonomy is exclusive with them
too, since a driving app beside the localizer or slam_toolbox leaves no CPU:
opening one stops autonomy (saving a mapping run first), and every
`racecar map` start, load, or resume stops them (`DRIVING_APPS` in
`scripts/map_control.py`). The pages ask before either. `camlabel` only reads the camera.
The apps page starts a stopped app through systemd and opens it once its port
answers (`/launch`, `/api/ready`); Clear apps stops them all. Both are allowed
for these four units by `scripts/polkit/49-racecar-dashboards.rules`. The map
viewer and JupyterLab start at boot and are left running. `setup_dashboards.sh` also removes
units left by the three retired dashboards (`eps`, `pursuit`,
`smartfollow`). Internals, data pipelines and the lidar convention:
`docs/dashboards.md`.

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
  /dotmatrix/pixels ├─▶ dotmatrix_node ─ /dotmatrix/frame ─┐
  /joy ─────────────┘   (glyph fallback)                    │
  battery_guard ─ /dotmatrix/alert ─────────────────────────┤
                                                            ├─▶ pit_node ─▶ Teensy ─▶ MAX7219 x3, LED strip
  /led/pixels (user code, teleop dashboard) ────────────────┤   (alert > user frame > override flash)
  battery_guard ─ /led/alert ───────────────────────────────┘
```

`dotmatrix_node` uses `luma` only for font rasterization. Text times out and
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
| `/mux/override` | `std_msgs/UInt32` | `dashboard.py` | `mux_node` |
| `/mux/override_active` | `std_msgs/Bool` (latched) | `mux_node` | `dashboard.py`, `dotmatrix_node`, `pit_node` |
| `/motor` | `ackermann_msgs/AckermannDriveStamped` | `throttle_node` | `pit_node` |
| `/imu/lsm9ds1`, `/imu/lsm9ds1/raw` | `sensor_msgs/Imu` | `pit_node` | `imu_fusion_node`, `calibrate_imu.py` |
| `/mag`, `/mag/raw` | `sensor_msgs/MagneticField` | `pit_node` | `calibrate_mag.py` |
| `/imu/realsense` | `sensor_msgs/Imu` | `realsense2_camera_node` | `imu_fusion_node`, `calibrate_realsense_imu.py` |
| `/imu/fused` | `sensor_msgs/Imu` | `imu_fusion_node` | user code |
| `/encoder/speed` | `std_msgs/Float32` | `pit_node` | user code |
| `/battery/voltage`, `/battery/current` | `std_msgs/Float32` | `pit_node` | user code |
| `/battery/state` | `sensor_msgs/BatteryState` (1 Hz means; discharge current negative) | `pit_node` | dashboard, `battery_guard.py` |
| `/battery/estimate` | `diagnostic_msgs/DiagnosticStatus` (latched, 1 Hz) | `battery_guard.py` | dashboard |
| `/battery/cancel_shutdown` | `std_msgs/Empty` | dashboard (Cancel shutdown) | `battery_guard.py` |
| `/rc/channels` | `std_msgs/Float32MultiArray` | `pit_node` | `mux_node` (when `rc_authority_enable`), user code |
| `/rc/link` | `std_msgs/Bool` | `pit_node` | `mux_node` (when `rc_authority_enable`) |
| `/odom` | `nav_msgs/Odometry` | `pit_node` | lab dashboards, user code |
| `/odom/integrated`, `/tf` (`odom -> base_link`) | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | `pit_node` | user code |
| `/camera/color` | `sensor_msgs/Image` | `realsense2_camera_node` | `edgetpu_node`, `webteleop`, `linefollow`, user code |
| `/camera/depth` | `sensor_msgs/Image` | `realsense2_camera_node` | `webteleop`, user code |
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` | `webteleop`, `wallfollow`, user code |
| `/edgetpu/inference` | `vision_msgs/Detection2DArray` | `edgetpu_node` | `webteleop`, user code |
| `/dotmatrix/text`, `/dotmatrix/pixels` | `std_msgs/String`, `UInt8MultiArray` | user code | `dotmatrix_node` |
| `/dotmatrix/frame`, `/led/pixels` | `std_msgs/UInt8MultiArray` | `dotmatrix_node`, user code | `pit_node` |
| `/dotmatrix/alert`, `/led/alert` | `std_msgs/UInt8MultiArray` (8 x 24 pixels, one byte each; 84 RGB triplets) | `battery_guard.py` | `pit_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | camera, `edgetpu_node` | dashboard, `diagnose.py` |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher`, `realsense2_camera_node` | TF consumers, viewers |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | viewers (RViz, Foxglove) |

`pit_node` stamps its telemetry topics (`/imu/lsm9ds1`, `/mag`, their `/raw`
twins, `/odom`) with the Teensy's sample time mapped onto ROS time
(`racecar_neo_ros2_driver/teensy_clock.py`), not the Pi's receive time;
`use_teensy_stamps: false` in `config/pit.yaml` reverts. `/encoder/speed` is
a bare `Float32`; a consumer that needs the stamp reads `/odom`.

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
├── frames.launch.py             ─┐
├── imu_fusion.launch.py          │
├── lidar.launch.py               │ gated by <name>_enable launch args
├── realsense.launch.py           │
├── edgetpu.launch.py  (+3.0 s)   │ stagger lets camera topics appear first
└── dotmatrix.launch.py          ─┘
```

Per-node files build on `single_node_launch()` in
`racecar_neo_ros2_driver/launch_common.py`, which declares a `<name>_config`
launch argument for the node's YAML and appends the car profile's
`params/<name>.yaml` when it exists (`local_overrides()`, read at launch time).
Two load extra files: `pit.launch.py` adds `lsm9ds1_cal.yaml` and
`lsm9ds1_mag_cal.yaml`, and `imu_fusion.launch.py` adds `realsense_cal.yaml`,
each followed by its profile override when present.
`realsense.launch.py` wraps the vendor launch and applies the topic remaps.
`frames.launch.py` does not use `single_node_launch()`; see
[Frame tree](#frame-tree).

`autonomy.launch.py` runs beside teleop, not inside it, and reads `/scan` and
the `odom -> base_link -> laser` transforms teleop publishes. Its `mode`,
`localizer`, `map`, and `pose` arguments select the nodes (`node_specs()`):

```
autonomy.launch.py
├── mode:=mapping map:=NAME         slam_toolbox (async) + lifecycle_manager_slam
├── mode:=localization map:=NAME
│   ├── localizer:=amcl (default)  map_server + amcl + lifecycle_manager_localization
│   └── localizer:=slam_toolbox    slam_toolbox (localization) + lifecycle_manager_slam
└── every mode                     autonomy_node
```

Each publishes `map -> odom`. slam_toolbox, AMCL, and map_server are lifecycle
nodes; a Nav2 lifecycle manager configures and activates them. A node that
exits ends the launch. Saved maps live under `maps/<name>/` in the data root
(`racecar_neo_ros2_driver/maps.py`; `RACECAR_MAP_ROOT` overrides the folder):
`<name>.posegraph` and `<name>.data` from slam_toolbox, `<name>.pgm` and
`<name>.yaml` from `map_saver_cli`. The validation scripts in
`scripts/validation/` run test sessions on top of this launch.

### Autonomy interface

`autonomy_node` (`racecar_neo_ros2_driver/autonomy_node.py`; logic in
`autonomy.py`; settings in `config/autonomy.yaml`) is the one interface for
the CLI, the library, and dashboards, the same whichever localizer runs:

| Name | Type | Behavior |
|---|---|---|
| `/autonomy/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map -> base_link` at `pose_rate_hz` (20), with the localizer's covariance |
| `/autonomy/status` | `diagnostic_msgs/DiagnosticStatus`, transient local | 1 Hz and on change of `localized`. Level OK when localized. Keys: `mode` (`mapping`, `localization`, or `off` on exit), `map`, `localizer`, `localized`, `tf_age_ms`, `scan_age_ms`, `pose_sd_m`, `pose_sd_deg`, `last_save`, `last_autosave`, `resumed_from_autosave`, `pose_source`, `scans_processed`, `scan_fit`, `session` |
| `/autonomy/save_map` | `std_srvs/Trigger` | Mapping: pose graph, `.pgm`/`.yaml`, and `meta.yaml` into `<map>/`; returns when written. Otherwise fails ("not mapping") |
| `/autonomy/autosave` | `std_srvs/Trigger` | Mapping: the same set into `<map>/autosave/` (built in `autosave.tmp/`, then swapped in), with `autosave.yaml` (pose, session) |
| `/autonomy/global_localization` | `std_srvs/Trigger` | Localization with AMCL: search the whole map, as `pose:=global` does at start (`localized` false until it converges). Otherwise fails |

`localized`: both modes need `map -> odom` and a scan within
`scan_timeout_sec` (0.5 s), so a stopped lidar clears it within about half a
second. Localization also needs the localizer's pose spread within
`localized_max_xy_sd_m` and `localized_max_yaw_sd_deg`, and with
`pose:=global` (AMCL only) stays false until AMCL's search converges;
`autonomy_node` starts that search through `reinitialize_global_localization`.
It also needs the live scan to fit `/map`: `fit_rate_hz` times a second,
`fit_beams` returns are placed at the localizer's pose and the share within
0.1 m of a wall is averaged over `fit_window_sec` (`scan_fit` in the status;
`autonomy.FitGate`). `localized` drops under `localized_min_fit` and returns
at `localized_enter_fit`. The spread alone passes a car carried to another
spot, since the particle cloud stays tight at the old pose; the fit does not.
While mapping it autosaves every `autosave_period_sec` (60 s); while localized
it writes `<map>/last_pose.yaml` every `last_pose_period_sec` (5 s) and on
exit. On exit (SIGINT or SIGTERM) it publishes `mode: off`, waiting for
subscribers to acknowledge it, which lifts the mapping speed and steering caps.

`racecar map` (`scripts/racecar_map.py`) is the operator front end. Its
lifecycle logic is in `scripts/map_control.py`, which imports without the
argument parser. A start or load checks the request and that teleop is
active, stops `racecar-autonomy.service`, resolves the start pose from the
files the stop left, writes `autonomy.conf` through a rename, starts the unit,
and waits for `/autonomy/status` to report the new `session`:

```
racecar map start|load
  └─ map_control: check ─▶ systemctl stop ─▶ start pose ─▶ autonomy.conf ─▶ systemctl start
                                                                              │
     wait for /autonomy/status with the new session (mapping: localized) ◀────┘
```

Start pose for `load`: `--pose`, `--global`, `<map>/last_pose.yaml`, the
`start_pose` in `meta.yaml` (the car's pose when the map was begun), then the
map origin. `--from` extends a saved map: `autonomy_args.py` passes
`extend:=<stem>`, and slam_toolbox loads that pose graph at the start pose
and matches the first scan against the saved scans near it; a same-session
autosave still takes priority (crash resume). `meta.yaml` also records
`last_change`, the time of the last scan added; an autosave whose
`last_change` is newer than the saved set's is unsaved work, which `stop`
reports and `recover` copies over the saved files (`autonomy.map_state`). A
save into a folder holding another run's map moves those files to
`<map>/previous/<time>/` first, and `start --force` moves the whole folder to
`<map_root>/replaced/`. `list`, `info`, and `recover` read the folder without
ROS.

The web map viewer (`scripts/mapview.py`, page `scripts/mapview.html`, unit
`racecar-mapview.service`) is the browser front end on port 8084: the live
map, the car's pose and trail, the scan, and status, with the same controls.
Start, save, stop, load, and keep run `racecar map` as a child process, one at
a time, and the page shows its output; set pose publishes `/initialpose` with
the spread `load` uses, and Find me calls `/autonomy/global_localization`. The
page publishes no `/drive`. ROS subscriptions open on the first request and
close 10 s after the last one; the map list, thumbnails, and saved maps are
read from the map folder without ROS.

```
browser ──HTTP poll 10 Hz──▶ mapview.py ──subscribes while polled──▶ /autonomy/status, /autonomy/pose, /map, /scan
                                 │
                                 └── buttons ──▶ racecar_map.py (child) ──▶ map_control ──▶ systemctl, /autonomy/*
```

## Frame tree

```
map ──► odom ──► base_link               map -> odom while autonomy runs (slam_toolbox,
                                         or AMCL); odom -> base_link from pit_node
base_link ──► laser                      lidar; yaw 180 deg, the mount faces aft
          ├─► imu_link                   IMU position; same axes as base_link
          └─► camera_bottom_screw_frame  camera mount; pitch and roll act here
                └─► camera_link ──► RealSense subtree (published by the camera driver)
```

REP-103 throughout: body frames x forward, y left, z up; `base_link` is the
floor point under the rear axle midpoint. The IMU topics (`/imu/lsm9ds1`,
`/imu/fused`, `/mag`, and their `/raw` forms) carry body axes too, in
`imu_link`, which has no rotation: `config/pit.yaml` remaps the LSM9DS1 and
`config/imu_fusion.yaml` remaps the RealSense IMU. Only `/imu/realsense`, the
camera driver's own topic, stays in the camera's optical axes (x right, y
down, z forward). `odom -> base_link` comes from the odometry in `pit_node`
(`racecar_neo_ros2_driver/odometry.py`, settings in `config/odom.yaml`): each
telemetry frame's encoder speed and gyro yaw rate (body z) are integrated as a unicycle at `base_link`, with a gyro bias
re-estimated while the car is stopped, and published at 50 Hz. It runs
inside `pit_node` because a separate node would deserialize messages
`pit_node` already holds decoded.

`urdf/racecar_neo.urdf.xacro` defines the tree. `launch/frames.launch.py` loads
`config/frames.yaml` and the profile's `params/frames.yaml` through
`racecar_neo_ros2_driver/frames.py`, converts millimeters and degrees to meters
and radians, renders the xacro, and starts `robot_state_publisher` with
`respawn` (2 s delay, 5 retries). The watchdog does not supervise it: the node
publishes `/tf_static` once, latched, and the watchdog's rate check would read
it as dead.

## Process model

Seven core systemd units, all `User=racecar`:

```
racecar-data.service        (oneshot at boot, before the others)

racecar-teleop.service ──── Wants= ───▶ racecar-watchdog.service
        ▲                                        │
        └────────────── BindsTo= ────────────────┘

racecar-dashboard.service   (independent, port 8080)
racecar-jupyter.service     (independent, port 8888)
racecar-mapview.service     (independent, port 8084)
racecar-battery-guard.service (independent; Restart=always)
```

`Wants=` pulls the watchdog up when teleop starts; `BindsTo=` takes it down
when teleop stops, so the supervisor never outlives the thing it supervises.
`racecar-data.service` writes the data root to `/run/racecar/`; teleop,
dashboard, jupyter, mapview and autonomy order after it and load
`RACECAR_DATA_DIR` from `/run/racecar/environment` (`EnvironmentFile=`).

`racecar-autonomy.service` runs SLAM on top of teleop, installed but not
enabled (mapping at boot would build an unsaved map and take CPU from every
lab):

```
racecar-teleop.service ◀── After=, BindsTo=, PartOf= ── racecar-autonomy.service
```

It stops when teleop stops and restarts when teleop restarts. Its mode comes
from `~/.config/racecar/autonomy.conf` (`EnvironmentFile`; `AUTONOMY_MODE`,
`AUTONOMY_MAP`, `AUTONOMY_SESSION`, `AUTONOMY_POSE`, `AUTONOMY_POSE_SOURCE`,
`AUTONOMY_LOCALIZER`, `AUTONOMY_FOXGLOVE`). `scripts/launch_autonomy.sh` turns
it into `autonomy.launch.py` arguments with `scripts/autonomy_args.py` and logs
to `~/logs/latest/autonomy_<time>/`. `ExecStop` runs `scripts/autonomy_stop.py`,
which calls `/autonomy/autosave` while slam_toolbox is still alive, so a stop,
a teleop restart, or a teleop shutdown keeps the map. `Restart=always`, since
`ros2 launch` exits 0 when slam_toolbox dies and systemd never restarts after
a requested stop; three failed starts in 2 minutes give up. A start whose map
holds an autosave of the same `AUTONOMY_SESSION` resumes from it
(`autonomy.resume_point`): slam_toolbox loads the autosaved pose graph and
continues from the newer of the autosave's pose and `mapping_pose.yaml`,
which `autonomy_node` writes each second, so a crash loses at most the time
since the last autosave. `scripts/polkit/49-racecar-autonomy.rules` lets the
`racecar` user start, stop, and restart this one unit without sudo.

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
topic advertising, and the node's entry-point path among the process command
lines (one `/proc` scan per 5 s poll). A node counts as failed when either
signal fails. `/scan` also carries a freshness window (`freshness_sec`, 5 s),
so a lidar that stays advertised but stops publishing is restarted too.

Each node carries a state (`NodeState`, advanced by `step()`):

```
starting ──topic seen──► healthy ◄──────── 60 s healthy clears the count
   │ 60 s grace            │ fails                     ▲
   └──────────────► restarting ──5 restarts, none held 60 s──► gave_up
                      │  ▲ backoff 30, 60, 120, 240 s             │
          device gone ▼  │ device back (count reset)              │ device gone
                   no_device ◄────────────────────────────────────┘
disabled: <name>_enable:=false at launch (recorded by launch_teleop.sh)
```

The startup grace covers the watchdog's first polls, which run before DDS
discovery lists teleop's topics; without it every node read as down and was
killed and relaunched as a watchdog child. A node that gave up is restarted
again when its device is unplugged and replugged, when it recovers on its own,
or when teleop restarts. Log lines are written on state changes, with a
reminder every 10 min while a node is in `no_device`, `restarting`, or
`gave_up`; each node's restart output is appended to `restart_<node>.log`.
Every poll writes `watchdog_state.json` to the session log directory (by
rename); `racecar status` turns it into the `watchdog nodes` row (`gave_up` is
FAIL, `no_device` and `restarting` WARN) and the dashboard into card colors.
Additional behavior:

- SIGTERM then SIGKILL escalation
- FastRTPS shared-memory orphan sweep every 60 s
- Pi 5 PMIC under-voltage alarm via the `rpi_volt` hwmon's `in0_lcrit_alarm`,
  found by driver name because hwmon numbering is not stable
- Hardware awareness: restart is skipped when the underlying device is absent,
  so an unplugged lidar does not produce a restart loop
- `robot_state_publisher` (frame tree) is not supervised here; launch respawns
  it, since its latched `/tf_static` has no rate to check

`edgetpu_node` and `dotmatrix_node` are not supervised; the dashboard shows
their cards as `unsupervised` rather than dead when they are down.

`scripts/dashboard.py` serves `/` and `/api/status` on port 8080. Node cards
resolve from topic advertisement; rate rows come from raw subscriptions
(counted without deserializing) except the three RealSense streams, which are
read from the camera's `/diagnostics` output. System Health reports the data
drive (`data_root_health`), the RTC backup cell voltage, and the sticky PMIC
under-voltage alarm. The Autonomy card
reads only `/autonomy/status` (1 Hz, latched) for mode, map, localized, scan
fit, and spread (`autonomy_view`), and links to the map viewer; the 20 Hz pose
is left to the viewer, which subscribes only while a browser is open.

`scripts/diagnose.py` is the one-shot counterpart behind `racecar status`,
where the dashboard is the continuous view. It opens every subscription at
once and shares a single 5 s sample window rather than measuring topics in
sequence, and runs the host checks on a worker thread beside it. Checks are
grouped as devices, sensors, actuators, autonomy, system, services and
network; rate
checks compare observed Hz against a per-topic floor rather than testing for
presence. Each rate row also carries one decoded sample from its topic in a
tab-separated column, and a payload outside its expected range (gravity,
battery voltage, channel count) sets the row's status. The autonomy section
reports the unit's state; while it runs, `/autonomy/pose` and
`/autonomy/status` join the window, giving the pose rate, mode, localized,
scan fit, spread, and saves (`autonomy_checks`). The RealSense streams
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

## Battery guard

`scripts/battery_guard.py`, run by `racecar-battery-guard.service` from boot,
apart from teleop, so a teleop restart never stops it. Modules:

| File | Role |
|---|---|
| `racecar_neo_ros2_driver/battery.py` | Charge from the measured curve (per cell), power and time left over 5 minutes, cell count, `CapacityLearner`; no ROS |
| `scripts/battery_guard.py` | `Guard` state machine (ok, warn, countdown, off), `/battery/estimate`, the CSV log, the alert frames at 20 Hz |
| `scripts/battery_alert.py` | Red line on every terminal of the user and in the tmux status bar (default and `racecar-terminal` sockets) |
| `scripts/battery_display.py` | Dot-matrix and LED frames for each level |
| `scripts/safe_shutdown.py` | Shutdown steps, shared with the 8080 Turn Off tile |

```
  /battery/state ─▶ battery_guard ─┬─▶ /battery/estimate ─▶ dashboard ─▶ 8080 pills, banner; /api/battery ─▶ /ui/battery.js
       (1 Hz)                      ├─▶ terminals, tmux status bar
                                   ├─▶ /dotmatrix/alert, /led/alert ─▶ pit_node
                                   ├─▶ <data root>/health/battery/<start>_guard.csv
                                   └─▶ safe_shutdown ─▶ racecar log stop, driving apps, autonomy, teleop, power off
  dashboard (Cancel shutdown) ─▶ /battery/cancel_shutdown ─▶ battery_guard
```

Levels: warn at `warn_min` (15) minutes left, held 60 s. Countdown (60 s)
when the 30 s median voltage reaches `floor_cell_volts` times the cell count
(7.1 V on 2S), or, with `shutdown_enabled`, at `shutdown_min` (5) minutes
left held 60 s; only the second can be canceled, once per boot. Readings
that are stale or outside 3.0 to 4.3 V per cell never warn or start a
countdown; a countdown already running finishes. After the countdown the
guard holds 2 s for SHUT and DOWN on the dot matrix, then runs the steps.
Stopping teleop ends `pit_node`, so the displays go dark before power off.

`49-racecar-power.rules` lets the racecar user stop teleop and power off
with other sessions open or inhibitors held; the other rules cover the lab
apps and autonomy. The power switch is the only battery disconnect: after
power off, the PIT board (its own 5 V regulator from the main bus) and the
halted Pi still draw about 100 to 200 mA.

## Terminal app

`scripts/terminal.py` on port 8086, run by `racecar-terminal.service` (not at
boot; started from the 8080 Terminal tile through
`49-racecar-dashboards.rules`). Standard library only.

```
  browser ─ :8086 ─▶ terminal.py ─┬─ login (PAM, racecar-terminal) ─▶ session cookie
                                  ├─ /api/layout, /api/presets/*, /api/scrollback
                                  └─ /pane/<id>/ ─▶ /run/racecar-terminal/ttyd.sock ─▶ ttyd
                                                        ─▶ tmux -L racecar-terminal new-session -A -s rcn-<id>
```

- `ttyd` is one child process on a Unix socket in a 0700 folder (systemd
  `RuntimeDirectory`), so 8086 is the only TCP listener. Its flags:
  `-b /pane -W -a -d 3`.
- The proxy checks the session, the terminal id (`[a-z0-9]{1,16}`), and on a
  WebSocket upgrade that `Origin` matches `Host`. It replaces the query with
  `?arg=rcn-<id>`, so a client cannot pass tmux arguments, then copies bytes
  both ways without parsing frames. Reads block; a watcher thread closes
  both sockets when the session ends.
- Each terminal is a tmux session `rcn-<id>` on the socket `racecar-terminal`
  (`config/terminal.tmux.conf`), started in `~/jupyter_ws`. The tmux server
  runs in the unit's cgroup (`KillMode=control-group`), so stopping the app
  ends every terminal.
- Login: the password of the account running the unit, through PAM service
  `racecar-terminal` (`scripts/pam.d-racecar-terminal`, 3 s fail delay).
  Five failures lock the login for 60 s. Sessions are in memory and end
  after 30 minutes without requests or traffic. Logins come from the access
  point subnet (10.42.0.0/24) and the car only, unless `car.yaml` sets
  `terminal.allow_lan`.
- Layout: a tree of terminals and splits in the profile's
  `terminal/layout.json`, versioned; a save over an older version returns
  409 and the current layout. The page places every terminal absolutely
  from the tree, since moving an iframe reloads it.
- Presets: layouts whose terminals may carry a start command.
  `config/terminal_presets.json` ships the built-ins; the profile's
  `terminal/presets.json` holds the car's. Applying one creates each tmux
  session with `new-session -d`, types its command with `send-keys -l`, saves
  the layout, and closes the old terminals.
- Export: `tmux capture-pane -p -J -S -`, returned to the page and written to
  `<data root>/terminal/<stamp>-<title>.txt`.
- The battery guard's low-battery line reaches the terminals through the
  tmux status bar on this socket.

## Configuration

One YAML per node under `config/`, keyed by node name and loaded through the
node's `<name>_config` launch argument.

| File | Node | Contents |
|---|---|---|
| `pit.yaml` | `pit_node` | serial device, axis order and polarity, topic names, CRC toggle |
| `mux.yaml` | `mux_node` | speed and steering limits, arming buttons, joystick timeout, RC gate |
| `throttle.yaml` | `throttle_node` | forward, reverse, and steering caps as fractions of full scale; `mapping_speed_cap`, `mapping_steering_cap`, `speed_cap` |
| `gamepad.yaml` | `gamepad_node` | axis indices and signs |
| `imu_fusion.yaml` | `imu_fusion_node` | source topics, output topic, rate, staleness timeout |
| `lidar.yaml` | `sllidar_node` | serial device, baud, scan mode |
| `dotmatrix.yaml` | `dotmatrix_node` | module count, refresh rate, scroll and pixel timeouts, splash, mode buttons |
| `edgetpu.yaml` | `edgetpu_node` | model path, labels, score threshold, image topic, inference rate |
| `lsm9ds1_cal.yaml`, `lsm9ds1_mag_cal.yaml` | `pit_node` | IMU bias and iron correction, zero defaults |
| `realsense_cal.yaml` | `imu_fusion_node` | RealSense IMU bias, zero defaults |
| `odom.yaml` | `pit_node` (loaded by `pit.launch.py`) | odometry: yaw axis and sign, speed and yaw-rate scales, publish rate, stopped-car bias settings |
| `autonomy.yaml` | `autonomy_node` | pose rate, autosave and last-pose periods, `localized` limits, scan-fit rate and thresholds |
| `slam_mapping.yaml`, `slam_localization.yaml` | `slam_toolbox`, `lifecycle_manager_slam` | frames, laser range 0.3 to 12 m, node spacing, loop closure |
| `amcl.yaml` | `amcl`, `map_server`, `lifecycle_manager_localization` | motion and laser models, particles, beam skipping, update distances |
| `foxglove.yaml` | `foxglove_bridge` (autonomy launch, when on) | port, topic whitelist, `/initialpose` as the only client topic |
| `foxglove_layout.json` | Lichtblick, imported by hand (not a node file) | 3D view, status table, pose plot |
| `battery.yaml` | `battery_guard.py` | capacity, measured curve, cell count, floor per cell, warning and shutdown minutes, capacity learning, `shutdown_enabled` |
| `terminal.tmux.conf`, `terminal_presets.json` | `scripts/terminal.py` (not node files) | tmux settings for the terminal app; built-in presets |
| `frames.yaml` | `frames.launch.py` (not a node parameter file) | frame offsets in mm and deg, fleet base values; see [Frame tree](#frame-tree) |

Per-car values (calibration, an enabled RC gate) live in the car profile
(`racecar_neo_ros2_driver/car_profile.py`), `~/.config/racecar` by default
(`RACECAR_CONFIG_DIR`, else `$XDG_CONFIG_HOME/racecar`), outside the driver
folder. Each launch file loads `params/<name>.yaml` from it after the committed
file of the same name, at launch time, so a teleop restart applies a new value
with no rebuild. The profile also holds `car.yaml` (car name, profile
version), `dashboards/<name>.yaml` (lab dashboard tuning), `autonomy.conf`,
`features.conf`, the network settings file, and `history/<stamp>/` (the
previous copy of each file a write replaced). `config/*.local.yaml` in the
driver folder is not read; `racecar profile check`, run by
`launch_teleop.sh`, logs a warning per file, and `racecar profile migrate`
moves them in.

## Data root

`racecar_neo_ros2_driver/data_root.py` holds the rule for where the car's
data goes: `/mnt/nvme/data` when the NVMe is mounted at `/mnt/nvme` and that
folder can be written, else `~/data` on the SD card. Settings stay in the car
profile.

```
racecar-data.service (boot) ──▶ /run/racecar/data_root    (path)
                           └──▶ /run/racecar/environment  (RACECAR_DATA_DIR=...)
                                   │
   systemd units (EnvironmentFile=) ┤
   /etc/profile.d/racecar-data.sh   ┤──▶ RACECAR_DATA_DIR
   racecar-tool.sh                  ┘
```

`data_root.current()` reads `RACECAR_DATA_DIR`, then the run file, then
applies the rule itself, once per process; `data_dir(sub)` returns a folder
under the root, created if absent. The student library's
`rc_utils.get_data_dir(subfolder)` is the notebook counterpart.

| Folder | Writer |
|---|---|
| `bags/` | `scripts/racecar_log.py` (`RACECAR_LOG_DIR`, `--dir` override) |
| `maps/` | `maps.py`, `scripts/map_control.py` (`RACECAR_MAP_ROOT` overrides) |
| `datasets/camlabel/` | the camlabel dashboard |
| `dashboards/<name>/` | lab dashboard CSV logs |
| `slam_eval/` | `scripts/validation/slam_eval.py`, `slam_session.py` |
| `images/` | lab programs, through the library's `rc_utils.get_data_dir("images")` |
| `health/` | `racecar health` reports; `battery/`: battery guard logs |
| `terminal/` | terminal app exports |

The root is fixed for the boot. `data_root.session_problem()` makes a new
`racecar log start` or `racecar map start`, `load`, or `resume` refuse when
the root cannot be written, so a drive that drops out after boot never splits a
session across two folders. `scripts/racecar_disk.py` backs `racecar data`
and `racecar disk` (status, setup, format, check); privileged steps run
through sudo. `diagnose.check_data_root` gives the `racecar status` row, and
`dashboard.data_root_health` the 8080 Data indicator.

## Calibration data flow

The committed calibration files hold zero defaults. Each utility subscribes to
a raw topic, fits, and writes the per-car result to `params/<name>.yaml` in
the car profile (`calibrate_common.write_local_yaml()`,
`update_local_values()`); the file it replaces goes to `history/`. It writes
only keys the node declares, and writes nothing (exit non-zero) when the fit
fails or there is too little data.

```
  /imu/lsm9ds1/raw ─▶ calibrate_imu.py ──────────▶ params/lsm9ds1_cal.yaml ─────┐
  /mag/raw ────────▶ calibrate_mag.py ───────────▶ params/lsm9ds1_mag_cal.yaml ─┴─▶ pit_node
  /imu/realsense ──▶ calibrate_realsense_imu.py ─▶ params/realsense_cal.yaml ─────▶ imu_fusion_node
  /scan ───────────▶ calibrate_lidar.py ─────────▶ params/frames.yaml ────────────▶ frames.launch.py
  /imu/lsm9ds1, ───▶ calibrate_odom.py ──────────▶ params/odom.yaml ──────────────▶ pit_node (odometry)
  /odom/integrated
  /scan, ──────────▶ calibrate_steering.py ──────▶ params/throttle.yaml ──────────▶ throttle_node
  /odom/integrated
  /scan, ──────────▶ calibrate_dynamics.py ──────▶ params/dynamics.yaml ──────────▶ the simulator
  /odom/integrated
```

`calibrate_lidar.py` writes a plain `params/frames.yaml` (no node key) and
keeps any other keys already in it.

The tools live in `scripts/calibration/`; `calibrate.py` (behind `racecar
calibrate`) runs them in order (`dynamics` only when named), reports old and
new values, and offers a teleop restart. The IMU tools fit accelerometer bias and
scale per axis from the six orientations, applied by `pit_node` and
`imu_fusion_node` as (raw - bias) * scale (`accelerometer.scale`,
`realsense_accel_scale`, unit by default). After a run, restart teleop; no
rebuild is needed. Each
tool's `--check` (run together by `racecar calibrate check`) verifies the
stored calibration against the live sensor: the IMUs at rest, the
magnetometer through one flat turn, and the lidar yaw, odometry, and steering
against scan matching while the car drives. `racecar profile export FILE`
saves the profile to one `.tar.gz` for a fresh image or SD card swap.

## Repository layout

```
racecar_neo_ros2_driver/
├── racecar_neo_ros2_driver/   node implementations, launch_common, pit_protocol,
│                              limits (shared clamp), frames (frames.yaml loader),
│                              car_profile (per-car settings in ~/.config/racecar),
│                              data_root (where the car's data goes)
├── launch/                    one file per subsystem plus teleop composite
├── config/                    one YAML per node, keyed by node name;
│                              frames.yaml (frame offsets); per-car
│                              overrides live in the car profile
├── urdf/                      racecar_neo.urdf.xacro (frame tree)
├── scripts/                   setup phases, racecar-tool.sh, watchdog, dashboard,
│                              diagnose, sysinfo, eth_monitor, wifi_scan,
│                              racecar_log, analyze_odom, dmatrix_patterns,
│                              racecar_map and map_control (`racecar map`),
│                              racecar_profile (`racecar profile`),
│                              racecar_disk (`racecar disk`, `racecar data`),
│                              autonomy unit launcher and stop step,
│                              systemd units, udev, polkit and modprobe rules,
│   │                          Coral overlay and gasket patch
│   ├── calibration/           calibrate.py (`racecar calibrate`), the
│   │                          calibrate_*.py tools, calibrate_common
│   ├── validation/            slam_session.py (one-terminal SLAM test session,
│   │                          offline replay), scan_fit.py (scan-to-map fit),
│   │                          slam_eval.py (single measurements), live_fit.py
│   │                          (live pose check), compare_maps.py (extension
│   │                          check); run from source
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
  package sits at the top level, the suite lives in `test/`, and the car's
  data lives in the data root outside the tree.
- `edgetpu_node` and `dotmatrix_node` run without watchdog supervision.
