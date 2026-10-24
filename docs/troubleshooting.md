# Troubleshooting

Failure modes that shaped the code, and the reasoning behind choices that look
arbitrary at the call site. Each section names the code it explains, so a
comment in the source can point here instead of carrying the full account.

## Contents

- [Diagnostic rate checks](#diagnostic-rate-checks)
- [CPU busy share](#cpu-busy-share)
- [eth0 addressing](#eth0-addressing)
- [AP isolation dispatcher](#ap-isolation-dispatcher)
- [NetworkManager authorization](#networkmanager-authorization)
- [Shell block rewriting](#shell-block-rewriting)
- [Desktop toggle scope](#desktop-toggle-scope)
- [Jupyter dependency pins](#jupyter-dependency-pins)
- [LSM9DS1 axis mapping](#lsm9ds1-axis-mapping)
- [EdgeTPU inference rate and threshold](#edgetpu-inference-rate-and-threshold)
- [Coral M.2 interrupts](#coral-m2-interrupts)
- [RealSense firmware flash privileges](#realsense-firmware-flash-privileges)
- [Depth view gaps](#depth-view-gaps)
- [raspi-config on Ubuntu](#raspi-config-on-ubuntu)
- [Gamepad hid_nintendo blacklist](#gamepad-hid_nintendo-blacklist)
- [Lidar /scan stalls](#lidar-scan-stalls)
- [Map quality](#map-quality)
- [Localization state](#localization-state)
- [Localizer choice](#localizer-choice)
- [Lab dashboards](#lab-dashboards)
- [WiFi persistence](#wifi-persistence)
- [Boot brownout with ethernet attached](#boot-brownout-with-ethernet-attached)
  - [Not a flat battery](#not-a-flat-battery)
  - [Root cause and repair](#root-cause-and-repair)

## Diagnostic rate checks

`scripts/diagnose.py`: `SENSOR_TOPICS`, `DEFAULT_WINDOW`, `sample_ros`.

Counting a topic means receiving it, so the diagnostic competes with the car
for the same CPU. `pit_node` reads the Teensy on a Python thread and loses
throughput under that competition, which through v0.8.0 made the sensor
section report the tool's own overhead as a hardware fault.

Two subscriptions dominated the cost:

- The RealSense color and depth streams, worth 40 percent of the PIT
  telemetry rate. Measured A/B on hardware over interleaved trials: 133.6 Hz
  mean without them, 79.9 Hz with. They are now read from `/diagnostics`,
  where the camera node publishes its own per-stream frequency and where
  `scripts/dashboard.py` has read them since v0.7.3.
- `/scan`, deserialized on arrival for its value check at 1080 ranges plus
  1080 intensities a message, which put a 7.2 Hz lidar at 4.6. Every
  subscription is raw now, and the one buffer a value check needs is
  deserialized after the window closes.

The window is 5 seconds because the PIT stream arrives in clumps and a short
window samples the clumping rather than the rate. Twelve trials per length on
`/imu/lsm9ds1` with nothing else subscribed:

| window | range | standard deviation |
| --- | --- | --- |
| 2.0 s | 88.2 to 231.5 Hz | 32.0 |
| 3.0 s | 144.0 to 196.3 Hz | 15.5 |
| 5.0 s | 146.5 to 157.3 Hz | 3.1 |

Nominal rates come from configuration where one is declared and from
measurement otherwise. Two were wrong before v0.8.1:

- The camera streams were declared at 25 and 12 fps against a note calling
  the gap to the configured 60 and 30 a known hardware shortfall. There is no
  shortfall; `/diagnostics` reports 59.0 and 29.6 fps. The 25 and 12 were the
  subscription cost above, recorded as a property of the camera.
- The lidar was declared at 8.0 Hz. With `scan_mode` unset in `config/lidar.yaml`
  the driver runs its "typical" mode at 7.2 Hz, so the floor sat 11 percent
  under the real rate and an ordinary dip failed a healthy unit.

The six PIT topics carry a 65 percent floor rather than 80. Their rate stays
load-dependent even with the diagnostic's own cost removed, and a car dipping
into under-voltage throttling drops further. At 88.4 Hz the floor passes
anything in the 90 to 110 Hz band while a halved frame rate (68 Hz) still
warns.

Below its floor a stream warns; below `STALL_HZ` (2 Hz) it fails. Through
v0.8.2 the floor itself was the fail line, so a Coral delivering 8.6 of its
15 Hz on a throttled car failed the run next to a dead link. A slow stream
still feeds the car; a stopped one does not, and the exit code now separates
them. A car whose stack is down entirely shows every topic as not published,
which fails rather than skips.

## CPU busy share

`scripts/diagnose.py`: `measure_cpu_busy`, `check_system`.
`scripts/sysinfo.py`: `read_cpu_times`, `read_arm_clock`.

Through v0.8.2 the system section reported the 1-minute load average divided
by the core count, which read 2.18x (8.71 on 4 cores) on 2026-09-22. Load
counts runnable and blocked threads, not CPU time, so it has no 100 percent
ceiling. That reading was real run-queue pressure, not an artifact: 95 percent
busy across all cores, 7 to 14 runnable threads, no I/O wait.

The cause was the clock. `vcgencmd get_throttled` read 0x50005, and the
firmware had capped the ARM at 1000 MHz of 2400 while cpufreq still reported
2400. The same teleop stack then needs about 2.4 times the CPU share it uses
at full clock. The under-voltage came from a bad supply wire on this car,
replaced 2026-10-03; see [Root cause and repair](#root-cause-and-repair). The
top consumers at the time, in percent of one core:

| process | CPU |
| --- | --- |
| `pit_node` | 71 |
| `edgetpu_node` | 68 |
| `imu_fusion_node` | 59 |
| `dashboard.py` | 47 |
| `realsense2_camera` | 41 |
| `mux_node` | 16 |
| `throttle_node` | 12 |

The row is now `cpu`: the busy share of all cores over one second, which
cannot pass 100 percent, and the firmware's ARM clock against its maximum.
The diagnostic's own CPU time over that second is subtracted, since its ROS
window runs alongside; with it included the same car read 95 percent, without
it 77. The row warns at 90 percent and never fails, because a saturated CPU
shows up as a fault only through the rates it slows, which the sensor rows
already report.

## eth0 addressing

`scripts/setup_eth.sh`.

eth0 previously carried a static address and a DHCP lease at once.
NetworkManager reconciles the whole IPv4 config for an interface on every
lease event, so the static was repeatedly torn down and re-added; in the field
the link drops periodically and returns only after the cable is reseated. The
modes are mutually exclusive now, and this script is the only writer of the
netplan file, so it and `racecar eth` cannot disagree.

Static is the default because a known address is what makes a car debuggable
on a bare switch. It carries no gateway or DNS in either family, so a static
car reaches the internet over wlan0 or not at all.

Removing the structural cause is not the same as proving the symptom gone. It
was periodic and recovered only when the cable was reseated, so a passing
afternoon says nothing; `scripts/eth_monitor.py` records addressing and link
state over days to turn "should be fixed" into evidence.

IPv6 keeps its addresses but never a default route in static mode. Router
advertisements would otherwise hand eth0 a v6 default route even with no v4
gateway, and since most large destinations are dual-stack a static car would
send most of its traffic out an interface the design treats as inert.
`ipv6.never-default` suppresses that route and nothing else. The kernel
`accept_ra` sysctls are not the lever: they read 0 on eth0 while the routes
are still `proto ra`, because NetworkManager handles RA itself.

## AP isolation dispatcher

`scripts/setup_networking.sh`, step 1.

NetworkManager runs dispatcher scripts through
`NetworkManager-dispatcher.service`, shipped enabled on Ubuntu Server but often
disabled on Desktop and Raspberry Pi OS images. Without it the script never
runs and the iptables isolation rules silently never apply, which is the
failure mode v0.0.6 hit on first install.

The service is D-Bus activated (`Type=dbus`, bus name
`org.freedesktop.nm_dispatcher`) and exits when idle, so it reads "inactive"
whenever no script is running. `is-active` is therefore the wrong probe;
`is-enabled` answers the question that matters, which is whether NetworkManager
can activate it on the next connection event.

## NetworkManager authorization

`scripts/polkit/49-racecar-network.rules`, installed by
`scripts/setup_user_env.sh`.

NetworkManager gates activation and profile edits behind polkit, and the stock
policy answers `auth`. polkit collects a password through an agent and an SSH
session has none, so `racecar wifi connect` failed at the activation call with
`Not authorized to control networking`, after the passphrase had been typed
and with nothing said about the remedy. A headless car driven from a terminal
is the normal case.

The rule grants five NetworkManager actions to the `sudo` group. The `49-`
prefix sorts it ahead of polkit's own `50-default.rules`, which is what lets
it answer first. Members of that group can already reach the same operations
through `sudo nmcli`, so this removes a password prompt rather than a
privilege boundary; the cost is that an unlocked shell on the car can change
its networking. That is the intended trade for a shared lab robot.

`racecar wifi` checks the permission before it prompts, so a car that has not
picked the rule up names the remedy instead of failing inside `nmcli`. Cars
imaged before v0.8.1 need one run of `setup_user_env.sh`; polkit re-reads
`rules.d` on change, so nothing restarts.

## Shell block rewriting

`scripts/setup_user_env.sh`: `replace_block`.

The managed `.bashrc` blocks hold no per-car state; every path is fixed or
resolved from `$HOME` at runtime. Each run therefore drops any existing copy
and writes the current one. A marker-present test would answer "was this ever
written" rather than "is this current", which is how the
`ROS_AUTOMATIC_DISCOVERY_RANGE` line reached only cars imaged after v0.7.3.

Hand edits inside a block do not survive a re-run. Personal settings belong
outside the markers.

## Desktop toggle scope

`scripts/racecar-tool.sh`: the `desktop` subcommand.

The boot target is the only lever and is sufficient alone. The display manager
unit is `static` (no `[Install]` section), so `systemctl enable` and `disable`
on it cannot work, and it is pulled in by `graphical.target` rather than by
its own enablement. Booting to `multi-user.target` never starts it.

The toggle is deliberately reboot-scoped: there is no `--now` or `isolate`
variant, so it can never tear down a desktop session out from under whoever is
sitting at it. Packages stay installed, so the toggle is reversible on a car
with no network.

## Jupyter dependency pins

`scripts/setup_jupyter.sh`: `LIB_DEPS`.

`matplotlib-inline` is pinned below 0.2. The 0.2.x releases call
`matplotlib.rcParams._get(...)`, which exists only in matplotlib 3.10 and
later. Pi OS bookworm and noble ship apt matplotlib 3.6.3, so on Python 3.12
the transitive 0.2.x release pulled in by ipykernel breaks `plt.subplots()`
inside Jupyter with `AttributeError: 'RcParams' object has no attribute
'_get'`. The pin holds until someone upgrades apt matplotlib or rebases on a
newer image.

`nptyping` is deliberately absent. Earlier v0.0.8 drafts pinned it below 2
because the v1 library used the deprecated
`NDArray[(480, 640, 3), np.uint8]` form. On Python 3.12 both major versions
are broken: 2.x raises `InvalidArgumentsError` at the class definition, and
1.x triggers a runaway `typing._type_repr` recursion that adds about 30
seconds to a cold import. MITUavNeo/uav-neo-library hit the same wall and
dropped the dependency, shipping a two-line inline `NDArray` stub in each
module that needs the syntax; racecar-neo-library v1.2.0 mirrors that.

## LSM9DS1 axis mapping

`config/pit.yaml`: `imu.accel_gyro_axis_order`, `imu.accel_gyro_axis_sign`,
`imu.mag_axis_order`, `imu.mag_axis_sign`; applied by
`racecar_neo_ros2_driver/pit_node.py` `remap_axes`.

Every published IMU topic uses REP-103 body axes (x forward, y left, z up).
`imu_fusion_node` averages the LSM9DS1 with the RealSense IMU, which it remaps
from the camera's optical axes (`config/imu_fusion.yaml`,
`realsense_axis_order` and `realsense_axis_sign`: X = +RS_Z, Y = -RS_X,
Z = -RS_Y). The LSM9DS1 accel and gyro mapping was derived on the car on
2026-07-07 against the camera (static gravity in about ten orientations,
camera gyro correlation +1.00 per axis) as RS_X = -LSM_Y, RS_Y = +LSM_Z,
RS_Z = +LSM_X. Composed with the camera-to-body remap:

```
X = +LSM_X    Y = +LSM_Y    Z = -LSM_Z          accel and gyro
X = -M_X      Y = +M_Y      Z = -M_Z            magnetometer
```

The accel and gyro mapping is a reflection (determinant -1): the frame the
firmware publishes for accel and gyro is left-handed, so a rotation-only
solver cannot express it. Accel and gyro share the convention, so one order
and sign serve both. The magnetometer has its own axes on the chip; its
mapping is a rotation, set on the reference car on 2026-10-08: at rest the
field points down (Z negative, as Earth's field does in the northern
hemisphere), a left turn rotates it clockwise in the car's frame, and with
the nose pointing north `/mag` x reads positive and y near zero (two
placements, north within 10 deg of the nose). Steel and phones nearby bend
the field: a reading whose dip is far from the local value (about 66 deg in
Massachusetts) is disturbed.

The calibration in `lsm9ds1_cal.local.yaml` and `lsm9ds1_mag_cal.local.yaml`
applies after the remap, so its values are in body axes. A car calibrated
before this mapping (values in the camera's optical axes for accel and gyro,
chip axes for the magnetometer) needs `racecar calibrate imu mag` again.
`realsense_cal.local.yaml` applies to `/imu/realsense` before the remap and
stays valid. Correlation also says nothing about scale: `imu.gyro_scale` and
`imu.mag_scale` are unverified, and a deg/s versus rad/s mismatch on the gyro
would be a factor of 57. Check both against the RealSense during a known
rotation before trusting `/imu/fused` angular velocity or `/mag` heading.

Magnetometer strength, measured 2026-10-04 with `racecar calibrate check mag`
on the reference car: 101.3 uT calibrated (150.1 uT raw), about twice Earth's
field (about 51 uT in Massachusetts). The firmware sets the LSM9DS1 to
+/-8 gauss (`cfg::LSM9DS1_IMU::mag_gain`), whose sensitivity is 0.29 mgauss per
count against 0.14 at the chip's default +/-4 gauss: a ratio of 2.07, and
101.3 / 2.07 = 49 uT. The likeliest cause is the chip staying at +/-4 gauss
while the conversion assumes +/-8, which would make the field read about
2.07x high. A uniform scale leaves the field's direction, and so a compass
heading, unchanged, which is why the check warns rather than fails on field
strength. To confirm: compare with a phone's magnetometer at the same spot,
away from steel, or read back the chip's CTRL_REG2_M in firmware. If
confirmed, the fix belongs in the firmware; until then `imu.mag_scale`
(1e-6) could be set to about 0.483e-6 per car.

## EdgeTPU inference rate and threshold

`config/edgetpu.yaml`: `inference_rate_hz`, `score_threshold`;
`racecar_neo_ros2_driver/edgetpu_node.py` `_rate_limited`.

The color stream runs at 60 fps, and nothing downstream reads detections faster
than a dashboard redraws them. Frames above `inference_rate_hz` (15) are
dropped before the image decode, where the cost is. `frames_dropped` in the
node's `/diagnostics` entry counts them. On 2026-09-22 the node held 14.7 Hz at
59 to 67 ms per inference with the Pi throttled for under-voltage, so there is
little headroom above the cap on this car. That throttling came from a bad
supply wire, since replaced; see
[Root cause and repair](#root-cause-and-repair).

`score_threshold` 0.4 is the Coral examples' default and the value the COCO
model was evaluated at. At 0.5 an ordinary indoor scene reported no detections.

## Coral M.2 interrupts

`scripts/setup_coral.sh`, with `scripts/coral-msi.dts` and
`scripts/gasket-msi-fallback.patch`.

The M.2 card needs two things the stock setup does not provide. The gasket
driver comes from the feranick fork built for kernel 6.8 and later, plus a
patch that falls back from MSI-X to MSI. The device-tree overlay routes the Pi
5 external PCIe MSIs to pcie1's own controller, which has enough vectors;
without it apex fails with `Couldn't initialize interrupts: -28`. Both take
effect at boot, so the M.2 path requires a reboot. The USB accelerator needs
neither and gets non-root access from the udev rules instead. Full procedure,
verification and removal: docs/specifics/coral-m2-migration.md.

## RealSense firmware flash privileges

`scripts/flash_realsense_offline.sh`: `flash_rs`.

Enumeration and the post-flash verify run as the invoking user. The normal
`0b3a` device is reachable through the `video` and `plugdev` groups, and
gating those behind sudo breaks in any no-TTY context: sudo blocks on the
password prompt, emits no device output, and looks like "no camera". The
script sources the ROS overlay, so `LD_LIBRARY_PATH` is already set for those
in-process calls.

Only the flash itself needs root, because it writes the device and DFU
re-enumerates as `8086:0adb`, which udev grants no user access. sudo strips
`LD_*`, so the library path is passed explicitly there.

## Depth view gaps

The depth previews (web dashboard Overview, web teleop) draw pixels with no
depth reading in flat gray. Two gaps are normal on the D435i:

- A strip down the left edge, wider toward the bottom. The camera measures
  depth by comparing its two infrared cameras, and the leftmost columns are
  seen by only one of them. The strip is about 386 x 50 / Z pixels wide at
  640 x 480, where Z is the distance in mm: 20 px at 1 m, 110 px at the
  floor just ahead of the car.
- The bottom rows, where the floor is closer than the camera's minimum range
  of about 0.17 m, and shiny or plain floors with too little texture.

Gray outlines on the left side of near objects are the same effect: the
right camera cannot see behind the object's edge.

The depth image is wider than the color image (about 79 against 55 degrees
across), so a gap at the edge of the depth view can lie outside the color
view. Neither gap is an object on the car.

## raspi-config on Ubuntu

`scripts/setup_raspi_config.sh`.

Ubuntu's raspi-config fork lacks the `do_serial_cons` and `do_serial_hw` split
that upstream Raspberry Pi OS ships; it carries only the older combined
`do_serial`. The script feature-detects and falls back.

The `DTOVERLAY[warn]: no matching platform found` that `do_i2c` and `do_spi`
emit on Ubuntu is benign. The dtparam edits still take effect; verify with
`ls /dev/i2c-1 /dev/spidev0.0` after a reboot.

## Gamepad hid_nintendo blacklist

`scripts/modprobe.d/blacklist-hid-nintendo.conf`, installed by
`scripts/setup_udev.sh`.

The EasySMX KC-8236 and similar controllers spoof the Switch Pro ID
`057e:2009`. hid_nintendo binds them with the wrong button map and no
`/dev/input/js0`. Pi 4 kernel images lacked the module; Pi 5 kernels (6.x)
load it on every boot. Per-device unbinding through udev does not work,
because hid_nintendo re-grabs the device immediately and the controller never
sees the rejection long enough to fall back to Xbox 360 mode (`2f24:016d`,
bound by `xpad`). A module-wide blacklist does, at the cost of genuine Switch
Pro controllers.

## Lidar /scan stalls

`scripts/udev/99-racecar.rules` (lidar rule), `scripts/watchdog.py` (`NODES`
lidar entry, `freshness_sec`).

During the 2026-05-12 endurance run, a ModemManager probe of the lidar's CP2102
tty desynced the sllidar SDK's binary frame reader mid-stream. `/scan` went
silent while the process kept running and the topic stayed advertised, because
the SDK swallows transient read errors. Two guards cover it. The udev rule sets
`ID_MM_DEVICE_IGNORE=1`, so ModemManager never opens the port. The watchdog
restarts the lidar when `/scan` is older than `freshness_sec` (5 s), which
catches a stall from any other cause.

## Map quality

Walls in a saved map are usually two cells (about 10 cm) thick and ragged
where furniture stands. The map is built by slam_toolbox during
`racecar map start`; AMCL only uses the finished map and does not change it.

Causes, largest first:

- Turning during a lidar sweep. One sweep takes 146 ms, and neither
  slam_toolbox nor AMCL corrects a scan for the car's motion within it. At
  70 deg/s the car turns 10 deg per sweep, which moves a wall 3 m away by up
  to 0.5 m between the start and end of one scan. On a room map, scans taken
  turning faster than 80 deg/s fit the finished map 85 to 89% within 0.1 m,
  against 96 to 97% driving straight. Forward motion adds about 7 cm per
  sweep at the 0.5 m/s mapping speed, which is why `throttle_node` holds the
  gamepad there while mapping (`mapping_speed_cap`).
- Repeated passes. Each pass adds the wall again from a slightly different
  pose, so a wall driven past many times becomes a band.
- The 5 cm grid and the lidar's range noise of a few centimeters.
- The room. The lidar sees a single plane about 20 cm above the floor: chair
  and table legs, cables, bags, and baseboards at that height show as
  speckle and ragged edges; anything lower is not seen. Glass, mirrors, and
  dark gloss return few points. Doorways let the lidar see past the walls.

What helps: wide, gentle turns; each area driven once or twice; a loop back
to the start at the end, which lets slam_toolbox close the loop; clutter at
lidar height cleared first. Limiting the steering while mapping
(`mapping_steering_cap` in `config/throttle.yaml`, off by default) lowers the
turn rate but needs many three-point turns in a room, and in a test the walls
came out no thinner; see docs/advanced-settings.md, "Mapping limits".

Thick walls barely affect localization: AMCL scores each scan point by its
distance to the nearest wall, and on these maps the median is 25 to 28 mm.

## Localization state

`autonomy_node` reports `localized` in `/autonomy/status`
(`racecar_neo_ros2_driver/autonomy.py`, `is_localized`). In localization it
needs all of:

- a scan within 0.5 s (`scan_timeout_sec`), so a stopped lidar clears it;
- the localizer's pose spread within 0.25 m and 20 deg, tighter (0.6 of
  each) to become localized again;
- the live scan to fit the map: twice a second, 180 lidar points placed at
  the localizer's pose are checked against `/map`, and the share within 0.1 m
  of a wall is averaged over 2 s (`scan_fit`). It drops under 0.5 and
  returns at 0.65 (`config/autonomy.yaml`).

The spread alone is not enough. A car carried to another spot keeps AMCL's
particle cloud tight at the old pose, because AMCL updates only when the
wheels move and its recovery settings are off; it reported `localized` at the
wrong place for 40 s in a test. The scan fit falls from about 80% to 25 to
30% as soon as the car is lifted, which clears `localized` within a few
seconds. AMCL's own recovery (`recovery_alpha_slow`, `recovery_alpha_fast`)
was tried and stays off: on normal drives it caused pose jumps of up to
5.7 m and 90 deg and never found the carried car.

Causes of `localized: false`, and what to do:

| Cause | Sign | Remedy |
|---|---|---|
| Car moved since the last pose, or carried | `scan_fit` low, spread small | Set pose in the map viewer, `racecar map load NAME --pose x,y,yaw`, or Find me / `--global` and drive slowly |
| Global search not converged | spread large | Drive slowly through a distinct part of the room; in identical corridors it can settle on the wrong one |
| Furniture moved since mapping | `scan_fit` between 0.5 and 0.65 | `racecar map start NAME --from NAME` over the changed area |
| Wheels spun or slipped hard | spread jumps | Stop, then Set pose or Find me and drive slowly |
| Lidar stopped | `scan_age_ms` large | `racecar status`, "Lidar /scan stalls" |

`racecar status` shows these fields in its autonomy section, and the map
viewer shows the fit and spread in its status bar.

## Localizer choice

AMCL is the default localizer (`racecar map load NAME`); the slam_toolbox
localizer is `--localizer slam_toolbox`. On the same 6 min drive both fit the
map equally well (80.1% and 79.4% of points within 0.1 m).

| | AMCL | slam_toolbox |
|---|---|---|
| Parked car after load | localizes | needs a short drive first |
| Whole-map search (Find me, `--global`) | yes | no; needs a start pose |
| Long drive (168 m) | held | lost the car, did not recover |
| Wheel spin or slip | `localized` drops; set the pose or search | corrects it by scan matching |
| Furniture moved within a session | degrades | tracks through it |
| CPU | about 5% of a core | not measured; scan matches every scan |

Use AMCL for rooms that stay as mapped. Use the slam_toolbox localizer for a
space that changes while the car drives (a classroom with people and bags
moving), and remap with `--from` when a space has changed for good.

## Lab dashboards

`scripts/setup_dashboards.sh`, `scripts/dashboards/`.

Each unit is rendered from the dashboard's `.service.in`, substituting its
directory and adding the discovery scope, rather than copied, so the unit
always points at the tree it was installed from.

The script also stops, disables and removes the units of the three retired
dashboards (`eps`, `pursuit`, `smartfollow`), so a car upgraded from v0.8.0
does not keep a second `/drive` publisher installed. Their old
checkouts under `scripts/dashboards/` are gitignored and left in place.

Saved tuning lives in `<name>.local.yaml`, never in the shipped
`<name>.yaml`, so updating the driver keeps a car's tuning and the tree stays
clean.

## WiFi persistence

`scripts/racecar-tool.sh`: the `wifi connect` and `wifi status` actions.

A car joined a network, rebooted, and came back with no wlan0 link, so the
connect had to be repeated every session.

Two separate flags decide whether NetworkManager rejoins at boot, and nothing
guaranteed either one:

- `connection.autoconnect` on the profile. This is what survives a reboot, and
  it was `no` on the affected profile. The tool never set it, relying on
  whatever default nmcli picks for the creation path; the GNOME network menu
  also writes `no` here when someone disconnects from the desktop, which a car
  booting to `graphical.target` makes easy to hit.
- The device's own autoconnect flag. `nmcli device disconnect`, which
  `racecar wifi disconnect` runs, sets it false so the car does not
  immediately rejoin. That is intended and is runtime-only, cleared at boot,
  but nothing cleared it on a later connect either.

`connect` now sets both on every path, and creates enterprise profiles with
`connection.autoconnect yes` rather than correcting them afterwards.
`racecar wifi status` reports the result as one line, because a car that reads
"connected" can still be one reboot from no link.

When testing this by hand: `nmcli device disconnect` leaves the device in
NetworkManager's user-requested disconnected state, and NM will not
auto-activate out of it however the flags are set, until something activates a
connection explicitly. That state does not survive a reboot, so a
disconnect-then-wait test says nothing about boot behavior. The profile flag
is the part that persists.

## Boot brownout with ethernet attached

Symptom, seen 2026-09-09: after the battery died, the car failed to boot three
times in a row with the ethernet cable attached, and booted normally once the
cable was pulled.

Known causes are an unseated BEC bulk capacitor and a bad contact in the
supply harness; see [Not a flat battery](#not-a-flat-battery). One car was repaired
on 2026-10-03 by replacing the buck to Pi 5 V wire; see
[Root cause and repair](#root-cause-and-repair) for that fix and the check
that confirms it.

Not a network fault. The supply was out of spec and the ethernet PHY was the
last straw.

Evidence:

- `EXT5V` measured 4.61 V. The Pi 5 wants 5 V and tolerates 4.75 V; below that
  the PMIC asserts its undervoltage alarm.
- Every boot, failed and successful, logged `hwmon hwmon4: Undervoltage
  detected!` about four seconds in.
- The three failed boots lasted 21, 21 and 28 seconds and their journals end
  mid-line with no shutdown sequence. That is a brownout reset, not a hang or a
  service timeout; a hang would leave a wait-online timeout and a reached-target
  line, and a clean reboot would leave a shutdown transaction.
- Boot `-1` ends on `device (wlan1): Activation: successful` at 09:17:25. The
  ALFA dongle starting to transmit is a current step, and the rail collapsed at
  that instant. The successful boot brought the same AP up at 19 seconds and
  held.
- The battery's final boot before the cut logged 15 undervoltage events against
  one for every boot since.

Read the ethernet cable as margin, not cause. The Pi 5 gigabit PHY draws a few
hundred milliwatts once linked; removing it left just enough headroom for the
AP to come up. A car whose supply is in spec carries both without trouble.

`PSU_MAX_CURRENT=5000` in the bootloader EEPROM, set by
`scripts/setup_raspi_config.sh`, is an aggravating factor here. It tells the
firmware the supply can deliver 5 A so that USB peripherals are not capped at
600 mA, which is correct for a healthy BEC feed that cannot negotiate USB-PD.
When the supply cannot hold 5 V it removes the last guard: the firmware permits
a draw the supply cannot sustain. Do not lower it to mask a brownout; fix the
supply.

### Not a flat battery

Measurement on 2026-09-10 rules out a depleted battery; do not start by
charging the battery.

- A fresh battery did not lift the rail. It read lower than the tired one had.
- The BEC output measures 4.94 V on a multimeter while the Pi reads 4.49 V,
  and after recovery the drop from the same BEC is 36 to 68 mV under a
  four-core load step. A large steady drop and a small one cannot both be the
  harness.
- Within a single boot, with no reboot and no battery change, `EXT5V` went 4.49 V
  faulted, then 4.87 to 4.90 V with the live throttle bits clear, then 4.64 V
  faulted again. A discharging battery does not recover and re-sag like that.
- The recovery coincided with the top half of the chassis being removed.

That profile is an intermittent connection rather than a depleted cell, and it
is mechanical: it changes when the car is handled. Two known causes produce it:

- A bulk capacitor on the BEC output unseated by the power wire resting
  against it. This hardware has failed this way before. A missing output bulk
  capacitor also explains the apparent contradiction above, since a multimeter
  averages while the Pi's undervoltage detector trips on excursions below
  roughly 4.63 V; both readings can be true at once with no drop across the
  harness at all. Confirming it needs a scope on the BEC output under load, or
  an inspection of whether that capacitor is seated and whether the power wire
  bears on it.
- A resistive or intermittent contact in the supply harness. This was the
  cause on the car repaired 2026-10-03; see
  [Root cause and repair](#root-cause-and-repair).

Check both on a car showing this profile. Neither is visible from software:
`EXT5V` is the furthest upstream the PMIC sees, and its ADC samples far too
slowly to distinguish ripple from a low average.

`racecar status` reports this condition already, under `throttling` and
`under-voltage` in the SYSTEM section. Treat any measurement taken while those
are set as suspect, since a throttled car also reads low on every topic rate.

### Root cause and repair

This section covers one car, repaired 2026-10-03. The fix there was
replacing the buck to Pi GPIO 5 V wire. It does not rule out the BEC bulk
capacitor in [Not a flat battery](#not-a-flat-battery) as the cause on another car.

Supply chain, battery to Pi:

```
7.4 V battery -> switch -> INA226 shunt -> boost (Pololu U3V70F9, 9 V)
           -> buck (Pololu D24V90F5, 5 V) -> Pi 5 GPIO header 5 V pins
```

The Pi is fed through the GPIO header, not USB-C. Two contacts in that chain
were intermittent:

- The switch to INA226 `I+` connection. Flexing that wire on 2026-09-27
  reproduced the fault: 26 dips under 4.75 V, 11 kernel undervoltage events,
  then a 4 s sag to 4.39 V and a brownout reset. With the top closed, the RC
  receiver dongle pressed on that wire. While loose, `/battery/current` read
  0.0 A.
- The buck to Pi GPIO 5 V wire. Probing along it produced a step recovery
  from about 4.65 V to 4.87 V, and touching the header end reset the Pi
  with no sag first. A step in both directions is a contact making and
  breaking, not heat or load.

Ruled out: the battery (sag persisted on a full 8.2 V battery), the buck (4.95 V
metered, within its 4.80 to 5.20 V tolerance), the boost (about 3 A load
against a 6 A input rating), the BEC bulk capacitor (inspected and seated on
this car), and the Pi and its peripherals (16 min on wall power at 5.01 to
5.09 V, `0x0`, RealSense attached). An earlier fix that only secured both
contacts held for about 1.5 h, then failed again under load the same day.

Repair: replacing the buck to GPIO 5 V wire fixed it. The INA226 connection
was repaired at the same time. Measured on 2026-10-03, teleop stack running:

| Measure | 2026-09-27 18:48, faulty | 2026-10-03, repaired |
| ------- | ------------------------ | -------------------- |
| `EXT5V` mean | 4.713 V | 4.914 V |
| `EXT5V` min | 4.544 V | 4.886 V |
| Samples under 4.75 V | 82% | 0 of 10112 |
| PMIC under-voltage alarm set | 48% of samples | 0 |
| `get_throttled` | 0x50005 | 0x0, sticky bits clear |
| Kernel undervoltage events | 5 to 9 per minute | 0 |
| INA226 | 0.0 A while loose | 7.93 V, 2.21 A at 147 Hz |

The rail held with no alarm while the harness was flexed and the car was shaken
hard. A multimeter at the rail read 4.916 V against the PMIC's 4.914 V. Before
the repair the PMIC read about 85 mV under the meter; that gap was drop across
the old wire, not ADC offset.

Margin: the buck sits near the low end of its tolerance, so the rail has about
165 mV above the 4.75 V floor. A new harness fault eats that quickly.

Check after any work on the power harness, and after a drive:

```bash
vcgencmd get_throttled                        # 0x0; bits 16 and 18 are sticky since boot
vcgencmd pmic_read_adc EXT5V_V                # above 4.80 V under teleop load
journalctl -b -k | grep -ci undervoltage      # 0
last -x -n 6                                  # no "crash" on the previous boot
```

A nonzero sticky bit means the rail dipped at some point since boot, even if it
reads clean now. A previous boot that ends in `crash` without a power cut is a
brownout reset.
