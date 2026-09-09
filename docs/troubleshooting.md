# Troubleshooting

Failure modes that shaped the code, and the reasoning behind choices that look
arbitrary at the call site. Each section names the code it explains, so a
comment in the source can point here instead of carrying the full account.

## Contents

- [Diagnostic rate checks](#diagnostic-rate-checks)
- [eth0 addressing](#eth0-addressing)
- [AP isolation dispatcher](#ap-isolation-dispatcher)
- [NetworkManager authorization](#networkmanager-authorization)
- [Shell block rewriting](#shell-block-rewriting)
- [Desktop toggle scope](#desktop-toggle-scope)
- [Jupyter dependency pins](#jupyter-dependency-pins)
- [Coral M.2 interrupts](#coral-m2-interrupts)
- [RealSense firmware flash privileges](#realsense-firmware-flash-privileges)
- [raspi-config on Ubuntu](#raspi-config-on-ubuntu)
- [Lab dashboard checkouts](#lab-dashboard-checkouts)
- [Boot brownout with ethernet attached](#boot-brownout-with-ethernet-attached)

## Diagnostic rate checks

`scripts/diagnose.py`: `SENSOR_TOPICS`, `DEFAULT_WINDOW`, `sample_ros`.

Counting a topic means receiving it, so the diagnostic competes with the car
for the same CPU. `pit_node` reads the Teensy on a Python thread and loses
throughput under that competition, which through v0.8.0 made the sensor
section report the tool's own overhead as a hardware fault.

Two subscriptions dominated the cost:

- The RealSense colour and depth streams, worth 40 percent of the PIT
  telemetry rate. Measured A/B on hardware over interleaved trials: 133.6 Hz
  mean without them, 79.9 Hz with. They are now read from `/diagnostics`,
  where the camera node publishes its own per-stream frequency and where
  `scripts/dashboard.py` has read them since v0.7.3.
- `/scan`, deserialised on arrival for its value check at 1080 ranges plus
  1080 intensities a message, which put a 7.2 Hz lidar at 4.6. Every
  subscription is raw now, and the one buffer a value check needs is
  deserialised after the window closes.

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
anything in the 90 to 110 Hz band while a halved frame rate (68 Hz) and a
dead link (0 Hz) both still fail.

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

The dispatcher is socket-activated by `NetworkManager-dispatcher.service`,
shipped enabled on Ubuntu Server but often disabled on Desktop and Raspberry
Pi OS images. Without it the script never runs and the iptables isolation
rules silently never apply, which is the failure mode v0.0.6 hit on first
install.

The service is `Type=simple` with no `RemainAfterExit`, so it reads
"inactive" whenever no script is currently running. `is-active` is therefore
the wrong probe; `is-enabled` answers the question that matters, which is
whether systemd will start it on the next connection event.

## NetworkManager authorization

`scripts/polkit/49-racecar-network.rules`, installed by
`scripts/setup_user_env.sh`.

NetworkManager gates activation and profile edits behind polkit, and the stock
policy answers `auth`. polkit collects a password through an agent and an SSH
session has none, so `racecar wifi connect` failed at the activation call with
`Not authorized to control networking`, after the passphrase had been typed
and with nothing said about the remedy. A headless car driven from a terminal
is the normal case, not an edge case.

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

## Coral M.2 interrupts

`scripts/setup_coral.sh`, with `scripts/coral-msi.dts` and
`scripts/gasket-msi-fallback.patch`.

The M.2 card needs two things the stock setup does not provide. The gasket
driver comes from the feranick fork built for kernel 6.8 and later, plus a
patch that falls back from MSI-X to MSI. The device-tree overlay routes the Pi
5 external PCIe MSIs to pcie1's own controller, which has enough vectors;
without it apex fails with `Couldn't initialize interrupts: -28`. Both take
effect at boot, so the M.2 path requires a reboot. The USB accelerator needs
neither and gets non-root access from the udev rules instead.

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

## raspi-config on Ubuntu

`scripts/setup_raspi_config.sh`.

Ubuntu's raspi-config fork lacks the `do_serial_cons` and `do_serial_hw` split
that upstream Raspberry Pi OS ships; it carries only the older combined
`do_serial`. The script feature-detects and falls back.

The `DTOVERLAY[warn]: no matching platform found` that `do_i2c` and `do_spi`
emit on Ubuntu is benign. The dtparam edits still take effect; verify with
`ls /dev/i2c-1 /dev/spidev0.0` after a reboot.

## Lab dashboard checkouts

`scripts/setup_dashboards.sh`.

The three dashboards are forks under MITRacecarNeo of Neobotics Foundation
repositories built for the NeoRacer. The forks carry this platform's lidar
convention, ports and branding, so unlike the four unforked dashboards that
v0.8.0 shipped they can be corrected at the source.

The unit is still rendered from the checkout's `.service.in` rather than
copied. A fork synced from Neobotics upstream comes back carrying Humble and
the neoracer names, and rendering is what keeps that car working instead of
pointing a unit at a ROS that is not installed. `unit_name` strips either
project's prefix so both land on the same `racecar-<name>.service`.

Updates are `git pull --ff-only` and never `reset --hard`, so a car's tuned
YAML survives.

## Boot brownout with ethernet attached

Symptom, seen 2026-09-09: after the pack died, the car failed to boot three
times in a row with the ethernet cable attached, and booted normally once the
cable was pulled.

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
- The pack's final boot before the cut logged 15 undervoltage events against
  one for every boot since, which is the decline that preceded it.

Read the ethernet cable as margin, not cause. The Pi 5 gigabit PHY draws a few
hundred milliwatts once linked; removing it left just enough headroom for the
AP to come up. A car whose supply is in spec carries both without trouble.

`PSU_MAX_CURRENT=5000` in the bootloader EEPROM, set by
`scripts/setup_raspi_config.sh`, is an aggravating factor here. It tells the
firmware the supply can deliver 5 A so that USB peripherals are not capped at
600 mA, which is correct for a healthy BEC feed that cannot negotiate USB-PD.
On a sagging pack it removes the last guard: the firmware permits a draw the
supply cannot sustain. Do not lower it to work around a weak battery; fix the
supply.

`racecar status` reports this condition already, under `throttling` and
`under-voltage` in the SYSTEM section. Treat any measurement taken while those
are set as suspect, since a throttled car also reads low on every topic rate.
