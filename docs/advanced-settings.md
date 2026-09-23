# Advanced settings

Settings where the shipped default differs from what an advanced user may
want. Defaults are chosen for a classroom fleet: predictable, recoverable in
the field, and safe for someone who has not read this file. Everything here is
reversible with one command.

Each table's "Safe path" row names the access path to use when changing the
setting cuts your current connection.

## Contents

- [Ethernet addressing](#ethernet-addressing)
- [Desktop session](#desktop-session)
- [WiFi client](#wifi-client)
- [ROS discovery scope](#ros-discovery-scope)
- [Access point identity](#access-point-identity)
- [RTC trickle charging](#rtc-trickle-charging)
- [Bootloader EEPROM](#bootloader-eeprom)
- [Per-car parameters](#per-car-parameters)
- [Golden image preparation](#golden-image-preparation)

## Ethernet addressing

| | |
|---|---|
| Default | `static`, `192.168.52.200/24`, no gateway |
| Change | `racecar eth dynamic` |
| Revert | `racecar eth static` |
| Safe path | `wlan0`, the AP (`wlan1`), or an HDMI console |

eth0 holds one IPv4 addressing mode at a time; the two are mutually exclusive
because carrying both is what made the static address drop. Static is the
default so a car is reachable at a known IP on a bare switch with no DHCP
server, which is the situation field debugging usually happens in.

The cost of static is that eth0 carries no gateway and no IPv6 default route,
so there is no route to the internet over the wire. A static car reaches the
internet over `wlan0` or not at all. Switch to `dynamic` for `apt`, `git`, or
anything else that needs the network, then switch back.

Changing the mode drops an SSH session arriving over eth0. The command detects
that and asks before applying; `--force` skips the prompt for scripted use.

To check that the static address no longer drops, `racecar eth monitor` logs
the addresses and link state so a drop leaves a record. The
`racecar-eth-monitor.service` unit is the multi-day version; it is not
installed by default and should be disabled once the soak is done.

## Desktop session

| | |
|---|---|
| Default | enabled (`graphical.target`) |
| Change | `racecar desktop disable` |
| Revert | `racecar desktop enable` |
| Safe path | any; applies on the next boot |

Disabling changes the boot target only. Packages stay installed, so the toggle
works on a car with no network and costs nothing to undo. There is no
immediate variant, so this cannot end a desktop session someone is using.
Disabling frees the memory and CPU the GNOME session holds on a car that is
only reached over SSH.

## WiFi client

| | |
|---|---|
| Default | no saved client profile |
| Change | `racecar wifi connect <ssid>` |
| Revert | `nmcli connection delete <ssid>` |
| Safe path | any; `wlan0` is not the AP |

Only `wlan0` is touched. The AP on `wlan1` is a separate radio and is
unaffected, so joining a network cannot drop an operator connected over the
AP.

A joined network rejoins after every reboot. `racecar wifi disconnect` drops
the link only until the next `connect` or reboot; deleting the profile is what
stops the car rejoining.

Enterprise (802.1X) profiles created by this command always validate the
RADIUS server. If a network needs a certificate or server domain that cannot
be derived from the account's realm, pass `--ca-cert=` and
`--domain-suffix-match=` explicitly rather than disabling validation.

Watch for subnet collisions: a network handing out `10.42.0.0/24` or
`192.168.52.0/24` overlaps the AP subnet or the eth0 static, and
`racecar wifi status` reports the overlap.

## ROS discovery scope

| | |
|---|---|
| Default | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Change | `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` |
| Revert | unset it, or start a new shell |
| Safe path | any |

Localhost-only discovery keeps topic traffic off the ALFA dongle, which was
driving CPU spikes. Every node runs on the robot, so nothing on-board loses a
peer.

The cost is that off-robot ROS access does not work: `rviz`, `ros2 topic echo`
and remote nodes on a laptop will not see the robot's topics. Set the variable
to `SUBNET` in the shell for a session that needs it. Changing it in
`setup_user_env.sh` or the systemd units makes it permanent, and gives the CPU
cost back.

## Access point identity

| | |
|---|---|
| Default | SSID `racecar-neo-<id>`, PSK `racecar@mit`, channel 6 |
| Change | `racecar setup networking --ssid=NAME --psk=PASS --channel=N` |
| Revert | `racecar setup networking --reset` |
| Safe path | `wlan0` or an HDMI console |

Each car needs a distinct SSID, so the first run prompts for this car's ID.
Values persist to `~/.config/racecar/networking.env` and are replayed on every
later run.

Reconfiguring the AP drops SSH sessions arriving over it, and the eth0 step
can drop a session arriving over eth0 (it asks first).

## RTC trickle charging

| | |
|---|---|
| Default | enabled at 3.0 V (`dtparam=rtc_bbat_vchg=3000000`) |
| Change | `RTC_VCHG_UV=0 bash scripts/setup_raspi_config.sh` (removes the `dtparam=rtc_bbat_vchg=` line; charging off) |
| Revert | `bash scripts/setup_raspi_config.sh` (writes the line back at 3.0 V) |
| Safe path | any; applies on the next boot |

Only enable charging for a rechargeable cell such as the official Raspberry Pi
RTC battery (ML2032). Charging a primary CR2032 can make it vent or leak, so
set `RTC_VCHG_UV=0` if a non-rechargeable cell is fitted.

`racecar status` reports the cell voltage; below 2.7 V the clock resets on the
next power cut.

## Bootloader EEPROM

| | |
|---|---|
| Default | `PSU_MAX_CURRENT=5000`, `POWER_OFF_ON_HALT=1`, `BOOT_UART=1`, `BOOT_ORDER=0xf461` |
| Change | `sudo rpi-eeprom-config --edit`; `RACECAR_EEPROM=0` makes later setup runs leave the EEPROM alone |
| Revert | `bash scripts/setup_raspi_config.sh` (writes the four keys back) |
| Safe path | any; applies on the next boot |

`PSU_MAX_CURRENT` is the one that matters. A car fed from a BEC never
negotiates USB-PD, so the firmware assumes a 3 A supply and caps total USB
peripheral current at 600 mA, which is not enough for the RealSense, lidar and
ALFA dongle together.

## Per-car parameters

| | |
|---|---|
| Default | committed `config/*.yaml`; calibration files hold zeros |
| Change | write `config/<name>.local.yaml`, or run a calibration utility; then `racecar build` and `racecar service restart teleop` |
| Revert | delete the `.local.yaml`, `racecar build`, restart teleop |
| Safe path | any |

Each launch file loads `config/<name>.local.yaml` after the committed file of
the same name, so its keys win. The files are gitignored and never dirty the
tree. The calibration utilities (`calibrate_imu.py`, `calibrate_mag.py`,
`calibrate_realsense_imu.py`) write `lsm9ds1_cal.local.yaml`,
`lsm9ds1_mag_cal.local.yaml` and `realsense_cal.local.yaml`; see README
"Sensor calibration". A reimage or fresh clone loses them, so keep a copy in
`~/.config/racecar/calibration/`.

## Golden image preparation

Before capturing an image from a car, strip the per-unit identity:

```sh
racecar setup networking --reset     # AP connection and saved car ID
nmcli connection delete <ssid>       # any saved WiFi client profiles
```

Saved client profiles hold credentials. An enterprise profile keeps an account
name and its password in NetworkManager, and both clone onto every car built
from the image, so delete them before capture.

Per-car calibration lives in the gitignored `config/*.local.yaml` files (a
backup copy in `~/.config/racecar/calibration/`). An image carries one car's
values to every clone, so delete them before capture or recalibrate each car.
