#!/usr/bin/env python3
"""
Whole-car diagnostic pass behind `racecar status`.

All subscriptions share one sample window; host checks run on a worker
thread alongside it.

The exit code is 1 only when a check FAILs; WARN and SKIP leave the car
usable. --strict restores the stricter rule that anything other than OK fails.
Deselecting a section with --quick or --section is distinct from a check
failing to run and affects neither mode.

Read-only. Nothing here commands the hardware.

Rate-check tuning and the measurement costs behind it: docs/troubleshooting.md.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass, field
import glob
import grp
import importlib
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import features  # noqa: E402, I100  sibling script, after the path insert
import sysinfo  # noqa: E402

OK, WARN, FAIL, SKIP = 'OK', 'WARN', 'FAIL', 'SKIP'
SEVERITY = {OK: 0, SKIP: 1, WARN: 2, FAIL: 3}

SECTIONS = ('devices', 'sensors', 'actuators', 'autonomy', 'system', 'services', 'network')

# Shorter windows measure the PIT stream's clumping rather than its rate.
# See docs/troubleshooting.md, "Diagnostic rate checks".
DEFAULT_WINDOW = 5.0
DISCOVERY_TIMEOUT = 6.0

# Upper bound on reading one message from each /diagnostics-sourced stream
# after the window closes.
GRAB_TIMEOUT = 1.0

# RealSense reports its own per-stream rates here, so they need no
# subscription. See docs/troubleshooting.md, "Diagnostic rate checks".
DIAGNOSTICS_TOPIC = '/diagnostics'
REALSENSE_DIAGNOSTIC_NAMES = sysinfo.REALSENSE_DIAGNOSTIC_NAMES

# A stream under its floor is degraded but delivering and warns. Under
# STALL_HZ it has effectively stopped and fails, whatever its nominal rate.
STALL_HZ = 2.0

# Busy share of all cores, excluding this process, at which the cpu row warns.
CPU_SAMPLE_SEC = 1.0
CPU_WARN_PCT = 90.0


@dataclass
class Check:
    """One diagnostic result; `data` is a sample of what the source reported."""

    group: str
    name: str
    status: str
    detail: str = ''
    data: str = ''


@dataclass
class TopicSpec:
    """Expected publication rate for one topic."""

    topic: str
    label: str
    nominal: float
    floor_frac: float = 0.8

    @property
    def floor(self) -> float:
        return self.nominal * self.floor_frac


# Nominal rates come from configuration where one is declared (mux.yaml sets
# 50 Hz for the drive chain, imu_fusion.yaml 100 Hz for the fused IMU) and
# from measurement on a running car otherwise. The PIT floor is wider than the
# default because the Teensy frame rate moves with load.
# See docs/troubleshooting.md, "Diagnostic rate checks".
PIT_FLOOR_FRAC = 0.65

SENSOR_TOPICS = [
    TopicSpec('/camera/color', 'RealSense color', 60.0),
    TopicSpec('/camera/depth', 'RealSense depth', 30.0),
    TopicSpec('/imu/realsense', 'RealSense IMU', 200.0),
    TopicSpec('/scan', 'RPLIDAR', 7.2),
    TopicSpec('/imu/lsm9ds1', 'PIT IMU', 136.0, PIT_FLOOR_FRAC),
    TopicSpec('/mag', 'PIT magnetometer', 136.0, PIT_FLOOR_FRAC),
    TopicSpec('/imu/fused', 'Fused IMU', 100.0),
    TopicSpec('/encoder/speed', 'Encoder', 136.0, PIT_FLOOR_FRAC),
    TopicSpec('/battery/voltage', 'Battery voltage', 136.0, PIT_FLOOR_FRAC),
    TopicSpec('/battery/current', 'Battery current', 136.0, PIT_FLOOR_FRAC),
    TopicSpec('/rc/channels', 'FlySky RC', 136.0, PIT_FLOOR_FRAC),
    # Capped by edgetpu_node's inference_rate_hz, not by the camera.
    TopicSpec('/edgetpu/inference', 'Coral inference', 15.0),
    TopicSpec('/joy', 'Gamepad', 16.0),
]

# The six topics decoded from the shared Teensy telemetry frame.
PIT_TOPICS = {
    '/imu/lsm9ds1',
    '/mag',
    '/encoder/speed',
    '/battery/voltage',
    '/battery/current',
    '/rc/channels',
}

# Rates read off /diagnostics rather than counted here.
DIAGNOSTIC_SOURCED = set(REALSENSE_DIAGNOSTIC_NAMES.values())

ACTUATOR_TOPICS = [
    TopicSpec('/motor', 'Throttle output', 50.0),
    TopicSpec('/mux_out', 'Mux output', 50.0),
]

USB_DEVICES = [
    ('8086:0b3a', 'RealSense D435i'),
    ('0e8d:7612', 'ALFA AP dongle'),
    ('10c4:ea60', 'CP2102 (lidar)'),
    ('045e:028e', 'Gamepad'),
]

SERVICE_UNITS = (
    'racecar-teleop',
    'racecar-watchdog',
    'racecar-dashboard',
    'racecar-jupyter',
    'racecar-mapview',
    'racecar-battery-guard',
)
# Started from the 8080 Terminal tile, not at boot, so stopped is fine.
TERMINAL_UNIT = 'racecar-terminal'

# Started by `racecar map` or the map viewer, never at boot, so it is not a
# service row; the autonomy section samples these while it runs.
AUTONOMY_UNIT = 'racecar-autonomy'
AUTONOMY_TOPICS = [
    TopicSpec('/autonomy/pose', 'pose rate', 20.0),
    TopicSpec('/autonomy/status', 'status rate', 1.0, 0.5),
]
# localized drops under 0.5 and returns at 0.65 (config/autonomy.yaml).
SCAN_FIT_WARN = 0.65


def worst(*statuses: str) -> str:
    return max(statuses, key=SEVERITY.__getitem__)


# ---------------------------------------------------------------------------
# Host checks (no ROS graph needed)
# ---------------------------------------------------------------------------


_run = sysinfo.run_cmd


def check_devices() -> list[Check]:
    """Device nodes, buses and group membership."""
    out: list[Check] = []
    g = 'devices'

    for link, hint in (('/dev/neo-pit-pcb', 'racecar udev'), ('/dev/lidar', 'racecar udev')):
        p = Path(link)
        if p.exists():
            out.append(Check(g, p.name, OK, str(p.resolve())))
        else:
            out.append(Check(g, p.name, FAIL, f'missing (run: {hint})'))

    # Coral is M.2 PCIe: probe /dev/apex_* and lspci, not lsusb.
    apex = sorted(glob.glob('/dev/apex_*'))
    if apex:
        slot = ''
        for line in _run(['lspci']).splitlines():
            if 'Coral' in line or 'Global Unichip' in line:
                slot = line.split()[0]
                break
        detail = apex[0] + (f' (pci {slot})' if slot else '')
        out.append(Check(g, 'coral', OK, detail))
    else:
        out.append(Check(g, 'coral', FAIL, 'no /dev/apex_* node'))

    lsusb = _run(['lsusb'])
    if lsusb:
        for ident, label in USB_DEVICES:
            status = OK if ident in lsusb else FAIL
            detail = ident if status == OK else f'{ident} not on the bus'
            out.append(Check(g, label, status, detail))
    else:
        for _ident, label in USB_DEVICES:
            out.append(Check(g, label, SKIP, 'lsusb unavailable'))

    for pattern, label in (
        ('/dev/gpiochip*', 'gpiochip'),
        ('/dev/i2c-1', 'i2c-1'),
    ):
        found = sorted(glob.glob(pattern))
        if found:
            detail = f'{len(found)} node(s)' if len(found) > 1 else found[0]
            out.append(Check(g, label, OK, detail))
        else:
            out.append(Check(g, label, FAIL, f'no {pattern}'))

    try:
        mine = {grp.getgrgid(gid).gr_name for gid in os.getgroups()}
    except OSError:
        mine = set()
    wanted = {'dialout', 'spi', 'gpio', 'apex'}
    existing = set()
    for name in wanted:
        try:
            grp.getgrnam(name)
            existing.add(name)
        except KeyError:
            continue
    missing = sorted(existing - mine)
    if not existing:
        out.append(Check(g, 'groups', SKIP, 'none of the racecar groups exist'))
    elif missing:
        out.append(
            Check(
                g, 'groups', FAIL, f'not a member of {", ".join(missing)} (log out and back in?)'
            )
        )
    else:
        out.append(Check(g, 'groups', OK, ' '.join(sorted(existing))))

    return out


def _own_cpu_seconds() -> float:
    t = os.times()
    return t.user + t.system


def measure_cpu_busy(interval: float = CPU_SAMPLE_SEC) -> float | None:
    """
    Return the percent of all-core time spent busy over `interval`.

    This process's own time is subtracted: the ROS sampling window runs
    alongside and would otherwise be reported as car load.
    """
    before = sysinfo.read_cpu_times()
    own_before = _own_cpu_seconds()
    time.sleep(interval)
    after = sysinfo.read_cpu_times()
    own = _own_cpu_seconds() - own_before
    if before is None or after is None:
        return None
    total = after[1] - before[1]
    if total <= 0:
        return None
    busy = after[0] - before[0] - own * os.sysconf('SC_CLK_TCK')
    return max(0.0, min(100.0, 100.0 * busy / total))


def check_system(cpu_interval: float = CPU_SAMPLE_SEC) -> list[Check]:
    """CPU, thermals, memory, disk and the clock."""
    out: list[Check] = []
    g = 'system'

    # Busy share rather than load average: load counts queued threads, so it
    # passes 100 percent whenever the cores are oversubscribed and says
    # nothing about why. The ARM clock is the usual why on this car, since
    # under-voltage caps it well below its maximum.
    busy = measure_cpu_busy(cpu_interval)
    if busy is None:
        out.append(Check(g, 'cpu', SKIP, 'unreadable'))
    else:
        detail = f'{busy:.0f}% busy'
        current, maximum = sysinfo.read_arm_clock()
        if current and maximum:
            detail += f', arm {current} of {maximum} MHz'
        out.append(Check(g, 'cpu', OK if busy < CPU_WARN_PCT else WARN, detail))

    temp = sysinfo.read_soc_temp()
    if temp is None:
        out.append(Check(g, 'soc temp', SKIP, 'unreadable'))
    else:
        status = OK if temp < 75 else (WARN if temp < 82 else FAIL)
        out.append(Check(g, 'soc temp', status, f'{temp:.1f} C'))

    flags, active = sysinfo.read_throttled()
    if flags is None:
        out.append(Check(g, 'throttling', SKIP, 'vcgencmd unavailable'))
    elif flags == 0:
        out.append(Check(g, 'throttling', OK, 'none'))
    else:
        live = [n for n in active if 'has occurred' not in n]
        status = FAIL if live else WARN
        out.append(Check(g, 'throttling', status, f'0x{flags:x}: {", ".join(active)}'))

    mem = sysinfo.read_memory()
    if mem is None:
        out.append(Check(g, 'memory', SKIP, 'unreadable'))
    else:
        frac = mem['available'] / mem['total'] if mem['total'] else 0
        status = OK if frac > 0.15 else (WARN if frac > 0.07 else FAIL)
        out.append(
            Check(g, 'memory', status, f'{mem["available"]} MiB available of {mem["total"]}')
        )

    disk = sysinfo.read_disk('/')
    if disk is None:
        out.append(Check(g, 'disk', SKIP, 'unreadable'))
    else:
        status = OK if disk['percent_used'] < 90 else (WARN if disk['percent_used'] < 95 else FAIL)
        out.append(
            Check(g, 'disk', status, f'{disk["free"]}G free on / ({disk["percent_used"]}% used)')
        )

    volts = sysinfo.read_rtc_voltage()
    rtc_status, rtc_label = sysinfo.classify_rtc(volts)
    out.append(
        Check(g, 'rtc cell', {'healthy': OK, 'stale': WARN, 'dead': FAIL}[rtc_status], rtc_label)
    )

    uv = sysinfo.read_under_voltage_alarm()
    if uv is None:
        out.append(Check(g, 'under-voltage', SKIP, 'no rpi_volt hwmon'))
    elif uv:
        # Sticky until reboot. A live dip already fails the throttling row.
        out.append(Check(g, 'under-voltage', WARN, 'alarm tripped this boot'))
    else:
        out.append(Check(g, 'under-voltage', OK, 'clear'))

    ntp = sysinfo.ntp_synchronized()
    up = sysinfo.read_uptime()
    up_s = sysinfo.format_uptime(up) if up is not None else 'unknown'
    if ntp is None:
        out.append(Check(g, 'clock', SKIP, f'sync unknown, up {up_s}'))
    elif ntp:
        out.append(Check(g, 'clock', OK, f'NTP synced, up {up_s}'))
    else:
        out.append(Check(g, 'clock', WARN, f'not NTP synced, up {up_s}'))

    out.append(check_profile())
    out.append(check_data_root())
    return out


# The data root warns at this share used or below this much free.
DATA_WARN_PCT = 80
DATA_WARN_FREE_GB = 10


def check_data_root(nvme_sys: Path = Path('/sys/class/nvme/nvme0')) -> Check:
    """Report where the car's data goes, its free space, and a fallback to the SD card."""
    try:
        from racecar_neo_ros2_driver import data_root
    except ImportError:
        return Check('system', 'data root', SKIP, 'driver not built')
    root, warning = data_root.current()
    if not warning and not data_root.on_nvme(root):
        warning = data_root.resolve()[1]
    disk = sysinfo.read_disk(str(root)) if root.is_dir() else None
    if disk is None:
        return Check('system', 'data root', FAIL, f'{root} missing')
    where = 'NVMe' if data_root.on_nvme(root) else 'SD card'
    detail = f'{root} ({where}), {disk["free"]}G free ({disk["percent_used"]}% used)'
    low = disk['percent_used'] >= DATA_WARN_PCT or disk['free'] < DATA_WARN_FREE_GB
    if warning and nvme_sys.exists():
        return Check('system', 'data root', WARN, f'{detail}; {warning}')
    return Check('system', 'data root', WARN if low else OK, detail)


def check_profile() -> Check:
    """Report the car profile: where it is, the car name, and per-car files the driver ignores."""
    try:
        from racecar_neo_ros2_driver import car_profile
    except ImportError:
        return Check('system', 'car profile', SKIP, 'driver not built')
    here = Path(__file__).resolve().parent
    left = car_profile.unmigrated(here.parent / 'config', here / 'dashboards')
    detail = f'{car_profile.car_info()["name"]}, {car_profile.profile_dir()}'
    if left:
        return Check(
            'system',
            'car profile',
            WARN,
            f'{detail}; {len(left)} file(s) not used: racecar profile migrate',
        )
    return Check('system', 'car profile', OK, detail)


def check_services() -> list[Check]:
    """Report the systemd units that make up the running car."""
    out: list[Check] = []
    for unit in SERVICE_UNITS:
        active = _run(['systemctl', 'is-active', unit]).strip()
        enabled = _run(['systemctl', 'is-enabled', unit]).strip()
        if not active and not enabled:
            out.append(Check('services', unit, SKIP, 'unit not installed'))
            continue
        if active == 'active' and enabled == 'enabled':
            out.append(Check('services', unit, OK, 'active, enabled'))
        else:
            out.append(
                Check('services', unit, WARN, f'active={active or "?"} enabled={enabled or "?"}')
            )
    out.append(
        terminal_check(
            _run(['systemctl', 'is-active', TERMINAL_UNIT]).strip(),
            _run(['tmux', '-L', 'racecar-terminal', 'ls', '-F', '#S']),
        )
    )
    out.append(watchdog_check(sysinfo.read_watchdog_state()))
    return out


def terminal_check(active: str, sessions: str) -> Check:
    """Summarize the terminal app: running with its terminal count, stopped, or not installed."""
    if not active:
        return Check('services', TERMINAL_UNIT, SKIP, 'unit not installed')
    if active != 'active':
        return Check('services', TERMINAL_UNIT, OK, f'{active} (starts from the Terminal tile)')
    count = sum(1 for line in sessions.splitlines() if line.startswith('rcn-'))
    return Check('services', TERMINAL_UNIT, OK, f'active, {count} terminal(s)')


def watchdog_check(state: dict[str, Any] | None) -> Check:
    """Summarize watchdog.py's per-node state file as one services row."""
    name = 'watchdog nodes'
    if state is None:
        return Check('services', name, SKIP, 'no watchdog state file')
    if state['stale']:
        return Check('services', name, WARN, f'state not updated for {state["age_sec"]:.0f}s')
    nodes: dict[str, dict[str, Any]] = state.get('nodes', {})
    by_state: dict[str, list[str]] = {}
    for node, info in sorted(nodes.items()):
        by_state.setdefault(info.get('state', '?'), []).append(node)
    data = ', '.join(f'{node}={info.get("state")}' for node, info in sorted(nodes.items()))
    if 'gave_up' in by_state:
        given_up = ', '.join(
            f'{node} ({nodes[node].get("reason") or "down"})' for node in by_state['gave_up']
        )
        return Check('services', name, FAIL, f'gave up: {given_up}', data)
    waiting = [
        f'{label}: {", ".join(by_state[key])}'
        for key, label in (('no_device', 'no device'), ('restarting', 'restarting'))
        if key in by_state
    ]
    if waiting:
        return Check('services', name, WARN, '; '.join(waiting), data)
    parts = [f'{len(by_state.get("healthy", []))} healthy']
    parts += [
        f'{key}: {", ".join(by_state[key])}' for key in ('starting', 'disabled') if key in by_state
    ]
    return Check('services', name, OK, ', '.join(parts), data)


def _iface_v4(iface: str) -> list[str]:
    out = _run(['ip', '-4', '-o', 'addr', 'show', iface, 'scope', 'global'])
    return [ln.split()[3] for ln in out.splitlines() if len(ln.split()) > 3]


def check_network() -> list[Check]:
    """eth0 addressing, the client radio, the AP, and the desktop target."""
    out: list[Check] = []
    g = 'network'

    # Static plus DHCP on eth0 is what `racecar eth static` clears, but a
    # dual-mode eth0 can be deliberate, so it warns rather than fails.
    eth = _iface_v4('eth0')
    if not eth:
        out.append(Check(g, 'eth0 address', WARN, 'no global IPv4 (cable out?)'))
    elif len(eth) > 1:
        out.append(Check(g, 'eth0 address', WARN, f'{", ".join(eth)} (dual mode)'))
    else:
        out.append(Check(g, 'eth0 address', OK, eth[0]))

    # Only meaningful in static mode, where eth0 is supposed to carry no
    # default route at all.
    mode = os.environ.get('RACECAR_ETH_MODE', '')
    if not mode:
        cfg = Path.home() / '.config' / 'racecar' / 'networking.env'
        try:
            m = re.search(r'^RACECAR_ETH_MODE="?(\w+)"?', cfg.read_text(), re.MULTILINE)
            mode = m.group(1) if m else 'static'
        except OSError:
            mode = 'static'
    v6def = _run(['ip', '-6', 'route', 'show', 'default', 'dev', 'eth0']).strip()
    if mode == 'static':
        if v6def:
            out.append(Check(g, 'eth0 v6 default', WARN, 'present in static mode'))
        else:
            out.append(Check(g, 'eth0 v6 default', OK, 'none'))
    else:
        out.append(Check(g, 'eth0 v6 default', OK, f'{mode} mode, route allowed'))

    wlan0 = _iface_v4('wlan0')
    if wlan0:
        out.append(Check(g, 'wlan0 client', OK, wlan0[0]))
    else:
        out.append(Check(g, 'wlan0 client', OK, 'not connected'))

    ap = _run(['nmcli', '-t', '-f', 'NAME,DEVICE', 'con', 'show', '--active'])
    if not ap:
        out.append(Check(g, 'wlan1 AP', SKIP, 'nmcli unavailable'))
    elif any(ln.startswith('racecar-neo-ap:') for ln in ap.splitlines()):
        out.append(Check(g, 'wlan1 AP', OK, 'up'))
    else:
        out.append(Check(g, 'wlan1 AP', WARN, 'not active (racecar setup networking)'))

    default_target = _run(['systemctl', 'get-default']).strip()
    if not default_target:
        out.append(Check(g, 'desktop', SKIP, 'systemctl unavailable'))
    elif default_target == 'graphical.target':
        out.append(Check(g, 'desktop', OK, 'enabled'))
    else:
        out.append(Check(g, 'desktop', OK, f'headless ({default_target})'))

    return out


# ---------------------------------------------------------------------------
# Sample readers: one message in, (status, one-line summary) out
# ---------------------------------------------------------------------------


def _xyz(v: Any, scale: float = 1.0) -> str:
    return f'{v.x * scale:+.2f} {v.y * scale:+.2f} {v.z * scale:+.2f}'


def read_pit_imu(msg: Any) -> tuple[str, str]:
    a = msg.linear_acceleration
    mag = math.sqrt(a.x**2 + a.y**2 + a.z**2)
    data = f'accel {_xyz(a)} m/s^2, |a| {mag:.2f}'
    if 8.0 < mag < 12.0:
        return OK, data
    return FAIL, f'{data}, expect 8 to 12 at rest'


def read_accel(msg: Any) -> tuple[str, str]:
    return OK, f'accel {_xyz(msg.linear_acceleration)} m/s^2'


def read_mag(msg: Any) -> tuple[str, str]:
    return OK, f'field {_xyz(msg.magnetic_field, 1e6)} uT'


def read_fused(msg: Any) -> tuple[str, str]:
    q = msg.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))
    return OK, f'yaw {yaw:+.1f} deg, gyro z {msg.angular_velocity.z:+.2f} rad/s'


def read_speed(msg: Any) -> tuple[str, str]:
    return OK, f'{msg.data:+.2f} m/s'


def read_pack_voltage(msg: Any) -> tuple[str, str]:
    v = msg.data
    if 5.0 < v < 13.0:
        return OK, f'{v:.2f} V'
    return FAIL, f'{v:.2f} V, expect 5 to 13'


def read_pack_current(msg: Any) -> tuple[str, str]:
    c = msg.data
    return (OK if c >= 0.0 else FAIL), f'{c:.2f} A'


def read_rc(msg: Any) -> tuple[str, str]:
    n = len(msg.data)
    head = ' '.join(f'{x:+.2f}' for x in msg.data[:4])
    data = f'{n} channels: {head}' + (' ...' if n > 4 else '')
    if n == 8:
        return OK, data
    return FAIL, f'{data}, expect 8'


def read_scan(msg: Any) -> tuple[str, str]:
    # Median rather than nearest: a fixed near return on this car's mount
    # would pin the nearest reading regardless of the room.
    n = len(msg.ranges)
    hits = sorted(
        r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max
    )
    data = f'{n} ranges, {len(hits)} returns'
    if hits:
        data += f', median {hits[len(hits) // 2]:.2f} m'
    if n == 1080:
        return OK, data
    return WARN, f'{data}, expect 1080'


def read_detections(msg: Any) -> tuple[str, str]:
    n = len(msg.detections)
    data = f'{n} detection' + ('' if n == 1 else 's')
    scored = [
        (h.hypothesis.score, h.hypothesis.class_id) for d in msg.detections for h in d.results
    ]
    if scored:
        score, label = max(scored)
        data += f', top {label} {score:.2f}'
    return OK, data


def read_joy(msg: Any) -> tuple[str, str]:
    pressed = sum(1 for b in msg.buttons if b)
    return OK, f'{len(msg.axes)} axes, {len(msg.buttons)} buttons, {pressed} pressed'


def read_color(msg: Any) -> tuple[str, str]:
    return OK, f'{msg.width}x{msg.height} {msg.encoding}'


def read_depth(msg: Any) -> tuple[str, str]:
    data = f'{msg.width}x{msg.height} {msg.encoding}'
    if msg.encoding in ('16UC1', 'mono16') and msg.width and msg.height:
        lo = (msg.height // 2) * msg.step + (msg.width // 2) * 2
        hi = lo + 2
        raw = bytes(msg.data[lo:hi])
        if len(raw) == 2:
            mm = int.from_bytes(raw, 'big' if msg.is_bigendian else 'little')
            data += f', center {mm / 1000:.2f} m' if mm else ', center no return'
    return OK, data


def read_drive(msg: Any) -> tuple[str, str]:
    d = msg.drive
    return OK, f'speed {d.speed:+.2f} m/s, steer {d.steering_angle:+.2f} rad'


def read_autonomy_pose(msg: Any) -> tuple[str, str]:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return OK, f'x {p.x:+.2f} m, y {p.y:+.2f} m, heading {yaw:+.0f} deg'


def read_autonomy_status(msg: Any) -> tuple[str, str]:
    values = {kv.key: kv.value for kv in msg.values}
    return OK, f"{values.get('mode', '?')} {values.get('map', '')}".rstrip()


SAMPLE_READERS: dict[str, Callable[[Any], tuple[str, str]]] = {
    '/camera/color': read_color,
    '/camera/depth': read_depth,
    '/imu/realsense': read_accel,
    '/scan': read_scan,
    '/imu/lsm9ds1': read_pit_imu,
    '/mag': read_mag,
    '/imu/fused': read_fused,
    '/encoder/speed': read_speed,
    '/battery/voltage': read_pack_voltage,
    '/battery/current': read_pack_current,
    '/rc/channels': read_rc,
    '/edgetpu/inference': read_detections,
    '/joy': read_joy,
    '/motor': read_drive,
    '/mux_out': read_drive,
    '/autonomy/pose': read_autonomy_pose,
    '/autonomy/status': read_autonomy_status,
}


def read_sample(topic: str, msg: Any) -> tuple[str, str]:
    """Summarize one message; a payload of an unexpected shape warns."""
    reader = SAMPLE_READERS.get(topic)
    if reader is None:
        return OK, ''
    if msg is None:
        return OK, 'no sample captured'
    try:
        return reader(msg)
    except (AttributeError, TypeError, ValueError) as exc:
        return WARN, f'unreadable sample ({exc.__class__.__name__})'


# ---------------------------------------------------------------------------
# ROS graph checks
# ---------------------------------------------------------------------------


@dataclass
class RosResult:
    """Outcome of the shared sampling window."""

    available: bool = False
    reason: str = ''
    counts: dict[str, int] = field(default_factory=dict)
    elapsed: float = 0.0
    present: set[str] = field(default_factory=set)
    values: dict[str, Any] = field(default_factory=dict)
    reported: dict[str, float] = field(default_factory=dict)


_read_diagnostic_rates = sysinfo.read_diagnostic_rates


def _msg_class(type_str: str) -> Any:
    pkg, _, cls = type_str.split('/')
    return getattr(importlib.import_module(f'{pkg}.msg'), cls)


def _grab_once(node: Any, topics: list[str], types: dict[str, Any], qos: Any) -> dict[str, Any]:
    """Take the first message on each topic, waiting at most GRAB_TIMEOUT."""
    import rclpy

    got: dict[str, Any] = {}
    subs = [
        node.create_subscription(types[t], t, lambda m, t=t: got.setdefault(t, m), qos)
        for t in topics
    ]
    deadline = time.monotonic() + GRAB_TIMEOUT
    while len(got) < len(topics) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    for sub in subs:
        node.destroy_subscription(sub)
    return got


def sample_ros(window: float, specs: list[TopicSpec]) -> RosResult:
    """Open every subscription at once and count arrivals over one window."""
    result = RosResult()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    except Exception as exc:  # noqa: BLE001 - any import failure means no ROS
        result.reason = f'rclpy unavailable ({exc.__class__.__name__})'
        return result

    wanted = [s.topic for s in specs]
    try:
        rclpy.init()
    except Exception as exc:  # noqa: BLE001
        result.reason = f'rclpy init failed ({exc.__class__.__name__})'
        return result

    node = None
    try:
        node = Node('racecar_diagnose')

        # Poll discovery until every expected topic appears or DISCOVERY_TIMEOUT.
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        names: dict[str, list[str]] = {}
        while time.monotonic() < deadline:
            names = dict(node.get_topic_names_and_types())
            if all(t in names for t in wanted):
                break
            time.sleep(0.2)
        if not names:
            result.reason = 'no ROS graph visible'
            return result

        # A visible graph with none of the car's topics is a stopped stack,
        # which every rate row then reports as not published.
        result.present = {t for t in wanted if t in names}
        if not result.present:
            result.available = True
            return result

        # Subscribing to the RealSense streams would cost 40 percent of the
        # PIT rate this same window is measuring; one DiagnosticArray at 1 Hz
        # costs nothing. See docs/troubleshooting.md, "Diagnostic rate checks".
        counted = sorted(result.present - DIAGNOSTIC_SOURCED)
        counts = dict.fromkeys(counted, 0)
        latest: dict[str, Any] = {}
        reported: dict[str, float] = {}
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # Raw subscriptions throughout; each topic's last buffer is kept and
        # decoded after the window. See docs/troubleshooting.md,
        # "Diagnostic rate checks".
        raw_latest: dict[str, bytes] = {}
        msg_classes = {t: _msg_class(names[t][0]) for t in result.present}

        def make_cb(topic: str) -> Callable[[bytes], None]:
            def cb(msg: bytes) -> None:
                counts[topic] += 1
                raw_latest[topic] = msg

            return cb

        for topic in counted:
            node.create_subscription(msg_classes[topic], topic, make_cb(topic), qos, raw=True)

        if result.present & DIAGNOSTIC_SOURCED:
            try:
                from diagnostic_msgs.msg import DiagnosticArray

                node.create_subscription(
                    DiagnosticArray,
                    DIAGNOSTICS_TOPIC,
                    lambda msg: _read_diagnostic_rates(msg, reported),
                    qos,
                )
            except Exception:  # noqa: BLE001 - fall through to "no rate"
                pass

        # Let subscriptions match their publishers before the clock starts,
        # otherwise the first tenth of the window is counted as silence.
        settle = time.monotonic() + 0.4
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.02)
        for t in counts:
            counts[t] = 0

        start = time.monotonic()
        while time.monotonic() - start < window:
            rclpy.spin_once(node, timeout_sec=0.01)
        result.elapsed = time.monotonic() - start

        # Window closed, so decoding no longer lands on any rate. A payload
        # that will not decode is dropped and its row reports no sample.
        from rclpy.serialization import deserialize_message

        for topic, buf in raw_latest.items():
            try:
                latest[topic] = deserialize_message(buf, msg_classes[topic])
            except Exception:  # noqa: BLE001
                pass

        # The camera streams are too costly to count but cheap to read once
        # now that the window is closed.
        grab = sorted(result.present & DIAGNOSTIC_SOURCED)
        if grab:
            latest.update(_grab_once(node, grab, msg_classes, qos))

        result.counts = dict(counts)
        result.values = latest
        result.reported = dict(reported)
        result.available = True
    except Exception as exc:  # noqa: BLE001
        result.reason = f'{exc.__class__.__name__}: {exc}'
    finally:
        try:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return result


def rate_status(hz: float, spec: TopicSpec) -> str:
    """OK at or above the floor, WARN while still delivering, FAIL once stalled."""
    if hz >= spec.floor:
        return OK
    return WARN if hz >= STALL_HZ else FAIL


def rate_checks(group: str, specs: list[TopicSpec], ros: RosResult) -> list[Check]:
    """Turn the sampled counts and payloads into one row per topic."""
    out: list[Check] = []
    for spec in specs:
        if not ros.available:
            out.append(Check(group, spec.label, SKIP, ros.reason or 'no ROS graph'))
            continue
        if spec.topic not in ros.present:
            out.append(Check(group, spec.label, FAIL, 'not published'))
            continue
        value_status, data = read_sample(spec.topic, ros.values.get(spec.topic))
        if spec.topic in DIAGNOSTIC_SOURCED and spec.topic not in ros.reported:
            # The publisher reports its own rate. Its absence means the
            # DiagnosticArray never arrived, not that the stream is dead, so
            # say which one failed rather than reporting 0 Hz.
            status = worst(WARN, value_status)
            out.append(Check(group, spec.label, status, f'no rate on {DIAGNOSTICS_TOPIC}', data))
            continue
        if spec.topic in DIAGNOSTIC_SOURCED:
            hz = ros.reported[spec.topic]
        else:
            hz = ros.counts.get(spec.topic, 0) / ros.elapsed if ros.elapsed else 0.0
        status = worst(rate_status(hz, spec), value_status)
        out.append(Check(group, spec.label, status, f'{hz:.1f}/{spec.nominal:g} Hz', data))
    return out


def actuator_checks(ros: RosResult) -> list[Check]:
    """Drive chain plus the two display devices, observed rather than driven."""
    out = rate_checks('actuators', ACTUATOR_TOPICS, ros)

    # The Teensy drives the display; dotmatrix_node renders frames for pit_node.
    if _run(['pgrep', '-f', 'dotmatrix_node']).strip():
        out.append(Check('actuators', 'Dot matrix', OK, 'dotmatrix_node running'))
    else:
        out.append(Check('actuators', 'Dot matrix', WARN, 'dotmatrix_node not running'))

    pit_running = bool(_run(['pgrep', '-f', 'pit_node']).strip())
    if pit_running:
        out.append(Check('actuators', 'LED strip', OK, 'pit_node running (owns the strip)'))
    else:
        out.append(Check('actuators', 'LED strip', WARN, 'pit_node not running'))

    return out


# Rows of the teleop subsystems the Features tiles turn off (scripts/features.py).
FEATURE_ROWS = {
    'camera': ('RealSense color', 'RealSense depth', 'RealSense IMU'),
    'detections': ('Coral inference',),
}


def note_turned_off(checks: list[Check], running: dict[str, bool] | None = None) -> list[Check]:
    """Mark the rows of a subsystem teleop was started without as turned off, not failed."""
    running = features.running() if running is None else running
    off = {
        name
        for feature, names in FEATURE_ROWS.items()
        if not running.get(feature, True)
        for name in names
    }
    return [
        Check(c.group, c.name, SKIP, 'turned off (Features on 8080)') if c.name in off else c
        for c in checks
    ]


def note_mapping_pause(checks: list[Check], ros: RosResult) -> list[Check]:
    """While mapping, edgetpu_node drops its camera input (pause_while_mapping); say so."""
    msg = ros.values.get('/autonomy/status')
    mode = {kv.key: kv.value for kv in msg.values}.get('mode') if msg is not None else None
    if mode != 'mapping':
        return checks
    return [
        Check(c.group, c.name, OK, 'paused while mapping') if c.name == 'Coral inference' else c
        for c in checks
    ]


def autonomy_checks(unit_state: str, ros: RosResult, sampled: bool) -> list[Check]:
    """
    Mode, localized, scan fit, spread, and saves from /autonomy/status, plus the pose rate.

    `unit_state` is `systemctl is-active` for the autonomy unit; with `sampled`
    false (--quick, or the unit stopped) only the unit row is reported.
    """
    g = 'autonomy'
    if unit_state == 'failed':
        return [Check(g, 'unit', WARN, f'{AUTONOMY_UNIT} failed; journalctl -u {AUTONOMY_UNIT}')]
    if unit_state != 'active':
        return [Check(g, 'unit', OK, 'stopped')]
    out = [Check(g, 'unit', OK, 'active')]
    if not sampled:
        return out
    out += [c for c in rate_checks(g, AUTONOMY_TOPICS, ros) if c.name == 'pose rate']
    msg = ros.values.get('/autonomy/status')
    if not ros.available:
        return out
    if msg is None:
        out.append(Check(g, 'status', FAIL, 'no /autonomy/status; see racecar map status'))
        return out
    v = {kv.key: kv.value for kv in msg.values}
    mode = v.get('mode', '')
    where = f"{mode} on {v.get('map') or '(no map)'}"
    if mode == 'localization' and v.get('localizer'):
        where += f" with {v['localizer']}"
    out.append(Check(g, 'mode', OK, where, f"session {v.get('session', '')}".rstrip()))
    localized = v.get('localized') == 'true'
    if localized:
        out.append(Check(g, 'localized', OK, 'yes'))
    elif mode == 'localization':
        hint = 'drive slowly, or Set pose / Find me in the map viewer (port 8084)'
        out.append(Check(g, 'localized', WARN, f'no; {hint}'))
    else:
        out.append(Check(g, 'localized', WARN, 'no; is the lidar running?'))
    if mode == 'localization':
        if v.get('scan_fit'):
            fit = float(v['scan_fit'])
            status = OK if fit >= SCAN_FIT_WARN else WARN
            out.append(Check(g, 'scan fit', status, f'{fit:.0%} of points within 0.1 m of a wall'))
        if v.get('pose_sd_m'):
            spread = f"{float(v['pose_sd_m']):.2f} m, {float(v.get('pose_sd_deg') or 0):.1f} deg"
            out.append(Check(g, 'pose spread', OK, spread))
    if mode == 'mapping':
        saved = v.get('last_save') or 'not saved yet'
        autosave = v.get('last_autosave') or '-'
        out.append(Check(g, 'saves', OK, f'saved {saved}; autosave {autosave}'))
    return out


# ---------------------------------------------------------------------------
# Presentation
# ---------------------------------------------------------------------------

MARK = {OK: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]', SKIP: '[SKIP]'}
ANSI = {OK: '32', WARN: '33', FAIL: '31'}


def overall(checks: list[Check]) -> str:
    """Summary verdict: FAIL on any failure, WARN on any warning or skip, else OK."""
    if any(c.status == FAIL for c in checks):
        return FAIL
    if any(c.status in (WARN, SKIP) for c in checks):
        return WARN
    return OK


def exit_code_for(checks: list[Check], strict: bool = False) -> int:
    """
    Return 1 when any check failed, else 0.

    Strict mode also fails on WARN and SKIP, so 0 means every check was OK.
    """
    if strict:
        return 0 if all(c.status == OK for c in checks) else 1
    return 1 if any(c.status == FAIL for c in checks) else 0


def use_color() -> bool:
    """Color only an interactive terminal; NO_COLOR (no-color.org) opts out."""
    return (
        sys.stdout.isatty()
        and 'NO_COLOR' not in os.environ
        and os.environ.get('TERM', '') != 'dumb'
    )


def paint(text: str, status: str, color: bool, bold: bool = False) -> str:
    code = ANSI.get(status)
    if not color or code is None:
        return text
    return f'\033[{"1;" if bold else ""}{code}m{text}\033[0m'


def render(
    checks: list[Check], elapsed: float, exit_code: int, strict: bool = False, color: bool = False
) -> str:
    """
    Lay the checks out by section.

    Rows that carry a sample pad everything before it to one width and then
    tab-separate it, so the sample column lines up and `cut -f2` extracts it.
    """
    lines: list[str] = []
    name_w = max((len(c.name) for c in checks), default=10)
    detail_w = max((len(c.detail) for c in checks if c.data), default=0)
    for group in SECTIONS:
        rows = [c for c in checks if c.group == group]
        if not rows:
            continue
        lines.append('')
        lines.append(group.upper())
        for c in rows:
            mark = paint(MARK[c.status], c.status, color)
            if c.data:
                lines.append(f'  {mark}  {c.name:<{name_w}}  {c.detail:<{detail_w + 1}}\t{c.data}')
            else:
                lines.append(f'  {mark}  {c.name:<{name_w}}  {c.detail}'.rstrip())

    tally = {s: sum(1 for c in checks if c.status == s) for s in (OK, WARN, FAIL, SKIP)}
    counts = '   '.join(
        paint(f'{tally[s]} {label}', s, color and tally[s] > 0)
        for s, label in ((OK, 'ok'), (WARN, 'warn'), (FAIL, 'fail'), (SKIP, 'skipped'))
    )
    result = overall(checks)
    lines.append('')
    lines.append(
        f'  RESULT {paint(result, result, color, bold=True)}   {counts}   '
        f'{elapsed:.1f}s   exit {exit_code}'
    )
    if strict and exit_code != 0:
        lines.append('  Strict: anything other than OK is a failure.')
    return '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(
        prog='racecar status',
        description=(
            'Whole-car diagnostic pass. Exits 1 when a check fails; '
            'WARN and SKIP exit 0 unless --strict.'
        ),
    )
    ap.add_argument(
        '--quick', action='store_true', help='skip the ROS sampling phase (host checks only)'
    )
    ap.add_argument('--json', action='store_true', help='machine-readable output')
    ap.add_argument(
        '--strict', action='store_true', help='exit 1 unless every requested check is OK'
    )
    ap.add_argument(
        '--section', default='', help=f'comma-separated subset of: {", ".join(SECTIONS)}'
    )
    ap.add_argument(
        '--window',
        type=float,
        default=DEFAULT_WINDOW,
        help=f'ROS sample window in seconds (default {DEFAULT_WINDOW})',
    )
    args = ap.parse_args()

    if args.section:
        requested = [s.strip() for s in args.section.split(',') if s.strip()]
        unknown = [s for s in requested if s not in SECTIONS]
        if unknown:
            print(f'racecar status: unknown section(s): {", ".join(unknown)}', file=sys.stderr)
            print(f'sections: {", ".join(SECTIONS)}', file=sys.stderr)
            return 2
    else:
        requested = list(SECTIONS)

    if args.quick:
        requested = [s for s in requested if s not in ('sensors', 'actuators')]

    autonomy_state = ''
    if 'autonomy' in requested:
        autonomy_state = _run(['systemctl', 'is-active', AUTONOMY_UNIT]).strip()
    sample_autonomy = autonomy_state == 'active' and not args.quick

    started = time.monotonic()

    # Host checks do not need the graph, so they run while it is being
    # discovered rather than before or after it.
    host: dict[str, list[Check]] = {}

    def run_host() -> None:
        if 'devices' in requested:
            host['devices'] = check_devices()
        if 'system' in requested:
            host['system'] = check_system()
        if 'services' in requested:
            host['services'] = check_services()
        if 'network' in requested:
            host['network'] = check_network()

    worker = threading.Thread(target=run_host, daemon=True)
    worker.start()

    ros = RosResult()
    needs_ros = any(s in requested for s in ('sensors', 'actuators')) or sample_autonomy
    if needs_ros:
        specs = SENSOR_TOPICS + ACTUATOR_TOPICS + (AUTONOMY_TOPICS if sample_autonomy else [])
        ros = sample_ros(args.window, specs)

    worker.join(timeout=30)

    checks: list[Check] = []
    checks.extend(host.get('devices', []))
    if 'sensors' in requested:
        checks.extend(
            note_turned_off(note_mapping_pause(rate_checks('sensors', SENSOR_TOPICS, ros), ros))
        )
    if 'actuators' in requested:
        checks.extend(actuator_checks(ros))
    if 'autonomy' in requested:
        checks.extend(autonomy_checks(autonomy_state, ros, sample_autonomy))
    checks.extend(host.get('system', []))
    checks.extend(host.get('services', []))
    checks.extend(host.get('network', []))

    exit_code = exit_code_for(checks, args.strict)
    elapsed = time.monotonic() - started

    if args.json:
        print(
            json.dumps(
                {
                    'elapsed_sec': round(elapsed, 2),
                    'result': overall(checks),
                    'strict': args.strict,
                    'exit_code': exit_code,
                    'checks': [c.__dict__ for c in checks],
                },
                indent=2,
            )
        )
    else:
        print(render(checks, elapsed, exit_code, args.strict, use_color()))
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
