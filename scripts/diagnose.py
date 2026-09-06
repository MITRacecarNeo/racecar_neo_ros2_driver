#!/usr/bin/env python3
"""
Whole-car diagnostic pass behind `racecar status`.

Replaces `test_async_core_real.ipynb` for the "is this car healthy" question.
The notebook is the behavioural spec, but its structure is the slow part:
fifteen sections, each opening its own fixed 8 to 30 second window, run one
after another. Nothing about the checks requires that, so every subscription
is opened at once and shares a single sample window, and the host checks run
on a worker thread beside it.

Strict by design: the exit code is 0 only when every requested check passed.
A car with teleop stopped skips its sensor checks, and a skip is not a pass;
a readiness check that returned 0 for a car running no software would be
worse than no check at all. Deselecting a section with --quick or --section
is different from a check failing to run, and does not affect the exit code.

Read-only. Nothing here commands the hardware.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import glob
import grp
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import sysinfo  # noqa: E402

OK, WARN, FAIL, SKIP = 'OK', 'WARN', 'FAIL', 'SKIP'

SECTIONS = ('devices', 'sensors', 'actuators', 'system', 'services', 'network')

DEFAULT_WINDOW = 2.0
DISCOVERY_TIMEOUT = 6.0


@dataclass
class Check:
    """One diagnostic result."""

    group: str
    name: str
    status: str
    detail: str = ''


@dataclass
class TopicSpec:
    """Expected publication rate for one topic."""

    topic: str
    label: str
    nominal: float
    floor_frac: float = 0.8
    note: str = ''

    @property
    def floor(self) -> float:
        return self.nominal * self.floor_frac


# Nominal rates come from configuration where one is declared and from
# measurement on a running car where it is not. The Teensy telemetry frame
# carries six topics at one rate; mux.yaml sets 50 Hz for the drive chain and
# imu_fusion.yaml sets 100 Hz for the fused IMU.
#
# The two camera streams are the exception to the 80 percent floor. They are
# configured for 60 and 30 fps but deliver roughly 25 and 12; that shortfall
# is a known issue tracked separately. Keying their floor to the configured
# rate would fail every car on every run for a condition already accepted, so
# nominal here is the observed rate and the gap is reported as detail.
SENSOR_TOPICS = [
    TopicSpec('/camera/color', 'RealSense color', 25.0, 0.5, 'configured 60, gap tracked'),
    TopicSpec('/camera/depth', 'RealSense depth', 12.0, 0.5, 'configured 30, gap tracked'),
    TopicSpec('/imu/realsense', 'RealSense IMU', 200.0),
    TopicSpec('/scan', 'RPLIDAR', 8.0),
    TopicSpec('/imu/lsm9ds1', 'PIT IMU', 136.0),
    TopicSpec('/mag', 'PIT magnetometer', 136.0),
    TopicSpec('/imu/fused', 'Fused IMU', 100.0),
    TopicSpec('/encoder/speed', 'Encoder', 136.0),
    TopicSpec('/battery/voltage', 'Pack voltage', 136.0),
    TopicSpec('/battery/current', 'Pack current', 136.0),
    TopicSpec('/rc/channels', 'FlySky RC', 136.0),
    TopicSpec('/edgetpu/inference', 'Coral inference', 17.0),
    TopicSpec('/joy', 'Gamepad', 16.0),
]

ACTUATOR_TOPICS = [
    TopicSpec('/motor', 'Throttle output', 50.0),
    TopicSpec('/mux_out', 'Mux output', 50.0),
]

# Topics whose payload is asserted, not just its arrival rate. Everything else
# is subscribed raw so messages are counted without being deserialised.
VALUE_TOPICS = {
    '/imu/lsm9ds1',
    '/battery/voltage',
    '/battery/current',
    '/scan',
    '/rc/channels',
}

USB_DEVICES = [
    ('8086:0b3a', 'RealSense D435i'),
    ('0e8d:7612', 'ALFA AP dongle'),
    ('10c4:ea60', 'CP2102 (lidar)'),
    ('045e:028e', 'Gamepad'),
]

SERVICE_UNITS = ('racecar-teleop', 'racecar-watchdog',
                 'racecar-dashboard', 'racecar-jupyter')


# ---------------------------------------------------------------------------
# Host checks (no ROS graph needed)
# ---------------------------------------------------------------------------

def _run(cmd: list[str], timeout: float = 5.0) -> str:
    """Run a command and return stdout, or an empty string on any failure."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return ''
    return r.stdout if r.returncode == 0 else ''


def check_devices() -> list[Check]:
    """Device nodes, buses and group membership."""
    out: list[Check] = []
    g = 'devices'

    for link, hint in (('/dev/neo-pit-pcb', 'racecar udev'),
                       ('/dev/lidar', 'racecar udev')):
        p = Path(link)
        if p.exists():
            out.append(Check(g, Path(link).name, OK, str(p.resolve())))
        else:
            out.append(Check(g, Path(link).name, FAIL, f'missing (run: {hint})'))

    # The Coral moved to M.2 PCIe; it has no USB presence at all, so an lsusb
    # probe for it reports a missing accelerator on a working car.
    apex = sorted(glob.glob('/dev/apex_*'))
    if apex:
        slot = ''
        for line in _run(['lspci']).splitlines():
            if 'Coral' in line or 'Global Unichip' in line:
                slot = line.split()[0]
                break
        detail = f'{apex[0]}' + (f' (pci {slot})' if slot else '')
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

    for pattern, label in (('/dev/spidev*', 'spidev'),
                           ('/dev/gpiochip*', 'gpiochip'),
                           ('/dev/i2c-1', 'i2c-1')):
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
        out.append(Check(g, 'groups', FAIL,
                         f'not a member of {", ".join(missing)} (log out and back in?)'))
    else:
        out.append(Check(g, 'groups', OK, ' '.join(sorted(existing))))

    return out


def check_system() -> list[Check]:
    """CPU, thermals, memory, disk and the clock."""
    out: list[Check] = []
    g = 'system'

    load = sysinfo.read_loadavg()
    cpus = sysinfo.cpu_count()
    if load is None:
        out.append(Check(g, 'load', SKIP, 'unreadable'))
    else:
        one = load[0]
        ratio = one / cpus
        # A car running the full teleop stack sits near 1.0x, so the warning
        # line has to sit above that or it fires on every healthy car.
        if ratio <= 1.5:
            status = OK
        elif ratio <= 3.0:
            status = WARN
        else:
            status = FAIL
        out.append(Check(g, 'load', status, f'{one:.2f} on {cpus} cores ({ratio:.2f}x)'))

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
        out.append(Check(g, 'memory', status,
                         f'{mem["available"]} MiB available of {mem["total"]}'))

    disk = sysinfo.read_disk('/')
    if disk is None:
        out.append(Check(g, 'disk', SKIP, 'unreadable'))
    else:
        status = OK if disk['percent_used'] < 90 else (
            WARN if disk['percent_used'] < 95 else FAIL)
        out.append(Check(g, 'disk', status,
                         f'{disk["free"]}G free on / ({disk["percent_used"]}% used)'))

    volts = sysinfo.read_rtc_voltage()
    rtc_status, rtc_label = sysinfo.classify_rtc(volts)
    out.append(Check(g, 'rtc cell',
                     {'healthy': OK, 'stale': WARN, 'dead': FAIL}[rtc_status],
                     rtc_label))

    uv = sysinfo.read_under_voltage_alarm()
    if uv is None:
        out.append(Check(g, 'under-voltage', SKIP, 'no rpi_volt hwmon'))
    elif uv:
        out.append(Check(g, 'under-voltage', FAIL, 'alarm tripped this boot'))
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

    return out


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
            out.append(Check('services', unit, WARN,
                             f'active={active or "?"} enabled={enabled or "?"}'))
    return out


def _iface_v4(iface: str) -> list[str]:
    out = _run(['ip', '-4', '-o', 'addr', 'show', iface, 'scope', 'global'])
    return [ln.split()[3] for ln in out.splitlines() if len(ln.split()) > 3]


def check_network() -> list[Check]:
    """eth0 addressing, the client radio, the AP, and the desktop target."""
    out: list[Check] = []
    g = 'network'

    eth = _iface_v4('eth0')
    if not eth:
        out.append(Check(g, 'eth0 address', WARN, 'no global IPv4 (cable out?)'))
    elif len(eth) > 1:
        # The dual-address state is what makes the static drop; it is the
        # condition `racecar eth` exists to prevent.
        out.append(Check(g, 'eth0 address', FAIL,
                         f'{len(eth)} IPv4 addresses ({", ".join(eth)}); run: racecar eth static'))
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
            out.append(Check(g, 'eth0 v6 default', FAIL,
                             'present in static mode; run: racecar eth static'))
        else:
            out.append(Check(g, 'eth0 v6 default', OK, 'none, as expected in static mode'))
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
# ROS graph checks
# ---------------------------------------------------------------------------

@dataclass
class RosResult:
    """Outcome of the shared sampling window."""

    available: bool = False
    reason: str = ''
    counts: dict = field(default_factory=dict)
    elapsed: float = 0.0
    present: set = field(default_factory=set)
    values: dict = field(default_factory=dict)


def sample_ros(window: float, specs: list[TopicSpec]) -> RosResult:
    """Open every subscription at once and count arrivals over one window."""
    result = RosResult()
    try:
        import importlib

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                               QoSReliabilityPolicy)
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

        # Discovery is the floor on runtime, not the sample window: nothing
        # arrives until the graph is known. Poll until the expected topics
        # show up rather than sleeping a fixed amount.
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        names: dict = {}
        while time.monotonic() < deadline:
            names = dict(node.get_topic_names_and_types())
            if sum(1 for t in wanted if t in names) >= len(wanted):
                break
            time.sleep(0.2)
        if not names:
            result.reason = 'no ROS graph visible'
            return result

        result.present = {t for t in wanted if t in names}
        if not result.present:
            result.reason = 'no racecar topics on the graph'
            return result

        counts = {t: 0 for t in result.present}
        latest: dict = {}
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.VOLATILE

        def make_cb(topic: str, keep: bool):
            def cb(msg):
                counts[topic] += 1
                if keep:
                    latest[topic] = msg
            return cb

        for topic in sorted(result.present):
            type_str = names[topic][0]
            pkg, _, cls = type_str.split('/')
            msg_cls = getattr(importlib.import_module(f'{pkg}.msg'), cls)
            keep = topic in VALUE_TOPICS
            node.create_subscription(
                msg_cls, topic, make_cb(topic, keep), qos, raw=not keep)

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
        result.counts = dict(counts)
        result.values = latest
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


def rate_checks(group: str, specs: list[TopicSpec], ros: RosResult) -> list[Check]:
    """Turn the sampled counts into per-topic rate checks."""
    out: list[Check] = []
    for spec in specs:
        if not ros.available:
            out.append(Check(group, spec.label, SKIP, ros.reason or 'no ROS graph'))
            continue
        if spec.topic not in ros.present:
            out.append(Check(group, spec.label, FAIL, f'{spec.topic} not published'))
            continue
        hz = ros.counts.get(spec.topic, 0) / ros.elapsed if ros.elapsed else 0.0
        detail = f'{hz:.1f} Hz (floor {spec.floor:.1f})'
        if spec.note:
            detail += f'; {spec.note}'
        out.append(Check(group, spec.label, OK if hz >= spec.floor else FAIL, detail))
    return out


def value_checks(ros: RosResult) -> list[Check]:
    """Assert what the notebook asserted about payloads, not just arrival."""
    out: list[Check] = []
    g = 'sensors'
    if not ros.available:
        for name in ('IMU magnitude', 'Pack voltage range', 'LIDAR samples', 'RC channels'):
            out.append(Check(g, name, SKIP, ros.reason or 'no ROS graph'))
        return out

    imu = ros.values.get('/imu/lsm9ds1')
    if imu is None:
        out.append(Check(g, 'IMU magnitude', SKIP, 'no sample captured'))
    else:
        a = imu.linear_acceleration
        mag = (a.x ** 2 + a.y ** 2 + a.z ** 2) ** 0.5
        status = OK if 8.0 < mag < 12.0 else FAIL
        out.append(Check(g, 'IMU magnitude', status,
                         f'{mag:.2f} m/s^2 (expect 8 to 12 at rest)'))

    volt = ros.values.get('/battery/voltage')
    curr = ros.values.get('/battery/current')
    if volt is None:
        out.append(Check(g, 'Pack voltage range', SKIP, 'no sample captured'))
    else:
        v = volt.data
        c = curr.data if curr is not None else 0.0
        status = OK if 5.0 < v < 13.0 and c >= 0.0 else FAIL
        out.append(Check(g, 'Pack voltage range', status, f'{v:.2f} V, {c:.2f} A'))

    scan = ros.values.get('/scan')
    if scan is None:
        out.append(Check(g, 'LIDAR samples', SKIP, 'no sample captured'))
    else:
        n = len(scan.ranges)
        status = OK if n == 1080 else WARN
        out.append(Check(g, 'LIDAR samples', status, f'{n} (expect 1080 angle-compensated)'))

    rc = ros.values.get('/rc/channels')
    if rc is None:
        out.append(Check(g, 'RC channels', SKIP, 'no sample captured'))
    else:
        n = len(rc.data)
        out.append(Check(g, 'RC channels', OK if n == 8 else FAIL, f'{n} channels (expect 8)'))

    return out


def actuator_checks(ros: RosResult) -> list[Check]:
    """Drive chain plus the two display devices, observed rather than driven."""
    out = rate_checks('actuators', ACTUATOR_TOPICS, ros)

    dot_running = bool(_run(['pgrep', '-f', 'dotmatrix_node']).strip())
    spidev = bool(glob.glob('/dev/spidev*'))
    if dot_running and spidev:
        out.append(Check('actuators', 'Dot matrix', OK, 'node running, spidev present'))
    elif not spidev:
        out.append(Check('actuators', 'Dot matrix', FAIL, 'no /dev/spidev*'))
    else:
        out.append(Check('actuators', 'Dot matrix', WARN, 'dotmatrix_node not running'))

    pit_running = bool(_run(['pgrep', '-f', 'pit_node']).strip())
    if pit_running:
        out.append(Check('actuators', 'LED strip', OK, 'pit_node running (owns the strip)'))
    else:
        out.append(Check('actuators', 'LED strip', WARN, 'pit_node not running'))

    return out


# ---------------------------------------------------------------------------
# Presentation
# ---------------------------------------------------------------------------

MARK = {OK: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]', SKIP: '[SKIP]'}


def render(checks: list[Check], elapsed: float, exit_code: int) -> str:
    lines: list[str] = []
    width = max((len(c.name) for c in checks), default=10)
    for group in SECTIONS:
        rows = [c for c in checks if c.group == group]
        if not rows:
            continue
        lines.append('')
        lines.append(group.upper())
        for c in rows:
            lines.append(f'  {MARK[c.status]}  {c.name:<{width}}  {c.detail}'.rstrip())

    tally = {s: sum(1 for c in checks if c.status == s) for s in (OK, WARN, FAIL, SKIP)}
    lines.append('')
    lines.append(
        f'  {tally[OK]} ok   {tally[WARN]} warn   {tally[FAIL]} fail   '
        f'{tally[SKIP]} skipped        {elapsed:.1f}s   exit {exit_code}'
    )
    if exit_code != 0:
        lines.append('  Strict: anything other than OK is a failure.')
    return '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(
        prog='racecar status',
        description='Whole-car diagnostic pass. Exits 0 only when every requested check passes.',
    )
    ap.add_argument('--quick', action='store_true',
                    help='skip the ROS sampling phase (host checks only)')
    ap.add_argument('--json', action='store_true', help='machine-readable output')
    ap.add_argument('--section', default='',
                    help=f'comma-separated subset of: {", ".join(SECTIONS)}')
    ap.add_argument('--window', type=float, default=DEFAULT_WINDOW,
                    help=f'ROS sample window in seconds (default {DEFAULT_WINDOW})')
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
    needs_ros = any(s in requested for s in ('sensors', 'actuators'))
    if needs_ros:
        ros = sample_ros(args.window, SENSOR_TOPICS + ACTUATOR_TOPICS)

    worker.join(timeout=30)

    checks: list[Check] = []
    checks.extend(host.get('devices', []))
    if 'sensors' in requested:
        checks.extend(rate_checks('sensors', SENSOR_TOPICS, ros))
        checks.extend(value_checks(ros))
    if 'actuators' in requested:
        checks.extend(actuator_checks(ros))
    checks.extend(host.get('system', []))
    checks.extend(host.get('services', []))
    checks.extend(host.get('network', []))

    exit_code = 0 if all(c.status == OK for c in checks) else 1
    elapsed = time.monotonic() - started

    if args.json:
        print(json.dumps({
            'elapsed_sec': round(elapsed, 2),
            'exit_code': exit_code,
            'checks': [c.__dict__ for c in checks],
        }, indent=2))
    else:
        print(render(checks, elapsed, exit_code))
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
