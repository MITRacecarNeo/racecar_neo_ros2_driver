#!/usr/bin/env python3
"""RACECAR Neo web dashboard: live node/topic monitor on port 8080 (stdlib HTTP + rclpy)."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
from types import FrameType
from typing import Any
from urllib.parse import parse_qs, urlparse

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, UInt32

# Shared with scripts/diagnose.py. Both are run by path rather than imported
# as a package, so the sibling has to be put on the path explicitly.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import features  # noqa: E402, I100  sibling script, after the path insert
from sysinfo import (  # noqa: E402
    classify_rtc as _classify_rtc,
    read_diagnostic_rates,
    read_memory as _read_memory,
    read_rtc_voltage as _read_battery_voltage,
    read_soc_temp as _read_soc_temp,
    read_under_voltage_alarm as _read_under_voltage_alarm,
    read_watchdog_state,
    REALSENSE_DIAGNOSTIC_NAMES,
)
import webui  # noqa: E402

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PORT = 8080

# Home page apps. Core apps start at boot. Utilities start when opened and
# stop with Clear apps, which needs scripts/polkit/49-racecar-dashboards.rules.
# `exclusive` utilities publish /drive, so one runs at a time. An app with no
# unit is shown but opens nothing.
APPS: list[dict[str, Any]] = [
    {'id': 'terminator', 'name': 'Terminator', 'icon': 'terminal', 'port': None,
     'group': 'core', 'unit': None, 'exclusive': False, 'startable': False,
     'desc': 'Open a terminal'},
    {'id': 'jupyter', 'name': 'JupyterLab', 'icon': 'notebook', 'port': 8888,
     'group': 'core', 'unit': 'racecar-jupyter', 'exclusive': False, 'startable': False,
     'desc': 'Notebooks and code'},
    {'id': 'mapview', 'name': 'Map Viewer', 'icon': 'map', 'port': 8084,
     'group': 'core', 'unit': 'racecar-mapview', 'exclusive': False, 'startable': False,
     'desc': 'Build and load maps'},
    {'id': 'webteleop', 'name': 'Web Teleop', 'icon': 'gamepad', 'port': 8081,
     'group': 'utilities', 'unit': 'racecar-webteleop', 'exclusive': True, 'startable': True,
     'desc': 'Drive from the browser with camera and lidar.'},
    {'id': 'linefollow', 'name': 'Line Follow', 'icon': 'line', 'port': 8082,
     'group': 'utilities', 'unit': 'racecar-linefollow', 'exclusive': True, 'startable': True,
     'desc': 'Follow a colored line with the camera.', 'needs': 'camera'},
    {'id': 'wallfollow', 'name': 'Wall Follow', 'icon': 'wall', 'port': 8083,
     'group': 'utilities', 'unit': 'racecar-wallfollow', 'exclusive': True, 'startable': True,
     'desc': 'Follow walls with the lidar.'},
    {'id': 'camlabel', 'name': 'Camera Label', 'icon': 'tag', 'port': 8085,
     'group': 'utilities', 'unit': 'racecar-camlabel', 'exclusive': False, 'startable': True,
     'desc': 'Capture camera frames and label them for training.', 'needs': 'camera'},
]

# Features: parts of the car that cost CPU and can be turned off. The camera
# and detections are teleop subsystems (scripts/features.py); changing one
# restarts teleop. Autonomy is its own unit, started from the map viewer.
FEATURES: list[dict[str, Any]] = [
    {'id': 'camera', 'name': 'Camera', 'icon': 'camera',
     'desc': 'Turn on/off the color and depth camera'},
    {'id': 'detections', 'name': 'Detections', 'icon': 'target',
     'desc': 'Turn on/off the detection node'},
    {'id': 'autonomy', 'name': 'Autonomy', 'icon': 'radar',
     'desc': 'Mapping or localization.'},
]
TELEOP_UNIT = 'racecar-teleop.service'
AUTONOMY_UNIT = 'racecar-autonomy.service'
AUTONOMY_MODE_FILE = Path.home() / '.config' / 'racecar' / 'autonomy.conf'
MAP_TOOL = [sys.executable, str(Path(__file__).resolve().parent / 'racecar_map.py')]
MAP_TOOL_TIMEOUT_SEC = 180.0
# Time for the sensors to come up after a teleop restart before autonomy resumes.
TELEOP_SETTLE_SEC = 10.0
READY_TIMEOUT_SEC = 1.0
SYSTEMCTL_TIMEOUT_SEC = 30.0
RATE_WINDOW_SEC = 3.0
MAPVIEW_PORT = 8084
# /autonomy/status arrives at 1 Hz; older than this, the node has stopped.
AUTONOMY_STALE_SEC = 5.0
SYSTEM_HEALTH_REFRESH_SEC = 60.0
# Rates, nodes, and app states are measured only while a page polls: the rate
# subscriptions wake the process for every message, a large share of a core.
# They stop VIEW_IDLE_SEC after the last request.
VIEW_IDLE_SEC = 15.0
MONITOR_PERIOD_SEC = 3.0
# Overview charts: pit_node's 1 Hz /battery/state and the monitor loop's CPU
# temperature and memory, kept for one battery's run time.
BATTERY_WINDOW_SEC = 2 * 3600
BATTERY_BUCKET_SEC = 10
BATTERY_STALE_SEC = 5.0

# supervised: the watchdog restarts this node. An unsupervised card that loses
# its topic shows gray instead of red.
MONITORED = {
    'pit': {'topic': '/imu/lsm9ds1', 'label': 'PIT board (drive + IMU)', 'supervised': True},
    'throttle': {'topic': '/motor', 'label': 'Throttle (clamping)', 'supervised': True},
    'mux': {'topic': '/mux_out', 'label': 'Mux (arbitrator)', 'supervised': True},
    'gamepad': {'topic': '/gamepad_drive', 'label': 'Gamepad', 'supervised': True},
    'imu_fusion': {'topic': '/imu/fused', 'label': 'IMU fusion', 'supervised': True},
    'lidar': {'topic': '/scan', 'label': 'RPLIDAR', 'supervised': True},
    'realsense': {'topic': '/camera/color', 'label': 'RealSense D435i', 'supervised': True},
    'edgetpu': {'topic': '/edgetpu/inference', 'label': 'Coral EdgeTPU', 'supervised': False},
    'dotmatrix': {'topic': '/dotmatrix/frame', 'label': 'Dot matrix', 'supervised': False},
    'led': {'topic': '/led/pixels', 'label': 'LED strip', 'supervised': False,
            'publisher': True},
}

# Subscribed to and counted by the dashboard.
RATE_TOPICS = [
    '/motor',
    '/mux_out',
    '/imu/fused',
    '/imu/lsm9ds1',
    '/scan',
    '/edgetpu/inference',
]
# Every topic in the rates table; the RealSense ones come from /diagnostics.
DISPLAY_RATE_TOPICS = [
    *RATE_TOPICS,
    '/imu/realsense',
    '/camera/color',
    '/camera/depth',
]

log = logging.getLogger('dashboard')

# ---------------------------------------------------------------------------
# Status collection (cached, background-refreshed)
# ---------------------------------------------------------------------------

_status_lock = threading.Lock()
_latest_status: dict[str, Any] = {
    'timestamp': '',
    'nodes': {},
    'node_list': [],
    'topic_list': [],
    'rates': {},
    'system_health': {},
    'watchdog_log': [],
    'log_dir': str(Path.home() / 'logs' / 'latest'),
    'autonomy': {},
    'apps': [],
}
_monitor_running = True


# ---------------------------------------------------------------------------
# Rate measurement via rclpy subscriptions
# ---------------------------------------------------------------------------


class History:
    """Timed samples of a few values over a sliding window (wall-clock seconds)."""

    def __init__(self, window_sec: float = BATTERY_WINDOW_SEC) -> None:
        self.window = window_sec
        self._samples: deque[tuple[float, ...]] = deque()
        self._lock = threading.Lock()

    def add(self, t: float, *values: float) -> None:
        with self._lock:
            self._samples.append((t, *values))
            while self._samples and self._samples[0][0] < t - self.window:
                self._samples.popleft()

    def latest(self, now: float) -> tuple[tuple[float, ...], float] | None:
        """Return the last values and their age, or None before the first sample."""
        with self._lock:
            if not self._samples:
                return None
            last = self._samples[-1]
        return last[1:], now - last[0]

    def buckets(self, now: float, bucket_sec: float = BATTERY_BUCKET_SEC) -> list[list[float]]:
        """Return [time, mean of each value] per bucket within the window, oldest first."""
        with self._lock:
            samples = [s for s in self._samples if s[0] >= now - self.window]
        acc: dict[int, list[float]] = {}
        for t, *values in samples:
            a = acc.setdefault(int(t // bucket_sec), [0.0] * (len(values) + 1))
            for i, v in enumerate(values):
                a[i] += v
            a[-1] += 1
        return [
            [(k + 0.5) * bucket_sec, *[round(s / acc[k][-1], 3) for s in acc[k][:-1]]]
            for k in sorted(acc)
        ]


_battery = History()
_system = History()

# The safety-controller override lets /drive through mux_node with no
# gamepad. The page keeps it alive; it ends when the page stops asking.
OVERRIDE_KEEPALIVE_SEC = 2.0
OVERRIDE_CONFIRM_SEC = 3.0
OVERRIDE_PUBLISH_SEC = 0.2


class OverrideSession:
    """
    One override session, numbered so mux_node can tell a new one from a cancelled one.

    The mux ends a session when a gamepad button other than LB is pressed;
    mux_active() then turns this side off too, and a keepalive cannot revive it.
    """

    def __init__(self) -> None:
        self.number = 0
        self._kept = 0.0
        self._started = 0.0
        self._confirmed = False
        self.ended = ''
        self._lock = threading.Lock()

    def start(self, now: float) -> None:
        with self._lock:
            self.number = secrets.randbelow(2**32 - 1) + 1
            self._kept = self._started = now
            self._confirmed = False
            self.ended = ''

    def keep(self, now: float) -> None:
        with self._lock:
            if self.number:
                self._kept = now

    def stop(self, reason: str = '') -> None:
        with self._lock:
            self.number = 0
            self.ended = reason

    def mux_active(self, active: bool, now: float) -> None:
        with self._lock:
            if not self.number:
                return
            if active:
                self._confirmed = True
            elif self._confirmed:
                self.number = 0
                self.ended = 'Ended by the controller.'

    def session(self, now: float) -> int:
        """Return the number to publish: the live session, or 0 once it lapses."""
        with self._lock:
            if self.number and now - self._kept > OVERRIDE_KEEPALIVE_SEC:
                self.number = 0
                self.ended = 'Ended. The page stopped responding.'
            if (self.number and not self._confirmed
                    and now - self._started > OVERRIDE_CONFIRM_SEC):
                self.number = 0
                self.ended = 'Teleop is not running.'
            return self.number

    def view(self, now: float) -> dict[str, Any]:
        on = bool(self.session(now))
        with self._lock:
            return {'on': on, 'confirmed': on and self._confirmed, 'ended': self.ended}


_override = OverrideSession()


def override_origin(origin: str | None, host: str | None) -> str | None:
    """
    Return `origin` when a driving lab page on this car may call /api/override.

    The lab pages (scripts/web/override.js) run on their own ports, so the
    browser needs a CORS header to read the reply.
    """
    if not origin or not host:
        return None
    o, h = urlparse(origin), urlparse(f'//{host}')
    ports = {a['port'] for a in APPS if a['exclusive']}
    if o.scheme == 'http' and o.hostname == h.hostname and o.port in ports:
        return origin
    return None


def battery_view(now: float) -> dict[str, Any] | None:
    """Return the current battery values for the status snapshot; stale after BATTERY_STALE_SEC."""
    latest = _battery.latest(now)
    if latest is None:
        return None
    (volts, amps), age = latest
    return {'volts': volts, 'amps': amps, 'age': age, 'stale': age > BATTERY_STALE_SEC}


def sample_system(now: float) -> None:
    """Record CPU temperature (C) and memory in use (%) for the overview charts."""
    temp = _read_soc_temp()
    mem = _read_memory()
    if temp is None or not mem or not mem.get('total'):
        return
    _system.add(now, round(temp, 1), round(100.0 * (1 - mem['available'] / mem['total']), 1))


class _RateSampler(Node):
    """Holds one BEST_EFFORT subscription per RATE_TOPICS entry and tracks arrivals."""

    def __init__(self, topics: list[str], window_sec: float = RATE_WINDOW_SEC) -> None:
        super().__init__('racecar_dashboard')
        self._window = window_sec
        self._stamps: dict[str, deque[float]] = {t: deque() for t in topics}
        self._lock = threading.Lock()
        # Subscriptions need a concrete message type, so attach_subscriptions()
        # creates each one once the topic and its type appear on the graph.
        self._qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._topics = list(topics)
        self._subs: dict[str, Any] = {}
        self.attached_at: float | None = None
        self._diagnostic_rates: dict[str, float] = {}
        self._diagnostic_last_update: dict[str, float] = {}
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._record_diagnostics,
            10,
        )
        # 1 Hz and latched: the autonomy card costs one small message a second.
        self._autonomy: tuple[dict[str, str], float] | None = None
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._autonomy_sub = self.create_subscription(
            DiagnosticStatus, '/autonomy/status', self._record_autonomy, latched
        )
        # 1 Hz means from pit_node; the raw 150 Hz /battery/* topics cost too much here.
        self._battery_sub = self.create_subscription(
            BatteryState, '/battery/state', self._record_battery, 10
        )
        self._override_pub = self.create_publisher(UInt32, '/mux/override', 10)
        self._override_sub = self.create_subscription(
            Bool, '/mux/override_active', self._record_override, latched
        )
        self.create_timer(OVERRIDE_PUBLISH_SEC, self._publish_override)

    def _publish_override(self) -> None:
        self._override_pub.publish(UInt32(data=_override.session(time.monotonic())))

    def _record_override(self, msg: Bool) -> None:
        _override.mux_active(bool(msg.data), time.monotonic())

    def _record(self, topic: str) -> None:
        now = time.monotonic()
        with self._lock:
            dq = self._stamps[topic]
            dq.append(now)
            cutoff = now - self._window
            while dq and dq[0] < cutoff:
                dq.popleft()

    def _record_diagnostics(self, msg: DiagnosticArray) -> None:
        updates: dict[str, float] = {}
        read_diagnostic_rates(msg, updates)
        if not updates:
            return
        now = time.monotonic()
        with self._lock:
            self._diagnostic_rates.update(updates)
            for topic in updates:
                self._diagnostic_last_update[topic] = now

    def _record_battery(self, msg: BatteryState) -> None:
        # BatteryState counts discharge as negative; the page shows draw as positive.
        _battery.add(time.time(), float(msg.voltage), -float(msg.current))

    def _record_autonomy(self, msg: DiagnosticStatus) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        with self._lock:
            self._autonomy = (values, time.monotonic())

    def autonomy(self) -> tuple[dict[str, str] | None, float | None]:
        """Return the last /autonomy/status values and their age in seconds."""
        with self._lock:
            if self._autonomy is None:
                return None, None
            values, at = self._autonomy
            return dict(values), time.monotonic() - at

    def attach_subscriptions(self) -> None:
        """Resolve each topic's type and create a subscription. Re-runnable; idempotent."""
        names_types = dict(self.get_topic_names_and_types())
        for topic in self._topics:
            if topic in self._subs:
                continue
            types = names_types.get(topic)
            if not types:
                continue
            try:
                msg_module, msg_name = types[0].rsplit('/', 1)
                pkg = msg_module.split('/')[0]
                module = __import__(f'{pkg}.msg', fromlist=[msg_name])
                msg_cls = getattr(module, msg_name)
            except (ValueError, ImportError, AttributeError) as exc:
                log.debug('Skipping %s: %s', topic, exc)
                continue
            self._subs[topic] = self.create_subscription(
                msg_cls, topic, self._arrival_callback(topic), self._qos, raw=True
            )
            if self.attached_at is None:
                self.attached_at = time.monotonic()

    def detach_subscriptions(self) -> None:
        """Drop the rate subscriptions and their counts until the next attach."""
        for sub in self._subs.values():
            self.destroy_subscription(sub)
        self._subs.clear()
        with self._lock:
            for dq in self._stamps.values():
                dq.clear()
        self.attached_at = None

    def warming(self) -> bool:
        """Return True until the rate window has filled since the subscriptions opened."""
        return self.attached_at is None or time.monotonic() - self.attached_at < self._window

    def _arrival_callback(self, topic: str) -> Callable[[Any], None]:
        return lambda _msg: self._record(topic)

    def measure_hz(self, topic: str) -> float | None:
        """
        Return the rate (Hz) for a topic, or None when no data.

        RealSense streams are read from /diagnostics; the rest are measured
        from arrival timestamps over the window.
        """
        with self._lock:
            now = time.monotonic()
            if topic in REALSENSE_DIAGNOSTIC_NAMES.values():
                last_update = self._diagnostic_last_update.get(topic)
                if last_update is None or now - last_update > self._window:
                    return None
                return self._diagnostic_rates[topic]
            dq = self._stamps.get(topic)
            if dq is None:
                return None
            cutoff = now - self._window
            while dq and dq[0] < cutoff:
                dq.popleft()
            if len(dq) < 2:
                return None
            return len(dq) / self._window

    def topic_list(self) -> list[tuple[str, list[str]]]:
        return sorted(self.get_topic_names_and_types(), key=lambda x: x[0])

    def node_list(self) -> list[str]:
        return sorted(f'/{n}' if not n.startswith('/') else n for n in self.get_node_names())


_sampler: _RateSampler | None = None
_overview: Any = None  # overview.Overview once main() runs


def _measure_hz(topic: str) -> float | None:
    """Return arrival rate (Hz) for a topic from the rclpy sampler, or None."""
    sampler = _sampler
    if sampler is None:
        return None
    return sampler.measure_hz(topic)


def _get_topic_list() -> list[str]:
    sampler = _sampler
    if sampler is None:
        return []
    return [name for name, _types in sampler.topic_list()]


def _get_node_list() -> list[str]:
    sampler = _sampler
    if sampler is None:
        return []
    return sampler.node_list()


WATCHDOG_LOG = Path.home() / 'logs' / 'latest' / 'watchdog.log'
# Byte offset of the last Clear, beside the log of this teleop session.
WATCHDOG_CLEARED = WATCHDOG_LOG.with_name('watchdog.cleared')


def _cleared_offset(size: int) -> int:
    try:
        offset = int(WATCHDOG_CLEARED.read_text().strip() or 0)
    except (OSError, ValueError):
        return 0
    # A shorter log is a new file, so an old mark does not apply.
    return offset if offset <= size else 0


def _read_watchdog_tail(n: int = 10, max_bytes: int = 4096) -> list[str]:
    """Return the last n lines of the watchdog log after the last Clear."""
    try:
        with open(WATCHDOG_LOG, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(_cleared_offset(size), size - max_bytes))
            tail = f.read()
    except OSError:
        return []
    return tail.decode(errors='replace').splitlines()[-n:]


def clear_watchdog_log() -> tuple[bool, str]:
    """Hide the lines logged so far; the file itself is kept."""
    try:
        size = WATCHDOG_LOG.stat().st_size
    except OSError:
        return True, 'Log is empty.'
    try:
        WATCHDOG_CLEARED.write_text(f'{size}\n')
    except OSError as exc:
        return False, str(exc)
    return True, 'Log cleared.'


def _collect_system_health() -> dict[str, dict[str, str]]:
    """Slow-refresh diagnostics: RTC battery + Pi under-voltage alarm."""
    volts = _read_battery_voltage()
    rtc_status, rtc_label = _classify_rtc(volts)
    uv_alarm = _read_under_voltage_alarm()
    if uv_alarm is None:
        uv_status, uv_label = ('dead', 'UNAVAILABLE')
    elif uv_alarm:
        uv_status, uv_label = ('dead', 'TRIPPED (5V dipped this boot)')
    else:
        uv_status, uv_label = ('healthy', 'OK')
    return {
        'rtc': {'label': 'RTC battery', 'status': rtc_status, 'detail': rtc_label},
        'under_voltage': {
            'label': 'Pi under-voltage alarm',
            'status': uv_status,
            'detail': uv_label,
        },
    }


# Watchdog states that change a card; healthy and starting leave the
# topic-based status alone.
WATCHDOG_VIEW = {
    'gave_up': ('dead', 'watchdog gave up; replug the device or restart teleop'),
    'no_device': ('stale', 'device not connected'),
    'restarting': ('stale', 'watchdog restarting'),
    'disabled': ('unsupervised', 'turned off'),
}


def watchdog_view(status: str, watched: dict[str, Any] | None) -> tuple[str, str]:
    """Return (card status, detail) after applying the watchdog's state for a node."""
    if not watched or watched.get('state') not in WATCHDOG_VIEW:
        return status, ''
    view_status, detail = WATCHDOG_VIEW[watched['state']]
    if watched['state'] == 'restarting':
        detail += f" ({watched.get('restarts', 0)} so far)"
    return view_status, detail


def autonomy_view(values: dict[str, str] | None, age: float | None) -> dict[str, Any]:
    """
    Card for the autonomy service: status (card color), headline, and detail lines.

    healthy: localized; stale: running but not localized; dead: status stopped
    arriving without the node saying it stopped; unsupervised (gray): stopped.
    """
    if values is None or values.get('mode') in (None, '', 'off'):
        return {
            'status': 'unsupervised',
            'headline': 'Stopped',
            'lines': ['Start mapping or load a map in the map viewer.'],
            'port': MAPVIEW_PORT,
        }
    mode, name = values.get('mode', ''), values.get('map') or '(no map)'
    headline = f'{mode.capitalize()} on {name}'
    if mode == 'localization' and values.get('localizer'):
        headline += f" ({values['localizer']})"
    if age is not None and age > AUTONOMY_STALE_SEC:
        return {
            'status': 'dead',
            'headline': headline,
            'lines': [f'No status for {age:.0f} s. The autonomy service may have stopped.'],
            'port': MAPVIEW_PORT,
        }
    localized = values.get('localized') == 'true'
    lines = ['Localized' if localized else 'Not localized']
    if mode == 'localization' and values.get('scan_fit'):
        lines.append(f"Scan fit {float(values['scan_fit']):.0%}")
    if mode == 'localization' and values.get('pose_sd_m'):
        sd = f"{float(values['pose_sd_m']):.2f} m, {float(values.get('pose_sd_deg') or 0):.0f} deg"
        lines.append(f'Spread {sd}')
    if mode == 'mapping':
        lines.append(f"Saved {values.get('last_save') or 'not yet'}")
    return {
        'status': 'healthy' if localized else 'stale',
        'headline': headline,
        'lines': lines,
        'port': MAPVIEW_PORT,
    }


def unit_states(units: list[str], run: Callable[..., Any] = subprocess.run) -> dict[str, str]:
    """Return systemctl is-active for each unit: active, inactive, failed, activating, ..."""
    try:
        out = run(['systemctl', 'is-active', *units], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return dict.fromkeys(units, 'unknown')
    lines = out.stdout.split()
    return {u: (lines[i] if i < len(lines) else 'unknown') for i, u in enumerate(units)}


def app_list(states: dict[str, str]) -> list[dict[str, Any]]:
    return [
        {**app, 'state': states.get(app['unit'], 'unknown') if app['unit'] else 'unavailable'}
        for app in APPS
    ]


def _app_units() -> list[str]:
    return [a['unit'] for a in APPS if a['unit']]


def open_app(target: str, run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """
    Start `target` if it is not running.

    An exclusive app first stops the other exclusive one and a running
    autonomy run (a mapping run is saved first); the page asks the user before
    calling this.
    """
    app = next((a for a in APPS if a['id'] == target), None)
    if app is None:
        return False, f'Unknown app: {target}'
    if app['unit'] is None:
        return False, f'{app["name"]} is not available.'
    if app['exclusive']:
        live = ('active', 'activating')
        if (unit_states([app['unit']], run)[app['unit']] not in live
                and unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in live):
            mapping = last_run().get('mode') == 'mapping'
            message = autonomy_action('save-stop' if mapping else 'stop', run)
            if unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in live:
                return False, f'Autonomy is still running. {message}'
        return activate(target, run)
    if unit_states([app['unit']], run)[app['unit']] in ('active', 'activating'):
        return True, f'{app["name"]} is running.'
    if not app['startable']:
        return False, f'{app["name"]} is stopped. Run: racecar service start {target}'
    error = _systemctl('start', app['unit'], run)
    if error:
        return False, error
    return True, f'{app["name"]} started.'


def clear_apps(run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """Stop every running utility to free CPU; core apps and the dashboard keep running."""
    units = [a['unit'] for a in APPS if a['group'] == 'utilities']
    states = unit_states(units, run)
    stopped, errors = 0, []
    for unit in units:
        if states.get(unit) in ('active', 'activating'):
            error = _systemctl('stop', unit, run)
            if error:
                errors.append(error)
            else:
                stopped += 1
    if errors:
        return False, ' '.join(errors)
    return True, 'Utilities stopped.' if stopped else 'No utilities running.'


def _systemctl(verb: str, unit: str, run: Callable[..., Any]) -> str:
    """Run systemctl without a password prompt; return '' or a short error."""
    try:
        r = run(
            ['systemctl', '--no-ask-password', verb, unit],
            capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT_SEC,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    if r.returncode == 0:
        return ''
    if 'auth' in (r.stderr or '').lower():
        return f'No permission to {verb} {unit}. Run scripts/setup_user_env.sh.'
    return (r.stderr or '').strip().splitlines()[-1] if r.stderr else f'{verb} failed'


def app_ready(port: int, timeout: float = READY_TIMEOUT_SEC) -> bool:
    """Return True once the app's web server answers on localhost."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    try:
        conn.request('GET', '/')
        conn.getresponse().read(0)
        return True
    except (OSError, http.client.HTTPException):
        return False
    finally:
        conn.close()


def activate(target: str, run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """Make `target` the one running exclusive app, or stop them all for 'none'."""
    labs = {a['id']: a['unit'] for a in APPS if a['exclusive']}
    if target != 'none' and target not in labs:
        return False, f'Unknown dashboard: {target}'
    states = unit_states(list(labs.values()), run)

    for lab_id, unit in labs.items():
        if lab_id != target and states.get(unit) in ('active', 'activating'):
            error = _systemctl('stop', unit, run)
            if error:
                return False, error
    if target == 'none':
        return True, 'Lab dashboards stopped.'
    if states.get(labs[target]) not in ('active', 'activating'):
        error = _systemctl('start', labs[target], run)
        if error:
            return False, error
    name = next(a['name'] for a in APPS if a['id'] == target)
    return True, f'{name} active.'


def last_run(path: Path | None = None) -> dict[str, str]:
    """Return the mode and map of the last autonomy run from its mode file, or {}."""
    try:
        lines = (path or AUTONOMY_MODE_FILE).read_text().splitlines()
    except OSError:
        return {}
    values = dict(line.split('=', 1) for line in lines if '=' in line)
    mode, name = values.get('AUTONOMY_MODE', ''), values.get('AUTONOMY_MAP', '')
    if mode not in ('mapping', 'localization') or not name:
        return {}
    return {'mode': mode, 'map': name}


def feature_list(
    states: dict[str, str],
    wanted: dict[str, bool],
    running: dict[str, bool],
    busy: bool,
    last: dict[str, str],
) -> list[dict[str, Any]]:
    """Return one tile per feature with its state: active, activating, or inactive."""
    teleop = states.get(TELEOP_UNIT, 'unknown')
    out = []
    for item in FEATURES:
        if item['id'] == 'autonomy':
            state = states.get(AUTONOMY_UNIT, 'unknown')
            out.append({**item, 'state': state, 'last': last})
            continue
        on = teleop == 'active' and running[item['id']]
        pending = busy or teleop in ('activating', 'reloading') or (
            teleop == 'active' and running[item['id']] != wanted[item['id']]
        )
        state = 'activating' if pending else ('active' if on else 'inactive')
        out.append({**item, 'state': state, 'wanted': wanted[item['id']]})
    return out


def map_tool(*argv: str, run: Callable[..., Any] = subprocess.run) -> tuple[int, str]:
    """Run `racecar map ARGV`; return its exit code and last output line."""
    try:
        r = run(
            [*MAP_TOOL, *argv],
            capture_output=True, text=True, timeout=MAP_TOOL_TIMEOUT_SEC,
            stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    lines = [line for line in (r.stdout + r.stderr).splitlines() if line.strip()]
    return r.returncode, lines[-1] if lines else ''


def describe(values: dict[str, bool]) -> str:
    names = {f['id']: f['name'] for f in FEATURES}
    return ' '.join(f'{names[k]} {"on" if v else "off"}.' for k, v in values.items())


def apply_features(
    values: dict[str, bool],
    run: Callable[..., Any] = subprocess.run,
    sleep: Callable[[float], None] = time.sleep,
) -> str:
    """
    Restart teleop with `values`; a running autonomy run is stopped first and resumed after.

    Stopping it first lets a localization run start again from the car's last
    pose rather than from where that run began.
    """
    features.write(values)
    resume = unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in ('active', 'activating')
    if resume:
        code, line = map_tool('stop', run=run)
        if code != 0:
            return f'Could not stop autonomy: {line}'
    error = _systemctl('restart', TELEOP_UNIT, run)
    if error:
        return error
    message = describe(values)
    if resume:
        sleep(TELEOP_SETTLE_SEC)
        code, line = map_tool('resume', run=run)
        message += f' {line}' if code == 0 else f' Autonomy did not resume: {line}'
    return message


def autonomy_action(action: str, run: Callable[..., Any] = subprocess.run) -> str:
    if action == 'resume':
        return map_tool('resume', run=run)[1]
    if action == 'save-stop':
        code, line = map_tool('save', run=run)
        if code != 0:
            return f'Not stopped. {line}'
    if action in ('stop', 'save-stop'):
        return map_tool('stop', run=run)[1]
    raise ValueError(f'unknown autonomy action {action!r}')


class FeatureJob:
    """One feature change at a time, in a thread; its message is shown on the page."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.busy = False
        self.message = ''

    def start(self, label: str, work: Callable[[], str]) -> bool:
        with self._lock:
            if self.busy:
                return False
            self.busy, self.message = True, label
        threading.Thread(target=self._run, args=(work,), daemon=True).start()
        return True

    def _run(self, work: Callable[[], str]) -> None:
        try:
            message = work()
        except Exception as exc:  # noqa: BLE001  shown to the user, not raised in a thread
            log.exception('feature change failed')
            message = f'Failed: {exc}'
        log.info('features: %s', message)
        with self._lock:
            self.busy, self.message = False, message

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {'busy': self.busy, 'message': self.message}


_features_job = FeatureJob()


def features_view() -> dict[str, Any]:
    job = _features_job.state()
    items = feature_list(
        unit_states([TELEOP_UNIT, AUTONOMY_UNIT]),
        features.read(),
        features.running(),
        job['busy'],
        last_run(),
    )
    return {'items': items, 'job': job}


def change_feature(name: str, on: bool, action: str = '') -> tuple[bool, str]:
    """Start a feature change in the background; return whether it started."""
    if name == 'autonomy':
        if action not in ('resume', 'stop', 'save-stop'):
            return False, 'Unknown action.'
        label = 'Starting autonomy...' if action == 'resume' else 'Stopping autonomy...'
        work: Callable[[], str] = lambda: autonomy_action(action)  # noqa: E731
    else:
        try:
            values = features.change(features.read(), name, on)
        except ValueError as exc:
            return False, str(exc)
        label = 'Restarting teleop...'
        work = lambda: apply_features(values)  # noqa: E731
    if not _features_job.start(label, work):
        return False, 'Another change is running.'
    return True, label


def _refresh_apps() -> None:
    apps = app_list(unit_states(_app_units()))
    with _status_lock:
        _latest_status['apps'] = apps


_last_view = 0.0
_wake = threading.Event()


def note_view() -> bool:
    """Record a page request; return True when monitoring was idle until now."""
    global _last_view
    idle = not viewing()
    _last_view = time.monotonic()
    if idle:
        _wake.set()
    return idle


def viewing() -> bool:
    return time.monotonic() - _last_view < VIEW_IDLE_SEC


def _monitor_loop() -> None:
    """Background thread that refreshes the cached status snapshot while a page polls."""
    last_system_health = 0.0
    system_health = _collect_system_health()
    while _monitor_running:
        try:
            sample_system(time.time())
            if not viewing():
                if _sampler is not None and _sampler.attached_at is not None:
                    _sampler.detach_subscriptions()
                    log.info('No page open; rate sampling stopped')
                _wake.wait(MONITOR_PERIOD_SEC)
                _wake.clear()
                continue
            # Publishers can appear after the dashboard starts, so retry the
            # topic-to-type lookup each tick.
            if _sampler is not None:
                _sampler.attach_subscriptions()

            topics = _get_topic_list()
            nodes = _get_node_list()

            watchdog = read_watchdog_state()
            watched = (
                (watchdog or {}).get('nodes', {}) if not (watchdog or {}).get('stale') else {}
            )
            node_status = {}
            for name, cfg in MONITORED.items():
                if cfg.get('publisher'):
                    # pit_node subscribes, so the topic exists without a driver.
                    present = _sampler is not None and _sampler.count_publishers(cfg['topic']) > 0
                else:
                    present = cfg['topic'] in topics
                if present:
                    status = 'healthy'
                elif cfg['supervised']:
                    status = 'dead'
                else:
                    status = 'unsupervised'
                status, detail = watchdog_view(status, watched.get(name))
                node_status[name] = {
                    'label': cfg['label'],
                    'topic': cfg['topic'],
                    'alive': present,
                    'supervised': cfg['supervised'],
                    'status': status,
                    'detail': detail,
                }

            rates = {}
            warming = _sampler is not None and _sampler.warming()
            for topic in DISPLAY_RATE_TOPICS:
                hz = _measure_hz(topic)
                if warming and hz is None and topic not in REALSENSE_DIAGNOSTIC_NAMES.values():
                    rates[topic] = {'hz': None, 'stale': False, 'warming': True}
                else:
                    rates[topic] = {'hz': hz, 'stale': hz is None or hz < 0.5}

            now = time.monotonic()
            if now - last_system_health >= SYSTEM_HEALTH_REFRESH_SEC:
                system_health = _collect_system_health()
                last_system_health = now

            snapshot = {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'nodes': node_status,
                'node_list': nodes,
                'topic_list': topics,
                'rates': rates,
                'system_health': system_health,
                'watchdog_log': _read_watchdog_tail(),
                'log_dir': str(Path.home() / 'logs' / 'latest'),
                'autonomy': autonomy_view(*(_sampler.autonomy() if _sampler else (None, None))),
                'apps': app_list(unit_states(_app_units())),
                'battery': battery_view(time.time()),
                'host': socket.gethostname(),
            }

            with _status_lock:
                _latest_status.update(snapshot)

        except Exception:  # noqa: BLE001
            log.exception('Error in monitor loop')

        _wake.wait(MONITOR_PERIOD_SEC)
        _wake.clear()


def get_status() -> dict[str, Any]:
    """Return the most recent status snapshot (non-blocking)."""
    with _status_lock:
        return dict(_latest_status)


# ---------------------------------------------------------------------------
# HTTP handler; the page is scripts/dashboard.html
# ---------------------------------------------------------------------------

_HTML_PATH = Path(__file__).resolve().parent / 'dashboard.html'
_LAUNCH_PATH = Path(__file__).resolve().parent / 'dashboard_launch.html'


def _read_page(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8')
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>{path.name} unreadable: {exc}</pre>'


def _load_dashboard_html() -> str:
    return _read_page(_HTML_PATH)


DASHBOARD_HTML = _load_dashboard_html()


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve GET / (HTML) and GET /api/status (JSON snapshot)."""

    def do_GET(self) -> None:
        asset = webui.static_file(self.path)
        if asset is not None:
            self.send_response(200)
            self.send_header('Content-Type', asset[1])
            self.send_header('Content-Length', str(len(asset[0])))
            self.send_header('Cache-Control', webui.CACHE_CONTROL)
            self.end_headers()
            self.wfile.write(asset[0])
            return
        url = urlparse(self.path)
        query = parse_qs(url.query)
        if url.path == '/':
            self._send_html(_load_dashboard_html())
        elif url.path == '/launch':
            self._send_html(_read_page(_LAUNCH_PATH))
        elif url.path == '/api/status':
            if note_view():
                # The cached snapshot may be minutes old; refresh what is cheap.
                _refresh_apps()
            if query.get('refresh') == ['1']:
                _refresh_apps()
            self._send_json(200, {
                **get_status(),
                'override': _override.view(time.monotonic()),
                'features': features_view(),
            })
        elif url.path == '/api/history':
            now = time.time()
            self._send_json(200, {
                'now': now,
                'window_sec': BATTERY_WINDOW_SEC,
                'battery': _battery.buckets(now),
                'system': _system.buckets(now),
            })
        elif url.path == '/api/overview':
            if _overview is None:
                self._send_json(503, {})
            else:
                _overview.touch()
                self._send_json(200, _overview.state())
        elif url.path in ('/api/overview/color.jpg', '/api/overview/depth.jpg'):
            name = url.path.rsplit('/', 1)[1].split('.')[0]
            if _overview is not None:
                _overview.touch_cameras()
            frame = _overview.jpeg(name) if _overview is not None else None
            if frame is None:
                self.send_error(404)
            else:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(frame)
        elif url.path == '/api/ready':
            app = next((a for a in APPS if a['id'] == query.get('app', [''])[0]), None)
            if app is None:
                self._send_json(404, {'ready': False})
            else:
                self._send_json(200, {'ready': app_ready(app['port']), 'port': app['port']})
        else:
            self.send_error(404)

    def _send_html(self, text: str) -> None:
        content = text.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_json(self, code: int, data: Any, headers: dict[str, str] | None = None) -> None:
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        if self.path == '/api/clear-apps':
            ok, message = clear_apps()
            _refresh_apps()
            log.info('clear apps: %s', message)
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        if self.path == '/api/clear-log':
            ok, message = clear_watchdog_log()
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        if self.path not in ('/api/open', '/api/override', '/api/feature'):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}')
            target = str(body.get('app', ''))
            action = str(body.get('action', ''))
            feature, on = str(body.get('feature', '')), bool(body.get('on'))
        except (ValueError, AttributeError):
            target = action = feature = ''
            on = False
        if self.path == '/api/feature':
            ok, message = change_feature(feature, on, action)
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        if self.path == '/api/override':
            self._override(action)
            return
        ok, message = open_app(target)
        _refresh_apps()
        log.info('open %s: %s', target, message)
        self._send_json(200 if ok else 400, {'ok': ok, 'message': message})

    def _override(self, action: str) -> None:
        now = time.monotonic()
        origin = override_origin(self.headers.get('Origin'), self.headers.get('Host'))
        cors = {'Access-Control-Allow-Origin': origin, 'Vary': 'Origin'} if origin else {}
        if action == 'start':
            _override.start(now)
            log.warning('Safety override turned on from %s', self.client_address[0])
        elif action == 'keep':
            _override.keep(now)
        elif action == 'stop':
            if _override.number:
                log.info('Safety override turned off')
            _override.stop()
        else:
            self._send_json(400, {'ok': False, 'message': 'Unknown action.'}, cors)
            return
        self._send_json(200, {'ok': True, **_override.view(now)}, cors)

    def log_message(self, fmt: str, *args: Any) -> None:
        """Suppress default per-request logging."""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    global _monitor_running, _sampler, _overview

    logdir = Path.home() / 'logs' / 'latest'
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stderr)]
    try:
        if logdir.exists():
            handlers.append(logging.FileHandler(logdir / 'dashboard.log'))
    except OSError:
        pass

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    rclpy.init()
    sampler = _RateSampler(RATE_TOPICS)
    _sampler = sampler
    from overview import Overview

    _overview = Overview(sampler)

    def _spin_sampler() -> None:
        try:
            rclpy.spin(sampler)
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:  # noqa: BLE001
            log.exception('rclpy.spin terminated')

    spinner = threading.Thread(target=_spin_sampler, daemon=True)
    spinner.start()
    log.info('rclpy sampler spinning')

    monitor = threading.Thread(target=_monitor_loop, daemon=True)
    monitor.start()
    log.info('Background monitor started')

    server = ThreadingHTTPServer(('0.0.0.0', PORT), DashboardHandler)
    server.daemon_threads = True
    log.info('Dashboard listening on http://0.0.0.0:%d', PORT)

    def _shutdown(signum: int, _frame: FrameType | None) -> None:
        global _monitor_running
        log.info('Received signal %d, shutting down', signum)
        _monitor_running = False
        threading.Thread(target=server.shutdown).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _monitor_running = False
        server.server_close()
        monitor.join(timeout=5)
        # rclpy shutdown wakes the spin thread; destroying the node while it
        # waits can block or abort the interpreter at exit.
        rclpy.try_shutdown()
        spinner.join(timeout=2)
        log.info('Dashboard stopped')


if __name__ == '__main__':
    main()
