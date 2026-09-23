#!/usr/bin/env python3
"""RACECAR Neo web dashboard: live node/topic monitor on port 8080 (stdlib HTTP + rclpy)."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import logging
from pathlib import Path
import signal
import sys
import threading
import time
from types import FrameType
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Shared with scripts/diagnose.py. Both are run by path rather than imported
# as a package, so the sibling has to be put on the path explicitly.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sysinfo import (  # noqa: E402
    classify_rtc as _classify_rtc,
    read_diagnostic_rates,
    read_rtc_voltage as _read_battery_voltage,
    read_under_voltage_alarm as _read_under_voltage_alarm,
    REALSENSE_DIAGNOSTIC_NAMES,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PORT = 8080
RATE_WINDOW_SEC = 3.0
SYSTEM_HEALTH_REFRESH_SEC = 60.0

# supervised: the watchdog restarts this node. An unsupervised card that loses
# its topic shows grey instead of red.
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
}
_monitor_running = True


# ---------------------------------------------------------------------------
# Rate measurement via rclpy subscriptions
# ---------------------------------------------------------------------------


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
        self._diagnostic_rates: dict[str, float] = {}
        self._diagnostic_last_update: dict[str, float] = {}
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._record_diagnostics,
            10,
        )

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


def _read_watchdog_tail(n: int = 10, max_bytes: int = 4096) -> list[str]:
    """Return the last n lines of the watchdog log without reading the whole file."""
    logfile = Path.home() / 'logs' / 'latest' / 'watchdog.log'
    try:
        with open(logfile, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            tail = f.read()
    except OSError:
        return []
    return tail.decode(errors='replace').splitlines()[-n:]


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


def _monitor_loop() -> None:
    """Background thread that continuously refreshes the cached status snapshot."""
    last_system_health = 0.0
    system_health = _collect_system_health()
    while _monitor_running:
        try:
            # Publishers can appear after the dashboard starts, so retry the
            # topic-to-type lookup each tick.
            if _sampler is not None:
                _sampler.attach_subscriptions()

            topics = _get_topic_list()
            nodes = _get_node_list()

            node_status = {}
            for name, cfg in MONITORED.items():
                present = cfg['topic'] in topics
                if present:
                    status = 'healthy'
                elif cfg['supervised']:
                    status = 'dead'
                else:
                    status = 'unsupervised'
                node_status[name] = {
                    'label': cfg['label'],
                    'topic': cfg['topic'],
                    'alive': present,
                    'supervised': cfg['supervised'],
                    'status': status,
                }

            rates = {}
            for topic in DISPLAY_RATE_TOPICS:
                hz = _measure_hz(topic)
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
            }

            with _status_lock:
                _latest_status.update(snapshot)

        except Exception:  # noqa: BLE001
            log.exception('Error in monitor loop')

        for _ in range(30):
            if not _monitor_running:
                break
            time.sleep(0.1)


def get_status() -> dict[str, Any]:
    """Return the most recent status snapshot (non-blocking)."""
    with _status_lock:
        return dict(_latest_status)


# ---------------------------------------------------------------------------
# HTTP handler; the page is scripts/dashboard.html
# ---------------------------------------------------------------------------

_HTML_PATH = Path(__file__).resolve().parent / 'dashboard.html'


def _load_dashboard_html() -> str:
    try:
        return _HTML_PATH.read_text(encoding='utf-8')
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>dashboard.html unreadable: {exc}</pre>'


DASHBOARD_HTML = _load_dashboard_html()


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve GET / (HTML) and GET /api/status (JSON snapshot)."""

    def do_GET(self) -> None:
        if self.path == '/':
            self._serve_html()
        elif self.path == '/api/status':
            self._serve_status()
        else:
            self.send_error(404)

    def _serve_html(self) -> None:
        content = DASHBOARD_HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _serve_status(self) -> None:
        data = get_status()
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        """Suppress default per-request logging."""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    global _monitor_running, _sampler

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

    server = HTTPServer(('0.0.0.0', PORT), DashboardHandler)
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
        try:
            sampler.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()
        log.info('Dashboard stopped')


if __name__ == '__main__':
    main()
