#!/usr/bin/env python3
"""RACECAR Neo node watchdog: monitor + restart the control pipeline and sensors."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import FrameType
from typing import Any, IO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sysinfo import under_voltage_alarm_path, WATCHDOG_STATE_FILE  # noqa: E402

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POLL_INTERVAL = 5  # seconds between health checks
SHM_CLEANUP_INTERVAL = 60  # seconds between FastRTPS shm orphan sweeps

# Restart policy. A failed node restarts at once, then waits RESTART_COOLDOWN
# doubled for each further restart that did not hold HEALTHY_RESET_SEC of health
# (30, 60, 120, 240 s, capped at MAX_BACKOFF). After MAX_RESTARTS such restarts
# the node is given up until its device is replugged, it recovers on its own,
# or teleop restarts.
RESTART_COOLDOWN = 30
MAX_BACKOFF = 300
MAX_RESTARTS = 5
HEALTHY_RESET_SEC = 60
# No restart before a node's topic has been seen once, or this long after the
# watchdog starts. Its first poll runs before discovery lists teleop's topics.
STARTUP_GRACE_SEC = 60
# Repeat interval for the no_device, restarting and gave_up log lines.
REMINDER_SEC = 600

# Written by scripts/launch_teleop.sh into the session log directory.
LAUNCH_ARGS_FILE = 'launch_args'

PACKAGE = 'racecar_neo_ros2_driver'

# Executable-path substrings matched against /proc/<pid>/cmdline. Specific
# enough to distinguish the node binary from `ros2 launch` wrappers.
DRIVER_LIB = '/install/racecar_neo_ros2_driver/lib/racecar_neo_ros2_driver'
SLLIDAR_LIB = '/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node'
# RealSense ships from /opt/ros/jazzy (apt package), not the workspace install tree.
REALSENSE_EXECUTABLE_PATH = '/realsense2_camera/realsense2_camera_node'


def _usb_device_present(usb_id: str) -> bool:
    """Check whether a USB vendor:product ID appears in lsusb output."""
    try:
        result = subprocess.run(
            ['lsusb'],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return usb_id.lower() in result.stdout.lower()
    except (subprocess.TimeoutExpired, OSError):
        return False


# enable_arg: the teleop.launch.py argument that disables the subsystem; a node
# whose argument was false at launch is not supervised.
NODES: dict[str, dict[str, Any]] = {
    # ----- Control pipeline (safety-critical) -----
    'pit': {
        'topic': '/imu/lsm9ds1',  # pit_node's steady telemetry output proves the UART link
        'launch': 'pit.launch.py',
        'device_check': lambda: os.path.exists('/dev/neo-pit-pcb'),
        'device_label': '/dev/neo-pit-pcb (Teensy PIT UART)',
        'kill_pattern': f'{DRIVER_LIB}/pit_node',
        'process_pattern': f'{DRIVER_LIB}/pit_node',
    },
    'throttle': {
        'topic': '/motor',  # downstream of throttle, alive iff throttle alive
        'launch': 'throttle.launch.py',
        'device_check': lambda: True,
        'device_label': 'throttle_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/throttle_node',
        'process_pattern': f'{DRIVER_LIB}/throttle_node',
    },
    'mux': {
        'topic': '/mux_out',
        'launch': 'mux.launch.py',
        'device_check': lambda: True,
        'device_label': 'mux_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/mux_node',
        'process_pattern': f'{DRIVER_LIB}/mux_node',
    },
    'gamepad': {
        'topic': '/gamepad_drive',
        'launch': 'gamepad.launch.py',
        'device_check': lambda: True,
        'device_label': 'gamepad_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/gamepad_node',
        'process_pattern': f'{DRIVER_LIB}/gamepad_node',
    },
    # ----- Sensors -----
    # /imu/fused merges /imu/realsense with pit_node's /imu/lsm9ds1.
    'imu_fusion': {
        'topic': '/imu/fused',
        'launch': 'imu_fusion.launch.py',
        'device_check': lambda: True,
        'device_label': 'imu_fusion_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/imu_fusion_node',
        'process_pattern': f'{DRIVER_LIB}/imu_fusion_node',
        'enable_arg': 'imu_fusion_enable',
    },
    'lidar': {
        'topic': '/scan',
        'launch': 'lidar.launch.py',
        'device_check': lambda: os.path.exists('/dev/lidar'),
        'device_label': '/dev/lidar (RPLIDAR)',
        'kill_pattern': SLLIDAR_LIB,
        'process_pattern': SLLIDAR_LIB,
        'enable_arg': 'lidar_enable',
        # sllidar can stall silently while its process and advertisement stay
        # up. 5 s spans many 7.2 Hz scans and stays under RESTART_COOLDOWN.
        'freshness_sec': 5.0,
    },
    'realsense': {
        'topic': '/camera/color',
        'launch': 'realsense.launch.py',
        'device_check': lambda: _usb_device_present('8086:0b3a'),
        'device_label': 'USB 8086:0b3a (RealSense D435i)',
        'kill_pattern': 'realsense2_camera_node',
        'process_pattern': REALSENSE_EXECUTABLE_PATH,
        'enable_arg': 'realsense_enable',
    },
}


# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

_running = True
_child_procs: dict[str, tuple[subprocess.Popen[bytes], IO[str]]] = {}

log = logging.getLogger('watchdog')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _clean_fastrtps_orphans() -> int:
    """Remove 0-byte /dev/shm/fastrtps_port* segments and stranded lock files."""
    shm = Path('/dev/shm')
    removed = 0
    for port in shm.glob('fastrtps_port*'):
        if port.name.endswith('_el'):
            continue
        try:
            if port.stat().st_size == 0:
                (shm / f'{port.name}_el').unlink(missing_ok=True)
                (shm / f'sem.{port.name}_mutex').unlink(missing_ok=True)
                port.unlink(missing_ok=True)
                removed += 1
        except OSError:
            pass
    for el in shm.glob('fastrtps_port*_el'):
        data = shm / el.name[: -len('_el')]
        if not data.exists():
            try:
                (shm / f'sem.{data.name}_mutex').unlink(missing_ok=True)
                el.unlink(missing_ok=True)
                removed += 1
            except OSError:
                pass
    return removed


def read_cmdlines(proc: Path = Path('/proc')) -> list[str]:
    """Return every process's command line, arguments joined by spaces."""
    out = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except OSError:
            continue  # exited during the scan, or not ours to read
        if raw:
            out.append(raw.replace(b'\0', b' ').decode(errors='replace').strip())
    return out


def process_running(pattern: str, cmdlines: Iterable[str]) -> bool:
    return any(pattern in line for line in cmdlines)


def disabled_args(path: Path) -> set[str]:
    """Return the `<name>_enable` launch arguments set false in a launch_args file."""
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return set()
    out = set()
    for arg in lines:
        key, sep, value = arg.strip().partition(':=')
        if sep and key.endswith('_enable') and value.strip().lower() in ('false', '0', 'no'):
            out.add(key)
    return out


class _FreshnessMonitor:
    """
    Track last-arrival monotonic time for topics that need a freshness check.

    Subscriptions are BEST_EFFORT (publishers like sllidar use BEST_EFFORT too)
    and re-attached every poll so a topic that comes up after a restart wires in.
    """

    _QOS = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(self, node: Node, topics: Iterable[str]) -> None:
        self._node = node
        self._topics = list(topics)
        self._lock = threading.Lock()
        self._last: dict[str, float] = {}
        self._subs: dict[str, Any] = {}

    def attach(self) -> None:
        names_types = dict(self._node.get_topic_names_and_types())
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
                log.debug('freshness: cannot subscribe to %s: %s', topic, exc)
                continue
            self._subs[topic] = self._node.create_subscription(
                msg_cls, topic, self._mark_callback(topic), self._QOS
            )

    def _mark_callback(self, topic: str) -> Callable[[Any], None]:
        return lambda _msg: self._mark(topic)

    def _mark(self, topic: str) -> None:
        with self._lock:
            self._last[topic] = time.monotonic()

    def reset(self, topic: str) -> None:
        """Forget the last-seen time for a topic (call after a restart kick)."""
        with self._lock:
            self._last.pop(topic, None)
        sub = self._subs.pop(topic, None)
        if sub is not None:
            try:
                self._node.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass

    def age(self, topic: str) -> float | None:
        """Seconds since last message on topic, or None if never seen."""
        with self._lock:
            ts = self._last.get(topic)
        if ts is None:
            return None
        return time.monotonic() - ts


def _get_active_topics(node: Node) -> set[str]:
    """Return the set of currently advertised ROS 2 topics."""
    try:
        return {name for name, _types in node.get_topic_names_and_types()}
    except Exception as exc:  # noqa: BLE001
        log.warning('node.get_topic_names_and_types failed: %s', exc)
        return set()


def stale_age(
    fresh_window: float | None,
    topic_alive: bool,
    proc_alive: bool,
    since_restart: float,
    age: float | None,
) -> float | None:
    """
    Return the topic's message age when it counts as stale, else None.

    Only nodes with a freshness window are judged, only while the topic and
    process are up, and only once a full window has passed since the last restart.
    """
    if not fresh_window or not (topic_alive and proc_alive) or since_restart < fresh_window:
        return None
    if age is not None and age > fresh_window:
        return age
    return None


def failure_reason(topic_alive: bool, proc_alive: bool, stale: float | None) -> str | None:
    """Return why a node counts as down, or None when it is healthy."""
    if not topic_alive and not proc_alive:
        return 'topic+process down'
    if not topic_alive:
        return 'topic not advertised'
    if not proc_alive:
        return 'process not running'
    if stale is not None:
        return f'topic stale ({stale:.1f}s)'
    return None


# ---------------------------------------------------------------------------
# Restart policy
# ---------------------------------------------------------------------------


def backoff(restarts: int) -> float:
    """Seconds to wait after `restarts` consecutive restarts that did not hold."""
    if restarts <= 0:
        return 0.0
    return float(min(RESTART_COOLDOWN * 2 ** (restarts - 1), MAX_BACKOFF))


@dataclass
class NodeState:
    """
    One supervised node's restart bookkeeping. Times are time.monotonic().

    state: starting (startup grace), healthy, restarting (failed; restarted or
    waiting out its backoff), no_device, gave_up, or disabled.
    """

    state: str = 'starting'
    restarts: int = 0  # consecutive; cleared after HEALTHY_RESET_SEC of health
    last_restart: float = float('-inf')
    seen: bool = False  # topic healthy at least once since the watchdog started
    healthy_since: float | None = None
    reason: str = ''
    entered_wall: float = field(default_factory=time.time)
    last_reminder: float = 0.0


Event = tuple[int, str]  # (logging level, message)


def _enter(st: NodeState, state: str, now: float) -> bool:
    if st.state == state:
        return False
    st.state = state
    st.entered_wall = time.time()
    st.last_reminder = now
    return True


def _reminder_due(st: NodeState, now: float) -> bool:
    if now - st.last_reminder >= REMINDER_SEC:
        st.last_reminder = now
        return True
    return False


def step(
    st: NodeState,
    now: float,
    started: float,
    failure: str | None,
    device_check: Callable[[], bool],
    device_label: str,
    disabled_by: str | None = None,
) -> tuple[bool, list[Event]]:
    """
    Advance one node by one poll; return (restart now, log events).

    Events are emitted on state changes, with a reminder every REMINDER_SEC
    for no_device, restarting and gave_up. device_check runs only for a
    failed node.
    """
    events: list[Event] = []
    if disabled_by:
        if _enter(st, 'disabled', now):
            events.append(
                (logging.INFO, f'disabled at launch ({disabled_by}:=false); not supervised')
            )
        return False, events

    if failure is None:
        st.seen = True
        st.reason = ''
        if st.healthy_since is None:
            st.healthy_since = now
        previous = st.state
        if _enter(st, 'healthy', now) and previous != 'starting':
            events.append((logging.INFO, f'healthy again (was {previous})'))
        if st.restarts and now - st.healthy_since >= HEALTHY_RESET_SEC:
            st.restarts = 0
        return False, events

    st.healthy_since = None
    st.reason = failure
    if not st.seen and now - started < STARTUP_GRACE_SEC:
        st.state = 'starting'
        return False, events

    if not device_check():
        if _enter(st, 'no_device', now):
            events.append(
                (
                    logging.WARNING,
                    f'{failure}; device {device_label} not connected; not restarting',
                )
            )
        elif _reminder_due(st, now):
            events.append((logging.WARNING, f'still waiting for {device_label}'))
        return False, events

    if st.state == 'no_device':
        st.restarts = 0
        events.append((logging.INFO, f'device {device_label} back; restart budget reset'))
    elif st.state == 'gave_up':
        if _reminder_due(st, now):
            events.append(
                (logging.ERROR, f'{failure}; given up (replug the device or restart teleop)')
            )
        return False, events

    if now - st.last_restart < backoff(st.restarts):
        if st.state != 'restarting':
            _enter(st, 'restarting', now)
        elif _reminder_due(st, now):
            events.append((logging.WARNING, f'{failure}; waiting to restart'))
        return False, events

    if st.restarts >= MAX_RESTARTS:
        _enter(st, 'gave_up', now)
        events.append(
            (
                logging.ERROR,
                f'{failure}; giving up after {st.restarts} restarts without '
                f'{HEALTHY_RESET_SEC}s of health (replug the device or restart teleop)',
            )
        )
        return False, events

    st.restarts += 1
    st.last_restart = now
    _enter(st, 'restarting', now)
    events.append(
        (logging.WARNING, f'{failure}; restart {st.restarts}/{MAX_RESTARTS} ({device_label})')
    )
    return True, events


def state_snapshot(states: dict[str, NodeState], started_wall: float) -> dict[str, Any]:
    """Return the watchdog_state.json payload."""
    iso = lambda t: datetime.fromtimestamp(t).isoformat(timespec='seconds')  # noqa: E731
    return {
        'updated': iso(time.time()),
        'started': iso(started_wall),
        'poll_sec': POLL_INTERVAL,
        'max_restarts': MAX_RESTARTS,
        'nodes': {
            name: {
                'state': st.state,
                'restarts': st.restarts,
                'reason': st.reason,
                'since': iso(st.entered_wall),
            }
            for name, st in states.items()
        },
    }


def write_state(path: Path, payload: dict[str, Any]) -> None:
    """Write the state file by rename, so readers never see a partial file."""
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(payload, indent=1) + '\n')
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Process control
# ---------------------------------------------------------------------------


def _log_dir() -> Path:
    """Resolve ~/logs/latest to the real session directory."""
    latest = Path.home() / 'logs' / 'latest'
    if latest.is_symlink() or latest.is_dir():
        return latest.resolve()
    fallback = Path.home() / 'logs'
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def _restart_node(name: str, cfg: dict[str, Any]) -> None:
    """Kill the node's processes and launch its per-node launch file as a child."""
    old = _child_procs.get(name)
    if old:
        old_proc, old_fh = old
        if old_proc.poll() is None:
            log.info('%s: terminating stale child PID %d', name, old_proc.pid)
            old_proc.terminate()
            try:
                old_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                old_proc.kill()
        try:
            old_fh.close()
        except OSError:
            pass

    # Kill any stale system-wide processes (e.g. from teleop.launch) that might
    # still hold the device open. SIGTERM first, then SIGKILL after 2 s.
    kill_pat = cfg.get('kill_pattern')
    if kill_pat:
        try:
            r = subprocess.run(
                ['pkill', '-f', kill_pat],
                capture_output=True,
                timeout=5,
            )
            if r.returncode == 0:
                log.info('%s: sent SIGTERM to processes matching "%s"', name, kill_pat)
                time.sleep(2)
                r2 = subprocess.run(
                    ['pkill', '-9', '-f', kill_pat],
                    capture_output=True,
                    timeout=5,
                )
                if r2.returncode == 0:
                    log.info('%s: sent SIGKILL to surviving processes', name)
                time.sleep(1)
        except subprocess.TimeoutExpired:
            pass

    restart_log = _log_dir() / f'restart_{name}.log'
    log.info('%s: restarting via %s; log: %s', name, cfg['launch'], restart_log)

    log_fh = open(restart_log, 'a')  # noqa: SIM115
    log_fh.write(f'=== {datetime.now().isoformat(timespec="seconds")} restart ===\n')
    log_fh.flush()
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(_log_dir())

    proc = subprocess.Popen(
        ['ros2', 'launch', PACKAGE, cfg['launch']],
        stdout=log_fh,
        stderr=subprocess.STDOUT,
        env=env,
    )
    _child_procs[name] = (proc, log_fh)
    log.info('%s: launched PID %d', name, proc.pid)


def _cleanup_children() -> None:
    """Terminate all child processes we spawned and close their log handles."""
    for name, (proc, fh) in _child_procs.items():
        if proc.poll() is None:
            log.info('Stopping child %s (PID %d)', name, proc.pid)
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        try:
            fh.close()
        except OSError:
            pass


def _signal_handler(signum: int, _frame: FrameType | None) -> None:
    global _running
    log.info('Received signal %d, shutting down', signum)
    _running = False


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def main() -> None:
    logdir = _log_dir()
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stderr)]
    try:
        handlers.append(logging.FileHandler(logdir / 'watchdog.log'))
    except OSError as exc:
        print(f'Warning: cannot open watchdog.log: {exc}', file=sys.stderr)

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    disabled = disabled_args(logdir / LAUNCH_ARGS_FILE)
    log.info('Watchdog started; monitoring: %s', ', '.join(NODES.keys()))
    log.info('Log directory: %s', logdir)
    if disabled:
        log.info('Disabled at launch: %s', ', '.join(sorted(disabled)))

    rclpy.init()
    intro_node = Node('racecar_watchdog')

    fresh_topics = [cfg['topic'] for cfg in NODES.values() if 'freshness_sec' in cfg]
    freshness = _FreshnessMonitor(intro_node, fresh_topics)

    def _spin() -> None:
        try:
            rclpy.spin(intro_node)
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:  # noqa: BLE001
            log.exception('rclpy.spin terminated')

    spinner = threading.Thread(target=_spin, daemon=True)
    spinner.start()

    volt_alarm_path = under_voltage_alarm_path()
    volt_alarm_seen = False
    if volt_alarm_path is None:
        log.info('Pi under-voltage alarm: rpi_volt hwmon not found (skipping check)')
    else:
        try:
            if volt_alarm_path.read_text().strip() == '1':
                log.warning(
                    'Pi under-voltage alarm already set at watchdog start '
                    '(under-voltage occurred earlier this boot)'
                )
                volt_alarm_seen = True
            else:
                log.info('Pi under-voltage alarm armed (%s)', volt_alarm_path)
        except OSError:
            pass

    startup_removed = _clean_fastrtps_orphans()
    if startup_removed:
        log.info('Cleaned %d FastRTPS SHM orphan(s) at startup', startup_removed)

    started = time.monotonic()
    started_wall = time.time()
    states = {name: NodeState() for name in NODES}
    state_path = logdir / WATCHDOG_STATE_FILE

    last_shm_cleanup = time.monotonic()
    while _running:
        topics = _get_active_topics(intro_node)
        freshness.attach()
        cmdlines = read_cmdlines()

        if time.monotonic() - last_shm_cleanup >= SHM_CLEANUP_INTERVAL:
            last_shm_cleanup = time.monotonic()
            n = _clean_fastrtps_orphans()
            if n:
                log.info('Cleaned %d FastRTPS SHM orphan(s)', n)

        if volt_alarm_path is not None and not volt_alarm_seen:
            try:
                if volt_alarm_path.read_text().strip() == '1':
                    log.warning(
                        'Pi under-voltage alarm tripped: 5V rail dipped below '
                        'threshold (USB devices may have reset). See '
                        'docs/troubleshooting.md, "Boot brownout with ethernet attached".'
                    )
                    volt_alarm_seen = True
            except OSError:
                pass

        for name, cfg in NODES.items():
            st = states[name]
            now = time.monotonic()
            topic = cfg['topic']
            topic_alive = topic in topics
            proc_alive = process_running(cfg['process_pattern'], cmdlines)

            stale = stale_age(
                cfg.get('freshness_sec'),
                topic_alive,
                proc_alive,
                now - st.last_restart,
                freshness.age(topic),
            )
            failure = failure_reason(topic_alive, proc_alive, stale)

            child = _child_procs.get(name)
            if child:
                child_proc, child_fh = child
                if child_proc.poll() is not None:
                    log.warning(
                        '%s: restarted child PID %d exited with code %s',
                        name,
                        child_proc.pid,
                        child_proc.returncode,
                    )
                    try:
                        child_fh.close()
                    except OSError:
                        pass
                    _child_procs.pop(name, None)

            enable_arg = cfg.get('enable_arg')
            restart, events = step(
                st,
                now,
                started,
                failure,
                cfg['device_check'],
                cfg['device_label'],
                enable_arg if enable_arg in disabled else None,
            )
            for level, message in events:
                log.log(level, '%s: %s', name, message)
            if restart:
                _restart_node(name, cfg)
                # Drop the stale subscription so it re-binds to the new publisher.
                if cfg.get('freshness_sec'):
                    freshness.reset(topic)

        try:
            write_state(state_path, state_snapshot(states, started_wall))
        except OSError as exc:
            log.debug('cannot write %s: %s', state_path, exc)

        # Sleep in short increments so we respond to signals promptly.
        for _ in range(POLL_INTERVAL * 10):
            if not _running:
                break
            time.sleep(0.1)

    _cleanup_children()
    try:
        intro_node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    rclpy.try_shutdown()
    log.info('Watchdog stopped')


if __name__ == '__main__':
    main()
