#!/usr/bin/env python3
"""
RACECAR Neo battery guard: low-battery warning and safe shutdown.

Reads /battery/state, estimates charge and time left (racecar_neo_ros2_driver
battery), and publishes /battery/estimate (DiagnosticStatus, latched) once a
second. Levels:

- ok: nothing to do.
- warn: minutes left at or under warn_min for HOLD_SEC.
- countdown: COUNTDOWN_SEC to shutdown. Starts when the 30 s median voltage
  is at or under the floor, or (with shutdown_enabled) when minutes left stay
  at or under shutdown_min for HOLD_SEC. An estimate countdown can be canceled
  once per boot on /battery/cancel_shutdown; a floor countdown cannot.
- off: the shutdown sequence has run.

Stale or implausible readings never warn or start a countdown; a countdown
already running finishes. Every reading is appended to a CSV under
<data root>/health/battery/, flushed per line so a power cut loses at most
one. Runs as racecar-battery-guard.service, independent of teleop.

--dry-run logs the shutdown sequence instead of running it, for a bench check
with --floor-cell-volts raised.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable, Sequence
import csv
from dataclasses import dataclass, field
import json
import logging
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from types import FrameType
from typing import Any, IO

sys.path.insert(0, str(Path(__file__).resolve().parent))

from battery_alert import Announcer  # noqa: E402
from racecar_neo_ros2_driver import car_profile, data_root  # noqa: E402
from racecar_neo_ros2_driver.battery import (  # noqa: E402
    BatteryConfig,
    CapacityLearner,
    detect_cells,
    Estimate,
    estimate,
    Sample,
)

SCRIPT_DIR = Path(__file__).resolve().parent

TICK_SEC = 1.0
HOLD_SEC = 60.0
FLOOR_MEDIAN_SEC = 30.0
COUNTDOWN_SEC = 60.0
# Minutes left must rise this far above warn_min to clear a warning.
WARN_CLEAR_MARGIN_MIN = 2.0
# Kept for the estimate window; also bounds memory.
HISTORY_SEC = 600.0
LAST_SHUTDOWN_FILE = 'last_shutdown.json'
CSV_FIELDS = (
    'time',
    'volts',
    'amps',
    'soc',
    'minutes_left',
    'power_w',
    'capacity_mah',
    'cells',
    'level',
    'event',
)

DRIVING_UNITS = (
    'racecar-webteleop.service',
    'racecar-linefollow.service',
    'racecar-wallfollow.service',
)
AUTONOMY_UNIT = 'racecar-autonomy.service'
TELEOP_UNIT = 'racecar-teleop.service'
STEP_TIMEOUT_SEC = 30.0

log = logging.getLogger('battery_guard')


@dataclass
class Guard:
    """Level state machine over battery samples; no ROS, time passed in."""

    cfg: BatteryConfig
    samples: deque[Sample] = field(default_factory=deque)
    level: str = 'ok'
    trigger: str = ''
    shutdown_at: float | None = None
    cells: int | None = None
    cells_known: bool = True
    canceled: bool = False
    last: Estimate = field(default_factory=lambda: Estimate(valid=False))
    _warn_since: float | None = None
    _clear_since: float | None = None
    _short_since: float | None = None
    _learned_t: float | None = None

    def __post_init__(self) -> None:
        self.learner = CapacityLearner(self.cfg)
        if self.cfg.cells:
            self.cells = self.cfg.cells

    def add(self, sample: Sample) -> bool:
        """Add a sample; return True when it set the cell count."""
        detected = False
        if self.cells is None:
            found = detect_cells(sample[1])
            self.cells_known = found is not None
            # An unknown battery still gets a floor: the nearest count at 3.8 V per cell.
            self.cells = found or max(1, round(sample[1] / 3.8))
            detected = True
        self.samples.append(sample)
        while self.samples and self.samples[0][0] < sample[0] - HISTORY_SEC:
            self.samples.popleft()
        return detected

    def tick(self, now: float) -> list[str]:
        """Advance to `now`; return the events that happened (for the log)."""
        if self.level == 'off' or self.cells is None:
            return []
        est = estimate(self.samples, now, self.cfg, self.cells, self.learner.capacity_mah)
        newest = self.samples[-1] if self.samples else None
        if newest is not None and newest[0] != self._learned_t:
            self._learned_t = newest[0]
            self.learner.update(newest[0], newest[2], est.soc if est.valid else None)
        self.last = est
        events: list[str] = []

        if self.level == 'countdown':
            assert self.shutdown_at is not None
            if now >= self.shutdown_at:
                self.level = 'off'
                events.append('shutdown')
            return events
        if not est.valid:
            self._warn_since = self._clear_since = self._short_since = None
            return events

        if self._floor_reached(now):
            return self._start_countdown(now, 'floor')

        minutes = est.minutes_left
        if minutes is not None and minutes <= self.cfg.shutdown_min and self.cfg.shutdown_enabled:
            self._short_since = self._short_since if self._short_since is not None else now
            if now - self._short_since >= HOLD_SEC and not self.canceled:
                return self._start_countdown(now, 'estimate')
        else:
            self._short_since = None

        if self.level == 'ok':
            if minutes is not None and minutes <= self.cfg.warn_min:
                self._warn_since = self._warn_since if self._warn_since is not None else now
                if now - self._warn_since >= HOLD_SEC:
                    self.level = 'warn'
                    events.append('warn')
            else:
                self._warn_since = None
        elif self.level == 'warn':
            if minutes is not None and minutes > self.cfg.warn_min + WARN_CLEAR_MARGIN_MIN:
                self._clear_since = self._clear_since if self._clear_since is not None else now
                if now - self._clear_since >= HOLD_SEC:
                    self.level = 'ok'
                    self._warn_since = self._clear_since = None
                    events.append('clear')
            else:
                self._clear_since = None
        return events

    def cancel(self, now: float) -> bool:
        """Cancel an estimate countdown, once per boot; a floor countdown cannot be canceled."""
        if self.level != 'countdown' or self.trigger != 'estimate' or self.canceled:
            return False
        self.canceled = True
        self.level, self.trigger, self.shutdown_at = 'warn', '', None
        self._short_since = None
        return True

    def _floor_reached(self, now: float) -> bool:
        assert self.cells is not None
        recent = [v for t, v, _ in self.samples if t >= now - FLOOR_MEDIAN_SEC]
        if len(recent) < FLOOR_MEDIAN_SEC * 0.8:
            return False
        recent.sort()
        return recent[len(recent) // 2] <= self.cfg.floor_volts(self.cells)

    def _start_countdown(self, now: float, trigger: str) -> list[str]:
        self.level, self.trigger, self.shutdown_at = 'countdown', trigger, now + COUNTDOWN_SEC
        return [f'countdown {trigger}']


def status_values(guard: Guard, now: float) -> dict[str, str]:
    """Return the /battery/estimate key/values; empty strings for unknown values."""
    est = guard.last

    def num(value: float | None, digits: int) -> str:
        return '' if value is None else f'{value:.{digits}f}'

    left = None if guard.shutdown_at is None else max(0.0, guard.shutdown_at - now)
    return {
        'level': guard.level,
        'trigger': guard.trigger,
        'valid': 'true' if est.valid else 'false',
        'volts': num(est.volts, 2),
        'soc': num(est.soc, 0),
        'power_w': num(est.power_w, 1),
        'minutes_left': num(est.minutes_left, 0),
        'capacity_mah': num(guard.learner.capacity_mah, 0),
        'cells': '' if guard.cells is None else str(guard.cells),
        'cells_known': 'true' if guard.cells_known else 'false',
        'shutdown_at': num(guard.shutdown_at, 1),
        'shutdown_in': num(left, 0),
        'cancel_used': 'true' if guard.canceled else 'false',
    }


class CsvLog:
    """Append rows to a CSV, flushed and synced per row so a power cut loses at most one."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        new = not path.exists()
        self.path = path
        self._file: IO[str] = open(path, 'a', newline='', buffering=1)
        self._writer = csv.writer(self._file)
        if new:
            self.write(CSV_FIELDS)

    def write(self, row: Sequence[Any]) -> None:
        self._writer.writerow(row)
        self._file.flush()
        os.fsync(self._file.fileno())

    def close(self) -> None:
        self._file.close()


def csv_row(sample: Sample | None, guard: Guard, event: str) -> list[str]:
    est = guard.last

    def num(value: float | None, digits: int) -> str:
        return '' if value is None else f'{value:.{digits}f}'

    t, volts, amps = sample if sample is not None else (time.time(), None, None)
    return [
        f'{t:.3f}',
        num(volts, 4),
        num(amps, 4),
        num(est.soc, 2),
        num(est.minutes_left, 2),
        num(est.power_w, 2),
        num(guard.learner.capacity_mah, 0),
        '' if guard.cells is None else str(guard.cells),
        guard.level,
        event,
    ]


def shutdown_sequence(
    run: Callable[..., Any] = subprocess.run, dry_run: bool = False
) -> list[str]:
    """
    Stop recording, driving, autonomy, and teleop, then power off.

    Each step is bounded and its failure does not stop the next; the car is
    turned off whatever happened. Returns one line per step for the log.
    """
    steps: list[tuple[str, list[str]]] = [
        ('stop recording', [sys.executable, str(SCRIPT_DIR / 'racecar_log.py'), 'stop']),
        *[
            (f'stop {u}', ['systemctl', '--no-ask-password', 'stop', u])
            for u in (*DRIVING_UNITS, AUTONOMY_UNIT, TELEOP_UNIT)
        ],
        ('power off', ['systemctl', '--no-ask-password', '--check-inhibitors=no', 'poweroff']),
    ]
    lines = []
    for name, argv in steps:
        if dry_run:
            lines.append(f'{name}: dry run, not run')
            continue
        try:
            r = run(argv, capture_output=True, text=True, timeout=STEP_TIMEOUT_SEC)
            ok = r.returncode == 0
            detail = '' if ok else f' ({(r.stderr or r.stdout or "").strip()[:200]})'
            lines.append(f'{name}: {"ok" if ok else "failed"}{detail}')
        except (OSError, subprocess.TimeoutExpired) as exc:
            lines.append(f'{name}: failed ({exc})')
    return lines


def write_last_shutdown(folder: Path, trigger: str, now: float, guard: Guard) -> Path:
    """Record a low-battery shutdown for the dashboard to show after the next boot."""
    path = folder / LAST_SHUTDOWN_FILE
    est = guard.last
    path.write_text(
        json.dumps({'time': now, 'trigger': trigger, 'volts': est.volts, 'soc': est.soc})
    )
    return path


def load_config(floor_cell_volts: float | None = None) -> BatteryConfig:
    """Return config/battery.yaml with the car profile's override on top."""
    from ament_index_python.packages import get_package_share_directory

    config_dir = Path(get_package_share_directory('racecar_neo_ros2_driver')) / 'config'
    params = car_profile.node_params('battery', 'battery_guard', config_dir)
    if floor_cell_volts is not None:
        params['floor_cell_volts'] = floor_cell_volts
    return BatteryConfig.from_params(params)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--dry-run', action='store_true', help='log the shutdown, do not run it')
    parser.add_argument('--floor-cell-volts', type=float, help='override the floor (bench check)')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Empty

    cfg = load_config(args.floor_cell_volts)
    guard = Guard(cfg)
    folder = data_root.data_dir('health') / 'battery'
    suffix = '_guard_dryrun.csv' if args.dry_run else '_guard.csv'
    csv_log = CsvLog(folder / (time.strftime('%Y%m%d_%H%M%S') + suffix))
    log.info(
        'Battery guard: floor %.2f V per cell, warn %g min, shutdown %g min (%s)%s; log %s',
        cfg.floor_cell_volts,
        cfg.warn_min,
        cfg.shutdown_min,
        'on' if cfg.shutdown_enabled else 'warns only',
        ', dry run' if args.dry_run else '',
        csv_log.path,
    )

    lock = threading.Lock()
    pending: list[Sample] = []
    # Terminal messages go out in order on one thread, so a slow tmux call
    # never holds up the tick.
    announcer = Announcer()
    announcements: queue.Queue[tuple[list[str], str, float | None, float | None, float]] = (
        queue.Queue()
    )

    def announce_worker() -> None:
        while True:
            item = announcements.get()
            try:
                announcer.update(*item)
            except Exception:  # noqa: BLE001
                log.exception('Terminal message failed')
            finally:
                announcements.task_done()

    threading.Thread(target=announce_worker, daemon=True).start()

    def announce(events: list[str], now: float) -> None:
        announcements.put((events, guard.level, guard.shutdown_at, guard.last.minutes_left, now))

    LEVELS = {'ok': DiagnosticStatus.OK, 'warn': DiagnosticStatus.WARN}

    class BatteryGuardNode(Node):
        def __init__(self) -> None:
            super().__init__('battery_guard')
            latched = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub = self.create_publisher(DiagnosticStatus, '/battery/estimate', latched)
            self.create_subscription(BatteryState, '/battery/state', self._on_state, 10)
            self.create_subscription(Empty, '/battery/cancel_shutdown', self._on_cancel, 10)
            self.create_timer(TICK_SEC, self._tick)

        def _on_state(self, msg: BatteryState) -> None:
            with lock:
                pending.append((time.time(), float(msg.voltage), -float(msg.current)))

        def _on_cancel(self, _msg: Empty) -> None:
            with lock:
                if guard.cancel(time.time()):
                    log.warning('Shutdown canceled')
                    csv_log.write(csv_row(None, guard, 'canceled'))
                    announce(['canceled'], time.time())

        def _tick(self) -> None:
            now = time.time()
            with lock:
                new, pending[:] = list(pending), []
                for s in new:
                    if guard.add(s):
                        if guard.cells_known:
                            log.info('Battery: %d cells at %.2f V', guard.cells, s[1])
                        else:
                            log.warning('Unknown battery: %.2f V fits no cell count', s[1])
                events = guard.tick(now)
                for s in new:
                    csv_log.write(csv_row(s, guard, ''))
                for event in events:
                    log.warning('Battery %s (%s)', event, status_values(guard, now))
                    csv_log.write(csv_row(None, guard, event))
                values = status_values(guard, now)
                announce(events, now)
            msg = DiagnosticStatus()
            msg.name = 'battery'
            msg.level = LEVELS.get(guard.level, DiagnosticStatus.ERROR)
            msg.message = values['level']
            msg.values = [KeyValue(key=k, value=v) for k, v in values.items()]
            self._pub.publish(msg)
            if 'shutdown' in events:
                threading.Thread(target=self._shutdown, daemon=True).start()

        def _shutdown(self) -> None:
            # The final message reaches the terminals before anything stops.
            announcements.join()
            if not args.dry_run:
                write_last_shutdown(folder, guard.trigger, time.time(), guard)
            for line in shutdown_sequence(dry_run=args.dry_run):
                log.warning('Shutdown: %s', line)
                csv_log.write(csv_row(None, guard, line))

    rclpy.init()
    node = BatteryGuardNode()

    def _stop(signum: int, _frame: FrameType | None) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        csv_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
