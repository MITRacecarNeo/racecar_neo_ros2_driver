#!/usr/bin/env python3
"""
Event watch behind `racecar health shake` and `poke`.

Everything that shows a loose contact is sampled together while the
operator moves the car one timed step at a time:

    5 V input       EXT5V as fast as vcgencmd answers; a sample under 4.75 V
    throttling      a throttle bit not set at the start, read once a second
    kernel log      USB disconnects and resets, under-voltage, NVMe and PCIe errors
    data root       a 4 KB write with fsync once a second; a failure or a write
                    over 1 s
    PIT link        a gap over 50 ms between /imu/lsm9ds1 stamps (5 ms apart
                    while the Teensy link holds), and a CRC failure, a
                    reconnect or a Teensy reset in pit_node's link report
    sensor rates    /scan in 2 s bins, and the RealSense rates on /diagnostics,
                    under half of the still step's

Each event is charged to the step it falls in, so a fault is named for the
axis shaken or the connector poked.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from diagnose import Check, FAIL, OK  # noqa: E402, I100  sibling scripts
import sysinfo  # noqa: E402

LINK_GAP_SEC = 0.05
RATE_DROP_FRAC = 0.5
# /scan arrives 0.12 to 0.41 s apart at about 6.5 Hz; 1 s bins fall under
# half by chance.
SCAN_BIN_SEC = 2.0
SLOW_WRITE_SEC = 1.0
# An event this soon after a step ends still belongs to it: the kernel logs
# and the rates trail the cause.
GRACE_SEC = 1.0
WRITE_FILE = '.racecar-health-write'
LINK_TOPIC = '/imu/lsm9ds1'
SCAN_TOPIC = '/scan'

KERNEL_EVENTS = (
    ('usb', re.compile(r'USB disconnect|reset \S+ USB device', re.IGNORECASE)),
    ('under-voltage', re.compile(r'undervoltage', re.IGNORECASE)),
    (
        'nvme',
        re.compile(
            r'nvme\S*:?\s.*(timeout|I/O error|resetting controller|controller is down|abort)',
            re.IGNORECASE,
        ),
    ),
    ('pcie', re.compile(r'PCIe Bus Error|AER: (Corrected|Uncorrected|Multiple)|link down', re.I)),
)


@dataclass
class Event:
    t: float
    kind: str
    detail: str


@dataclass
class Step:
    name: str
    prompt: str
    seconds: float
    start: float = 0.0
    end: float = 0.0


def kernel_event(line: str) -> tuple[str, str] | None:
    """Return (kind, line) for a kernel log line that means a fault, or None."""
    for kind, pattern in KERNEL_EVENTS:
        if pattern.search(line):
            return kind, line.strip()
    return None


def new_throttle_bits(start: int, now: int) -> int:
    """Bits set now that were not at the start; live bits count whenever set."""
    return (now & ~start) | (now & 0xF)


class Watch:
    """Runs the samplers on threads between start() and stop()."""

    def __init__(self, data_dir: Path, clock: Callable[[], float] = time.monotonic) -> None:
        self.data_dir = data_dir
        self.clock = clock
        self.events: list[Event] = []
        self.ext5v: list[tuple[float, float]] = []
        self.scan_times: list[float] = []
        self.realsense: list[tuple[float, str, float]] = []
        self.link_count = 0
        self.t0 = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._journal: subprocess.Popen[str] | None = None

    def add(self, kind: str, detail: str, t: float | None = None) -> None:
        with self._lock:
            self.events.append(Event(self.clock() if t is None else t, kind, detail))

    def start(self) -> None:
        self.t0 = self.clock()
        for target in (self._ext5v_loop, self._second_loop, self._kernel_loop, self._ros_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()
        if self._journal is not None:
            self._journal.terminate()
        for thread in self._threads:
            thread.join(timeout=5)
        (self.data_dir / WRITE_FILE).unlink(missing_ok=True)

    def _ext5v_loop(self) -> None:
        while not self._stop.is_set():
            volts = sysinfo.read_ext5v()
            if volts is None:
                return
            t = self.clock()
            self.ext5v.append((t, volts))
            if volts < sysinfo.EXT5V_MIN_VOLTS:
                self.add('5 V input', f'{volts:.3f} V', t)

    def _second_loop(self) -> None:
        start = sysinfo.read_throttled()[0] or 0
        seen = 0
        path = self.data_dir / WRITE_FILE
        while not self._stop.wait(1.0):
            flags = sysinfo.read_throttled()[0]
            if flags is not None:
                new = new_throttle_bits(start, flags) & ~seen
                if new:
                    names = sysinfo.decode_throttled(f'throttled=0x{new:x}')[1]
                    self.add('throttling', ', '.join(names))
                    seen |= new
            began = self.clock()
            try:
                with open(path, 'wb') as f:
                    f.write(os.urandom(4096))
                    f.flush()
                    os.fsync(f.fileno())
            except OSError as exc:
                self.add('data write', str(exc))
                continue
            took = self.clock() - began
            if took > SLOW_WRITE_SEC:
                self.add('data write', f'took {took:.1f} s')

    def _kernel_loop(self) -> None:
        try:
            self._journal = subprocess.Popen(
                ['journalctl', '-k', '-f', '-n', '0', '-o', 'cat'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except OSError:
            return
        assert self._journal.stdout is not None
        for line in self._journal.stdout:
            found = kernel_event(line)
            if found:
                self.add(*found)

    def _ros_loop(self) -> None:
        try:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from diagnostic_msgs.msg import DiagnosticArray
            from sensor_msgs.msg import Imu, LaserScan
        except ImportError:
            return
        rclpy.init()
        node = rclpy.create_node('racecar_health_watch')
        last = [0.0]

        def on_imu(msg: Any) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.link_count += 1
            if last[0] and stamp - last[0] > LINK_GAP_SEC:
                self.add('PIT link', f'{(stamp - last[0]) * 1000:.0f} ms without a frame')
            last[0] = stamp

        previous: dict[str, float] = {}

        def on_diagnostics(msg: Any) -> None:
            rates: dict[str, float] = {}
            sysinfo.read_diagnostic_rates(msg, rates)
            t = self.clock()
            with self._lock:
                self.realsense.extend((t, topic, hz) for topic, hz in rates.items())
            report = sysinfo.read_link_status(msg)
            if report is None:
                return
            for event in link_events(previous, report):
                self.add('PIT link', event, t)
            previous.clear()
            previous.update(report)

        node.create_subscription(Imu, LINK_TOPIC, on_imu, qos_profile_sensor_data)
        node.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            lambda _m: self.scan_times.append(self.clock()),
            qos_profile_sensor_data,
        )
        node.create_subscription(DiagnosticArray, '/diagnostics', on_diagnostics, 10)
        try:
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def link_events(previous: dict[str, float], report: dict[str, float]) -> list[str]:
    """Return what the link report counts that the one before it did not."""
    if not previous:
        return []
    names = (
        ('crc_failures', 'CRC failure'),
        ('reconnects', 'reconnect'),
        ('resets', 'Teensy reset'),
    )
    out = []
    for key, name in names:
        new = int(report.get(key, 0) - previous.get(key, 0))
        if new > 0:
            out.append(f'{new} {name}{"s" if new > 1 else ""}')
    return out


def run_steps(
    steps: Sequence[Step],
    clock: Callable[[], float],
    ask: Callable[[str], str] = input,
    out: Callable[[str], None] = print,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Prompt each step, start it on Enter, and time it; sets each step's start and end."""
    for i, step in enumerate(steps, 1):
        out('')
        out(f'Step {i} of {len(steps)}: {step.prompt}')
        ask('Press Enter to start.')
        step.start = clock()
        out(f'Go. {step.seconds:.0f} seconds.')
        sleep(step.seconds)
        step.end = clock()
        out('Stop.')


def rate_drops(
    steps: Sequence[Step],
    scan_times: Sequence[float],
    realsense: Sequence[tuple[float, str, float]],
) -> list[Event]:
    """
    Return an event where /scan or a RealSense stream falls under half of the still step's rate.

    The first step is the still one; its rates are the baseline. A stream
    already down when a step starts is charged to the step it went down in,
    not to the steps after it, until it comes back.
    """
    if not steps:
        return []
    still = steps[0]
    span = max(still.end - still.start, 1e-6)
    scan_base = sum(1 for t in scan_times if still.start <= t < still.end) / span
    calm = [(tp, hz) for t, tp, hz in realsense if still.start <= t < still.end]
    base = {
        topic: statistics.median(hz for tp, hz in calm if tp == topic)
        for topic in {tp for tp, _ in calm}
    }
    events: list[Event] = []
    for step in steps[1:]:
        if scan_base:
            floor = scan_base * RATE_DROP_FRAC
            lead = sum(1 for s in scan_times if step.start - SCAN_BIN_SEC <= s < step.start)
            down = lead / SCAN_BIN_SEC < floor
            t = step.start
            while t + SCAN_BIN_SEC <= step.end:
                hz = sum(1 for s in scan_times if t <= s < t + SCAN_BIN_SEC) / SCAN_BIN_SEC
                if hz < floor and not down:
                    detail = f'{SCAN_TOPIC} {hz:.1f} Hz'
                    events.append(Event(t + SCAN_BIN_SEC, 'sensor rate', detail))
                down = hz < floor
                t += SCAN_BIN_SEC
        for topic, rate in base.items():
            floor = rate * RATE_DROP_FRAC
            earlier = [hz for when, tp, hz in realsense if tp == topic and when < step.start]
            down = bool(earlier) and earlier[-1] < floor
            for when, tp, hz in realsense:
                if tp != topic or not step.start <= when < step.end + GRACE_SEC:
                    continue
                if hz < floor and not down:
                    events.append(Event(when, 'sensor rate', f'{topic} {hz:.0f} Hz'))
                down = hz < floor
    return events


def step_of(t: float, steps: Sequence[Step]) -> Step | None:
    for step in steps:
        if step.start <= t < step.end + GRACE_SEC:
            return step
    return None


def step_checks(
    group: str,
    steps: Sequence[Step],
    events: Sequence[Event],
    ext5v: Sequence[tuple[float, float]],
) -> list[Check]:
    """Return one row per step, FAIL with its events; events outside every step get a row."""
    checks: list[Check] = []
    for step in steps:
        mine = [e for e in events if step_of(e.t, steps) is step]
        volts = [v for t, v in ext5v if step.start <= t < step.end]
        low = f'min {min(volts):.3f} V' if volts else 'no 5 V samples'
        if not mine:
            checks.append(Check(group, step.name, OK, f'{low}, no events'))
            continue
        checks.append(Check(group, step.name, FAIL, f'{low}; {summarize(mine)}'))
    between = [e for e in events if step_of(e.t, steps) is None]
    if between:
        checks.append(Check(group, 'between steps', FAIL, summarize(between)))
    return checks


def summarize(events: Sequence[Event]) -> str:
    counts: dict[str, int] = {}
    for e in events:
        counts[e.kind] = counts.get(e.kind, 0) + 1
    kinds = ', '.join(f'{n} {kind}' for kind, n in counts.items())
    return f'{kinds}; first: {events[0].kind} {events[0].detail}'
