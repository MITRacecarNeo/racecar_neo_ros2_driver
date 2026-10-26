#!/usr/bin/env python3
"""
`racecar health`: hardware checks that sample over time and keep a report.

    racecar health [all]    power, then nvme
    racecar health power    the Pi's 5 V input for 30 s, throttling, the PMIC
                            alarm, the ARM clock, the battery
    racecar health nvme     the data drive: placement, a 256 MB write and
                            read, free space, SMART, kernel log errors

Each run writes report.md, report.json and samples.csv to
<data root>/health/<time>/. Exit 1 when any check fails. The battery row
needs teleop running (pit_node publishes /battery/state); without it the row
is skipped.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import asdict
from datetime import datetime
import json
import mmap
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

from diagnose import (  # noqa: E402, I100  sibling script, after the path insert
    Check,
    DATA_WARN_FREE_GB,
    DATA_WARN_PCT,
    FAIL,
    MARK,
    nvme_check,
    OK,
    overall,
    paint,
    SKIP,
    use_color,
    WARN,
)
import sysinfo  # noqa: E402

CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
STEPS = ('power', 'nvme')
POWER_SEC = 30.0

# Throttle bits 0-3 are live; 16-19 are sticky for the boot.
LIVE_MASK = 0xF
STICKY_MASK = 0xF0000

SPEED_TEST_MB = 256
SPEED_CHUNK_MB = 4
SPEED_FILE = '.racecar-health-speed'
# The Pi 5's PCIe link gives a few hundred MB/s; under these the drive or
# link is in trouble.
SPEED_WARN_WRITE_MBS = 100.0
SPEED_WARN_READ_MBS = 150.0

# Kernel log lines that mean the drive or its PCIe link misbehaved.
NVME_ERROR = re.compile(
    r'nvme\S*:?\s.*(timeout|I/O error|resetting controller|controller is down|abort|failed)'
    r'|Buffer I/O error on dev nvme',
    re.IGNORECASE,
)
PCIE_ERROR = re.compile(
    r'PCIe Bus Error|AER: (Corrected|Uncorrected|Multiple)|link down|link reset',
    re.IGNORECASE,
)

Run = Callable[..., Any]


def run_command(argv: Sequence[str], **kw: Any) -> subprocess.CompletedProcess:
    return subprocess.run(list(argv), capture_output=True, text=True, check=False, **kw)


# ---------------------------------------------------------------------------
# Power
# ---------------------------------------------------------------------------


def sample_ext5v(
    seconds: float,
    read: Callable[[], float | None] = sysinfo.read_ext5v,
    clock: Callable[[], float] = time.monotonic,
) -> list[tuple[float, float]]:
    """Read EXT5V as fast as vcgencmd answers for `seconds`; return (t, volts) pairs."""
    start = clock()
    samples: list[tuple[float, float]] = []
    while (t := clock() - start) < seconds:
        volts = read()
        if volts is None:
            break
        samples.append((t, volts))
    return samples


def ext5v_row(samples: Sequence[tuple[float, float]], seconds: float) -> Check:
    if not samples:
        return Check('power', 'ext5v', SKIP, 'vcgencmd unavailable')
    volts = [v for _, v in samples]
    low = min(volts)
    under = sum(1 for v in volts if v < sysinfo.EXT5V_MIN_VOLTS)
    status = {'healthy': OK, 'stale': WARN, 'dead': FAIL}[sysinfo.classify_ext5v(low)]
    detail = (
        f'min {low:.3f} V, mean {statistics.fmean(volts):.3f} V, '
        f'{100 * under / len(volts):.1f}% under {sysinfo.EXT5V_MIN_VOLTS:.2f} V '
        f'({len(volts)} samples in {seconds:.0f} s)'
    )
    return Check('power', 'ext5v', status, detail)


def throttle_row(before: int | None, after: int | None) -> Check:
    """Live bits fail; sticky bits set during the run fail; sticky bits from earlier warn."""
    if after is None:
        return Check('power', 'throttling', SKIP, 'vcgencmd unavailable')
    names = sysinfo.decode_throttled(f'throttled=0x{after:x}')[1]
    new = after & ~(before or 0) & STICKY_MASK
    if after & LIVE_MASK:
        return Check('power', 'throttling', FAIL, f'now: {", ".join(names)}')
    if new:
        return Check('power', 'throttling', FAIL, f'during the run: {", ".join(names)}')
    if after & STICKY_MASK:
        return Check('power', 'throttling', WARN, f'earlier this boot: {", ".join(names)}')
    return Check('power', 'throttling', OK, 'none')


def alarm_row(tripped: bool | None) -> Check:
    if tripped is None:
        return Check('power', 'pmic alarm', SKIP, 'no rpi_volt hwmon')
    if tripped:
        return Check('power', 'pmic alarm', WARN, 'tripped this boot')
    return Check('power', 'pmic alarm', OK, 'clear')


def arm_row(current: int | None, maximum: int | None) -> Check:
    if not current or not maximum:
        return Check('power', 'arm clock', SKIP, 'unreadable')
    return Check('power', 'arm clock', OK, f'{current} of {maximum} MHz')


def battery_row(samples: Sequence[tuple[float, float]], floor_cell_volts: float) -> Check:
    """Check the battery against the guard's floor; `samples` are (volts, amps drawn)."""
    from racecar_neo_ros2_driver import battery

    if not samples:
        return Check('power', 'battery', SKIP, 'no /battery/state; start teleop')
    volts = [v for v, _ in samples]
    cells = battery.detect_cells(max(volts))
    detail = (
        f'min {min(volts):.2f} V, mean {statistics.fmean(volts):.2f} V, '
        f'{statistics.fmean(a for _, a in samples):.2f} A'
    )
    if cells is None:
        return Check('power', 'battery', WARN, f'{detail}; cell count unknown')
    floor = floor_cell_volts * cells
    status = FAIL if min(volts) < floor else OK
    return Check('power', 'battery', status, f'{detail}; floor {floor:.2f} V ({cells}S)')


def floor_cell_volts() -> float:
    """Return the guard's floor per cell: config/battery.yaml with the car profile on top."""
    from racecar_neo_ros2_driver import car_profile
    from racecar_neo_ros2_driver.battery import BatteryConfig

    params = car_profile.node_params('battery', 'battery_guard', CONFIG_DIR)
    return BatteryConfig.from_params(params).floor_cell_volts


class BatterySampler:
    """Collects /battery/state (1 Hz means from pit_node) on a thread while a step runs."""

    def __init__(self) -> None:
        self.samples: list[tuple[float, float]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            import rclpy
            from sensor_msgs.msg import BatteryState
        except ImportError:
            return
        rclpy.init()
        node = rclpy.create_node('racecar_health')

        def record(msg: Any) -> None:
            # ROS counts discharge current as negative.
            self.samples.append((float(msg.voltage), -float(msg.current)))

        node.create_subscription(BatteryState, '/battery/state', record, 10)
        try:
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def stop(self) -> list[tuple[float, float]]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        return self.samples


def run_power(seconds: float, out: Callable[[str], None]) -> tuple[list[Check], dict[str, Any]]:
    sampler = BatterySampler()
    sampler.start()
    before, _ = sysinfo.read_throttled()
    out(f'Sampling the 5 V input for {seconds:.0f} s.')
    samples = sample_ext5v(seconds)
    after, _ = sysinfo.read_throttled()
    battery = sampler.stop()
    checks = [
        ext5v_row(samples, seconds),
        throttle_row(before, after),
        alarm_row(sysinfo.read_under_voltage_alarm()),
        arm_row(*sysinfo.read_arm_clock()),
        battery_row(battery, floor_cell_volts()),
    ]
    rails = sysinfo.read_pmic_rails()
    return checks, {'ext5v_samples': samples, 'rails': rails}


# ---------------------------------------------------------------------------
# NVMe
# ---------------------------------------------------------------------------


def _open_direct(path: Path, flags: int) -> int:
    """Open with O_DIRECT so the page cache does not stand in for the drive."""
    try:
        return os.open(path, flags | getattr(os, 'O_DIRECT', 0), 0o600)
    except OSError:
        # tmpfs and some others refuse O_DIRECT.
        return os.open(path, flags, 0o600)


def speed_test(
    folder: Path, size_mb: int = SPEED_TEST_MB, chunk_mb: int = SPEED_CHUNK_MB
) -> tuple[float, float]:
    """
    Write, read back and remove a test file; return (write MB/s, read MB/s).

    Each chunk carries its index, so a chunk read from the wrong place does
    not match. Raises OSError on an I/O failure, ValueError on a mismatch.
    """
    chunk = chunk_mb << 20
    count = max(1, size_mb // chunk_mb)
    # mmap buffers are page aligned, which O_DIRECT needs.
    wbuf = mmap.mmap(-1, chunk)
    wbuf.write(os.urandom(chunk))
    rbuf = mmap.mmap(-1, chunk)
    path = folder / SPEED_FILE
    try:
        fd = _open_direct(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        start = time.monotonic()
        try:
            for i in range(count):
                wbuf[:8] = i.to_bytes(8, 'little')
                if os.write(fd, wbuf) != chunk:
                    raise OSError('short write')
            os.fsync(fd)
        finally:
            os.close(fd)
        write_sec = time.monotonic() - start

        fd = _open_direct(path, os.O_RDONLY)
        read_sec = 0.0
        try:
            for i in range(count):
                start = time.monotonic()
                got = os.readv(fd, [rbuf])
                read_sec += time.monotonic() - start
                wbuf[:8] = i.to_bytes(8, 'little')
                if got != chunk or rbuf[:] != wbuf[:]:
                    raise ValueError(f'read back differs at {i * chunk_mb} MB')
        finally:
            os.close(fd)
    finally:
        path.unlink(missing_ok=True)
    total = count * chunk_mb
    return total / max(write_sec, 1e-6), total / max(read_sec, 1e-6)


def speed_row(folder: Path | None, test: Callable[[Path], tuple[float, float]]) -> Check:
    if folder is None:
        return Check('nvme', 'speed', SKIP, 'data root is not on the drive')
    try:
        write, read = test(folder)
    except (OSError, ValueError) as exc:
        return Check('nvme', 'speed', FAIL, f'{SPEED_TEST_MB} MB test: {exc}')
    slow = write < SPEED_WARN_WRITE_MBS or read < SPEED_WARN_READ_MBS
    detail = f'write {write:.0f} MB/s, read {read:.0f} MB/s ({SPEED_TEST_MB} MB, read back)'
    return Check('nvme', 'speed', WARN if slow else OK, detail)


def free_row(root: Path) -> Check:
    disk = sysinfo.read_disk(str(root)) if root.is_dir() else None
    if disk is None:
        return Check('nvme', 'free space', FAIL, f'{root} missing')
    low = disk['percent_used'] >= DATA_WARN_PCT or disk['free'] < DATA_WARN_FREE_GB
    detail = f'{disk["free"]}G free of {disk["total"]}G ({disk["percent_used"]}% used)'
    return Check('nvme', 'free space', WARN if low else OK, detail)


def unsafe_row(smart: dict[str, Any] | None, last: dict[str, Any] | None) -> Check:
    """
    Report unsafe shutdowns: power lost with the Pi running.

    A count only, never a warning: turning the car off at the switch is one.
    """
    if smart is None:
        return Check('nvme', 'unsafe shutdowns', SKIP, 'SMART unreadable')
    total = int(smart.get('unsafe_shutdowns', 0))
    if not last or 'unsafe_shutdowns' not in last.get('smart', {}):
        return Check('nvme', 'unsafe shutdowns', OK, f'{total} total; no earlier report')
    new = total - int(last['smart']['unsafe_shutdowns'])
    detail = f'{total} total, {new} new since {last["time"]}'
    return Check('nvme', 'unsafe shutdowns', OK, detail)


def kernel_row(text: str | None) -> Check:
    """Check this boot's kernel log for NVMe and PCIe errors."""
    if text is None:
        return Check('nvme', 'kernel log', SKIP, 'journalctl unavailable')
    nvme = [ln for ln in text.splitlines() if NVME_ERROR.search(ln)]
    pcie = [ln for ln in text.splitlines() if PCIE_ERROR.search(ln)]
    if not nvme and not pcie:
        return Check('nvme', 'kernel log', OK, 'no NVMe or PCIe errors this boot')
    first = (nvme or pcie)[0].strip()
    detail = f'{len(nvme)} NVMe, {len(pcie)} PCIe error line(s) this boot; first: {first}'
    return Check('nvme', 'kernel log', FAIL, detail)


def read_kernel_log(run: Run = run_command) -> str | None:
    r = run(['journalctl', '-k', '-b', '--no-pager', '-o', 'cat'])
    return r.stdout if r.returncode == 0 else None


def run_nvme(
    last: dict[str, Any] | None,
    run: Run = run_command,
    test: Callable[[Path], tuple[float, float]] = speed_test,
) -> tuple[list[Check], dict[str, Any]]:
    from racecar_neo_ros2_driver import data_root

    if not sysinfo.NVME_SYS.exists():
        return [Check('nvme', 'drive', FAIL, 'no NVMe drive on PCIe')], {}
    model = ''
    try:
        model = (sysinfo.NVME_SYS / 'model').read_text().strip()
    except OSError:
        pass
    checks = [Check('nvme', 'drive', OK, model or 'present')]
    mounted = os.path.ismount(data_root.NVME_MOUNT)
    checks.append(
        Check(
            'nvme',
            'mount',
            OK if mounted else FAIL,
            f'{data_root.NVME_MOUNT}' if mounted else f'not mounted at {data_root.NVME_MOUNT}',
        )
    )
    root, warning = data_root.current()
    on_drive = data_root.on_nvme(root) and not warning
    checks.append(
        Check(
            'nvme',
            'data root',
            OK if on_drive else FAIL,
            str(root) if on_drive else f'{root} on the SD card with a drive fitted',
        )
    )
    checks.append(speed_row(root if on_drive and root.is_dir() else None, test))
    checks.append(free_row(root))
    smart = sysinfo.read_nvme_smart()
    health = nvme_check(True, sysinfo.read_nvme_temp(), smart)
    checks.append(Check('nvme', 'smart', health.status, health.detail))
    checks.append(unsafe_row(smart, last))
    checks.append(kernel_row(read_kernel_log(run)))
    return checks, {'smart': smart or {}}


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def last_report(folder: Path, key: str) -> dict[str, Any] | None:
    """Return the newest report.json under `folder` that has `key`, or None."""
    for path in sorted(folder.glob('*/report.json'), reverse=True):
        try:
            report: dict[str, Any] = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if report.get(key):
            return report
    return None


def render_lines(checks: Sequence[Check], color: bool = False) -> list[str]:
    width = max((len(c.name) for c in checks), default=10)
    lines: list[str] = []
    for step in STEPS:
        rows = [c for c in checks if c.group == step]
        if not rows:
            continue
        lines += ['', step.upper()]
        for c in rows:
            mark = paint(MARK[c.status], c.status, color)
            lines.append(f'  {mark}  {c.name:<{width}}  {c.detail}')
    return lines


def report_markdown(stamp: str, car: str, checks: Sequence[Check], result: str) -> str:
    lines = [f'# Health report {stamp}', '', f'Car: {car}', f'Result: {result}', '']
    titles = {'power': 'Power', 'nvme': 'Data drive'}
    for step in STEPS:
        rows = [c for c in checks if c.group == step]
        if not rows:
            continue
        lines += [f'## {titles[step]}', '', '| Check | Result | Detail |', '| --- | --- | --- |']
        lines += [f'| {c.name} | {c.status} | {c.detail} |' for c in rows]
        lines.append('')
    return '\n'.join(lines)


def write_report(
    folder: Path,
    stamp: str,
    car: str,
    checks: Sequence[Check],
    extra: dict[str, Any],
) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    result = overall(list(checks))
    samples = extra.pop('ext5v_samples', [])
    report = {
        'time': stamp,
        'car': car,
        'result': result,
        'checks': [asdict(c) for c in checks],
        **extra,
    }
    (folder / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (folder / 'report.md').write_text(report_markdown(stamp, car, checks, result))
    with open(folder / 'samples.csv', 'w') as f:
        f.write('time_s,ext5v_v\n')
        f.writelines(f'{t:.4f},{v:.5f}\n' for t, v in samples)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='racecar health', description='Hardware checks over time, with a saved report.'
    )
    parser.add_argument('step', nargs='?', default='all', choices=('all', *STEPS))
    parser.add_argument(
        '--seconds', type=float, default=POWER_SEC, help='power sample time (default 30)'
    )
    args = parser.parse_args(argv)

    from racecar_neo_ros2_driver import car_profile, data_root

    steps = STEPS if args.step == 'all' else (args.step,)
    health_dir = data_root.data_dir('health')
    now = datetime.now()
    stamp = now.strftime('%Y-%m-%d %H:%M:%S')
    checks: list[Check] = []
    extra: dict[str, Any] = {}
    if 'power' in steps:
        rows, data = run_power(args.seconds, print)
        checks += rows
        extra.update(data)
    if 'nvme' in steps:
        print('Checking the data drive.')
        rows, data = run_nvme(last_report(health_dir, 'smart'))
        checks += rows
        extra.update(data)

    folder = health_dir / now.strftime('%Y%m%d_%H%M%S')
    write_report(folder, stamp, car_profile.car_info()['name'], checks, extra)
    color = use_color()
    for line in render_lines(checks, color):
        print(line)
    result = overall(checks)
    print(f'\n  RESULT {paint(result, result, color, bold=True)}   report: {folder}/report.md')
    return 1 if any(c.status == FAIL for c in checks) else 0


if __name__ == '__main__':
    sys.exit(main())
