#!/usr/bin/env python3
"""
Measurements for SLAM runs on the car, alongside `racecar launch autonomy`.

    slam_eval.py save NAME                 # write the mapping run to <map root>/NAME/
    slam_eval.py load [--seconds N]        # CPU per node, temperature, throttling; Ctrl-C ends
    slam_eval.py mark LABEL [--tag T]      # pose in the map frame with the car stopped on a mark
    slam_eval.py report [--taped FILE]     # mark repeatability and distances against a tape
    slam_eval.py quality MAP               # occupied, free, and unknown cells of a saved map

Records go to ~/data/slam_eval/: one CSV per `load` run and marks.csv for
`mark`. A taped file has one line per pair of marks, `A B meters`.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Iterable
import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import sysinfo  # noqa: E402

EVAL_DIR = Path.home() / 'data' / 'slam_eval'
MARKS_FILE = 'marks.csv'
# Label -> substring of /proc/<pid>/cmdline. Python nodes run as python3, so
# match the executable path, not the process name.
PROCESSES = {
    'pit_node': '/pit_node',
    'slam_toolbox': 'slam_toolbox_node',
    'amcl': 'nav2_amcl/amcl',
    'map_server': 'nav2_map_server/map_server',
    'lifecycle_manager': 'nav2_lifecycle_manager/lifecycle_manager',
    'realsense': 'realsense2_camera_node',
    'imu_fusion': '/imu_fusion_node',
    'edgetpu': '/edgetpu_node',
    'lidar': 'sllidar_node',
    'dashboard': 'dashboard.py',
    'foxglove_bridge': 'foxglove_bridge',
}
CLK_TCK = os.sysconf('SC_CLK_TCK')
MARK_SECONDS = 2.0
# pgm values written by map_saver_cli in trinary mode.
PGM_OCCUPIED, PGM_FREE = 0, 254


# --- save ---------------------------------------------------------------------


def save(name: str) -> int:
    from racecar_neo_ros2_driver.maps import map_stem, valid_name

    if not valid_name(name):
        print(f'Map names use letters, digits, - and _: {name!r}', file=sys.stderr)
        return 2
    stem = map_stem(name)
    stem.parent.mkdir(parents=True, exist_ok=True)
    import rclpy
    from slam_toolbox.srv import SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node('slam_eval_save')
    try:
        client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5.0):
            print('slam_toolbox is not running: racecar launch autonomy', file=sys.stderr)
            return 1
        future = client.call_async(SerializePoseGraph.Request(filename=str(stem)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        result = future.result()
        if result is None or result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            print('slam_toolbox could not write the pose graph.', file=sys.stderr)
            return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    saver = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(stem),
        '--ros-args', '-p', 'map_subscribe_transient_local:=true',
    ]  # fmt: skip
    if subprocess.run(saver, check=False).returncode != 0:
        print('map_saver_cli failed; the pose graph is saved.', file=sys.stderr)
        return 1
    for path in sorted(stem.parent.glob(stem.name + '.*')):
        print(path)
    return 0


# --- load ---------------------------------------------------------------------


def process_ticks(stat_text: str) -> int:
    """Return utime + stime from /proc/<pid>/stat; the name field may contain spaces."""
    fields = stat_text.rsplit(')', 1)[1].split()
    return int(fields[11]) + int(fields[12])


def find_processes(cmdlines: dict[int, str]) -> dict[str, list[int]]:
    found: dict[str, list[int]] = defaultdict(list)
    for pid, cmd in cmdlines.items():
        for label, needle in PROCESSES.items():
            if needle in cmd:
                found[label].append(pid)
                break
    return dict(found)


def read_cmdlines() -> dict[int, str]:
    out = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except OSError:
            continue
        if raw:
            out[int(entry.name)] = raw.replace(b'\0', b' ').decode(errors='replace')
    return out


def read_ticks(pids: Iterable[int]) -> dict[int, int]:
    """Return CPU ticks per live pid; a pid that has exited is left out."""
    ticks = {}
    for pid in pids:
        try:
            ticks[pid] = process_ticks(Path(f'/proc/{pid}/stat').read_text())
        except (OSError, IndexError, ValueError):
            continue
    return ticks


def tick_delta(before: dict[int, int], after: dict[int, int]) -> int:
    """Ticks used between two readings by pids alive at both."""
    return sum(max(after[pid] - before[pid], 0) for pid in after.keys() & before.keys())


@dataclass
class Sample:
    t: float
    cpu_busy: float  # percent of all cores
    temp_c: float | None
    throttled: int | None
    arm_mhz: int | None
    per_process: dict[str, float]  # percent of one core


def summarize(samples: list[Sample]) -> list[str]:
    if not samples:
        return ['No samples.']
    lines = [f'{len(samples)} samples over {samples[-1].t - samples[0].t:.0f} s']
    labels = sorted({k for s in samples for k in s.per_process})
    lines.append('Process            mean %   max %  (one core = 100)')
    for label in labels:
        values = [s.per_process.get(label, 0.0) for s in samples]
        lines.append(f'{label:18s} {sum(values) / len(values):6.1f}  {max(values):6.1f}')
    busy = [s.cpu_busy for s in samples]
    lines.append(f'All cores busy     {sum(busy) / len(busy):6.1f}  {max(busy):6.1f}')
    temps = [s.temp_c for s in samples if s.temp_c is not None]
    if temps:
        lines.append(f'SoC temperature    {sum(temps) / len(temps):6.1f}  {max(temps):6.1f} C')
    flags = {s.throttled for s in samples if s.throttled}
    clocks = [s.arm_mhz for s in samples if s.arm_mhz]
    if clocks:
        lines.append(f'ARM clock          min {min(clocks)} MHz')
    lines.append(
        'Throttled: ' + ', '.join(f'0x{f:x}' for f in sorted(flags)) + '. Discard this run.'
        if flags
        else 'Throttled: 0x0 for the whole run.'
    )
    return lines


def record_load(seconds: float | None, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f'{datetime.now():%Y%m%d-%H%M%S}-load.csv'
    procs = find_processes(read_cmdlines())
    print(
        'Recording ' + ', '.join(sorted(procs)) + f' to {path}; nodes started later are '
        'added. Ctrl-C ends.'
    )
    samples: list[Sample] = []
    last_ticks = {label: read_ticks(pids) for label, pids in procs.items()}
    last_cpu = sysinfo.read_cpu_times()
    start = last_t = time.monotonic()
    try:
        while seconds is None or time.monotonic() - start < seconds:
            time.sleep(1.0)
            now = time.monotonic()
            dt = now - last_t
            per = {}
            for label, pids in find_processes(read_cmdlines()).items():
                ticks = read_ticks(pids)
                if label in last_ticks:
                    per[label] = tick_delta(last_ticks[label], ticks) / CLK_TCK / dt * 100.0
                last_ticks[label] = ticks
            cpu = sysinfo.read_cpu_times()
            busy = 0.0
            if cpu and last_cpu and cpu[1] > last_cpu[1]:
                busy = (cpu[0] - last_cpu[0]) / (cpu[1] - last_cpu[1]) * 100.0
            last_cpu, last_t = cpu, now
            samples.append(
                Sample(
                    now - start,
                    busy,
                    sysinfo.read_soc_temp(),
                    sysinfo.read_throttled()[0],
                    sysinfo.read_arm_clock()[0],
                    per,
                )
            )
    except KeyboardInterrupt:
        pass
    labels = sorted({k for s in samples for k in s.per_process})
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'cpu_busy_pct', 'temp_c', 'throttled', 'arm_mhz', *labels])
        for s in samples:
            w.writerow(
                [f'{s.t:.1f}', f'{s.cpu_busy:.1f}', s.temp_c, s.throttled, s.arm_mhz]
                + [f'{s.per_process.get(k, 0.0):.1f}' for k in labels]
            )
    print('\n'.join(summarize(samples)))
    return 0


# --- mark and report ----------------------------------------------------------


@dataclass(frozen=True)
class Mark:
    label: str
    tag: str
    x: float
    y: float
    yaw: float


def mean_pose(poses: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """Mean of (x, y, yaw) samples; yaw by its unit vector."""
    n = len(poses)
    x = sum(p[0] for p in poses) / n
    y = sum(p[1] for p in poses) / n
    yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    return x, y, yaw


def mark(label: str, tag: str, out_dir: Path) -> int:
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node('slam_eval_mark')
    buffer = Buffer()
    TransformListener(buffer, node)
    poses: list[tuple[float, float, float]] = []
    end = time.monotonic() + MARK_SECONDS + 2.0
    try:
        while time.monotonic() < end and len(poses) < 40:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                t = buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append((t.transform.translation.x, t.transform.translation.y, yaw))
            time.sleep(MARK_SECONDS / 40)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not poses:
        print('No map -> base_link transform: is the localizer running?', file=sys.stderr)
        return 1
    x, y, yaw = mean_pose(poses)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / MARKS_FILE
    new = not path.exists()
    with path.open('a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['time', 'label', 'tag', 'x', 'y', 'yaw_deg', 'samples'])
        w.writerow(
            [f'{datetime.now():%Y-%m-%d %H:%M:%S}', label, tag, f'{x:.4f}', f'{y:.4f}',
             f'{math.degrees(yaw):.2f}', len(poses)]
        )  # fmt: skip
    print(f'{label} ({tag}): x {x:.3f} m, y {y:.3f} m, yaw {math.degrees(yaw):.1f} deg')
    return 0


def read_marks(path: Path) -> list[Mark]:
    with path.open() as f:
        return [
            Mark(
                r['label'],
                r['tag'],
                float(r['x']),
                float(r['y']),
                math.radians(float(r['yaw_deg'])),
            )
            for r in csv.DictReader(f)
        ]


def read_taped(path: Path) -> list[tuple[str, str, float]]:
    pairs = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 3 and not line.lstrip().startswith('#'):
            pairs.append((parts[0], parts[1], float(parts[2])))
    return pairs


def report_lines(marks: list[Mark], taped: list[tuple[str, str, float]]) -> list[str]:
    """Per tag: spread of repeated visits to each mark, then mark distances against the tape."""
    lines = []
    by_tag: dict[str, dict[str, list[Mark]]] = defaultdict(lambda: defaultdict(list))
    for m in marks:
        by_tag[m.tag][m.label].append(m)
    for tag in sorted(by_tag):
        lines.append(f'== {tag}')
        lines.append('Mark   visits  spread mm  yaw spread deg')
        means = {}
        for label in sorted(by_tag[tag]):
            visits = by_tag[tag][label]
            mx, my, myaw = mean_pose([(v.x, v.y, v.yaw) for v in visits])
            means[label] = (mx, my)
            spread = max(math.hypot(v.x - mx, v.y - my) for v in visits)
            yaw_spread = max(
                abs(math.degrees(math.atan2(math.sin(v.yaw - myaw), math.cos(v.yaw - myaw))))
                for v in visits
            )
            lines.append(f'{label:6s} {len(visits):6d}  {spread * 1e3:9.0f}  {yaw_spread:14.1f}')
        rows = [(a, b, d) for a, b, d in taped if a in means and b in means]
        if rows:
            lines.append('Pair      taped m  map m   error mm')
            for a, b, d in rows:
                got = math.dist(means[a], means[b])
                lines.append(f'{a + "-" + b:9s} {d:7.3f}  {got:6.3f}  {(got - d) * 1e3:+8.0f}')
    return lines


# --- quality ------------------------------------------------------------------


def cell_counts(pixels: Iterable[int]) -> tuple[int, int, int]:
    occupied = free = unknown = 0
    for p in pixels:
        if p == PGM_OCCUPIED:
            occupied += 1
        elif p == PGM_FREE:
            free += 1
        else:
            unknown += 1
    return occupied, free, unknown


def quality(name: str) -> int:
    from PIL import Image
    import yaml

    from racecar_neo_ros2_driver.maps import map_stem

    stem = map_stem(name)
    meta = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
    image = Image.open(stem.parent / meta['image'])
    occupied, free, unknown = cell_counts(image.getdata())
    res = float(meta['resolution'])
    print(f'{stem}: {image.width} x {image.height} cells at {res * 1e3:.0f} mm')
    print(f'occupied {occupied}  free {free}  unknown {unknown}')
    area = free * res * res
    print(
        f'free area {area:.2f} m^2; occupied cells per free m^2 {occupied / max(area, 1e-9):.1f}'
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('save', help='save the mapping run under the map root')
    p.add_argument('name')
    p = sub.add_parser('load', help='record CPU, temperature, and throttling')
    p.add_argument('--seconds', type=float)
    p = sub.add_parser('mark', help='record the pose at a mark on the floor')
    p.add_argument('label')
    p.add_argument('--tag', default='run', help='groups marks, e.g. amcl or slam_toolbox')
    p = sub.add_parser('report', help='mark repeatability and taped distances')
    p.add_argument('--marks', type=Path, default=EVAL_DIR / MARKS_FILE)
    p.add_argument('--taped', type=Path)
    p = sub.add_parser('quality', help='cell counts of a saved map')
    p.add_argument('map')
    args = parser.parse_args(argv)

    if args.cmd == 'save':
        return save(args.name)
    if args.cmd == 'load':
        return record_load(args.seconds, EVAL_DIR)
    if args.cmd == 'mark':
        return mark(args.label, args.tag, EVAL_DIR)
    if args.cmd == 'report':
        taped = read_taped(args.taped) if args.taped else []
        print('\n'.join(report_lines(read_marks(args.marks), taped)))
        return 0
    return quality(args.map)


if __name__ == '__main__':
    sys.exit(main())
