#!/usr/bin/env python3
"""
Judge the phase 2 odometry runs from bags, against the autonomy plan's criteria.

Record a run with only the topics it needs, for example:

    racecar log start circle --topics /odom/integrated /odom /motor
    ... drive ...
    racecar log stop

then analyze it (no bag argument: the newest bag):

    analyze_odom.py circle [BAG]                full circle back to a floor mark
    analyze_odom.py straight [BAG] --tape-m 3   taped straight run, either way
    analyze_odom.py turn [BAG] --radius-m 0.74  laps at fixed steering, taped radius
    analyze_odom.py cpu --baseline 52.5         live pit_node CPU, not from a bag

The run is the span from the first to the last sample with the wheels turning
(`/odom/integrated` speed not 0). Each command prints its criteria (PASS,
WARN, FAIL, INFO) like `racecar calibrate check` and exits 1 on a FAIL.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'calibration'))

from calibrate_common import criteria_lines, Criterion, overall, use_color  # noqa: E402, I100
from racecar_log import load_config, resolve_log_root  # noqa: E402

# Acceptance, from docs/specifics/autonomy-service-plan.md, "Odometry".
CIRCLE_YAW_TOLERANCE_DEG = 5.0
STRAIGHT_TOLERANCE = 0.05
TURN_BIAS_LIMIT = 0.03  # above this the odometry needs a steering-indexed correction
CPU_ADDED_LIMIT = 10.0  # percent of one core added to pit_node

TOPICS = ('/odom/integrated', '/motor')


@dataclass
class Run:
    """Samples of `/odom/integrated` (and `/motor` steering), times in s."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray  # unwrapped, rad
    v: np.ndarray
    omega: np.ndarray
    steer_t: np.ndarray
    steer: np.ndarray

    def window(self) -> Run:
        """Return the span from the first to the last sample with the wheels turning."""
        moving = np.flatnonzero(self.v != 0.0)
        if moving.size == 0:
            raise ValueError('the wheels never turned in this bag')
        lo, hi = moving[0], moving[-1] + 1
        t0, t1 = self.t[lo], self.t[hi - 1]
        keep = (self.steer_t >= t0) & (self.steer_t <= t1)
        return Run(
            self.t[lo:hi],
            self.x[lo:hi],
            self.y[lo:hi],
            self.theta[lo:hi],
            self.v[lo:hi],
            self.omega[lo:hi],
            self.steer_t[keep],
            self.steer[keep],
        )

    def path_length(self) -> float:
        return float(np.sum(np.hypot(np.diff(self.x), np.diff(self.y))))


def run_from_arrays(
    t: list[float],
    x: list[float],
    y: list[float],
    yaw: list[float],
    v: list[float],
    omega: list[float],
    steer_t: list[float] | None = None,
    steer: list[float] | None = None,
) -> Run:
    return Run(
        np.asarray(t, float),
        np.asarray(x, float),
        np.asarray(y, float),
        np.unwrap(np.asarray(yaw, float)),
        np.asarray(v, float),
        np.asarray(omega, float),
        np.asarray(steer_t or [], float),
        np.asarray(steer or [], float),
    )


def read_bag(bag: Path) -> Run:
    """Read `/odom/integrated` and `/motor` from a bag (any storage the bag declares)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = rosbag2_py.Info().read_metadata(str(bag), '')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    missing = [topic for topic in TOPICS[:1] if topic not in types]
    if missing:
        raise ValueError(f'{bag} has no {", ".join(missing)}; record it with racecar log')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    classes = {name: get_message(types[name]) for name in TOPICS if name in types}

    cols: dict[str, list[float]] = {k: [] for k in ('t', 'x', 'y', 'yaw', 'v', 'w', 'st', 's')}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == '/odom/integrated':
            q = msg.pose.pose.orientation
            cols['t'].append(stamp)
            cols['x'].append(msg.pose.pose.position.x)
            cols['y'].append(msg.pose.pose.position.y)
            cols['yaw'].append(2.0 * math.atan2(q.z, q.w))
            cols['v'].append(msg.twist.twist.linear.x)
            cols['w'].append(msg.twist.twist.angular.z)
        else:
            cols['st'].append(stamp)
            cols['s'].append(msg.drive.steering_angle)
    order = np.argsort(cols['t'])
    pick = lambda k: [cols[k][i] for i in order]  # noqa: E731
    return run_from_arrays(
        pick('t'), pick('x'), pick('y'), pick('yaw'), pick('v'), pick('w'), cols['st'], cols['s']
    )


# ---------------------------------------------------------------------------
# Run types
# ---------------------------------------------------------------------------


def circle_criteria(run: Run) -> list[Criterion]:
    """Judge one full circle back to a floor mark: the integrated yaw should be 360 deg."""
    run = run.window()
    turned = math.degrees(run.theta[-1] - run.theta[0])
    error = abs(turned) - 360.0
    closure = math.hypot(run.x[-1] - run.x[0], run.y[-1] - run.y[0])
    return [
        Criterion(
            'heading closure',
            f'{turned:+.1f} deg ({error:+.1f})',
            f'360 within {CIRCLE_YAW_TOLERANCE_DEG:g} deg',
            'PASS' if abs(error) <= CIRCLE_YAW_TOLERANCE_DEG else 'FAIL',
            'check the gyro scale (calibrate_odom spin) and that the car stopped on the mark',
        ),
        Criterion(
            'position closure',
            f'{closure * 1000:.0f} mm over {run.path_length():.2f} m',
            'same mark start and end',
            'INFO',
        ),
        Criterion('duration', f'{run.t[-1] - run.t[0]:.1f} s', '', 'INFO'),
    ]


def straight_criteria(run: Run, tape_m: float) -> list[Criterion]:
    """Judge a straight run over a taped distance, forward or reverse."""
    run = run.window()
    dx, dy = run.x[-1] - run.x[0], run.y[-1] - run.y[0]
    measured = math.hypot(dx, dy)
    direction = 'forward' if float(np.mean(run.v)) > 0 else 'reverse'
    error = measured / tape_m - 1.0
    heading = math.degrees(run.theta[-1] - run.theta[0])
    return [
        Criterion(
            f'distance, {direction}',
            f'{measured:.3f} m ({error:+.1%})',
            f'{tape_m:g} m within {STRAIGHT_TOLERANCE:.0%}',
            'PASS' if abs(error) <= STRAIGHT_TOLERANCE else 'FAIL',
            'encoder scale off: calibrate_odom sets odometry.speed_scale',
        ),
        Criterion(
            'heading change',
            f'{heading:+.2f} deg ({heading / max(measured, 1e-6):+.2f} deg/m)',
            'steering trim input',
            'INFO',
        ),
    ]


def turn_criteria(run: Run, radius_m: float) -> list[Criterion]:
    """
    Judge laps at fixed steering around a taped radius (rear axle midpoint).

    Laps come from the gyro (|heading change| / 360 deg), true distance is laps
    x 2 pi R, and the encoder distance is the integrated speed; their ratio is
    the encoder's over-read in turns.
    """
    run = run.window()
    laps = abs(run.theta[-1] - run.theta[0]) / (2.0 * math.pi)
    speeds = np.abs(run.v)
    encoder = float(np.sum(0.5 * (speeds[1:] + speeds[:-1]) * np.diff(run.t)))
    true = laps * 2.0 * math.pi * radius_m
    bias = encoder / true - 1.0 if true > 0 else float('nan')
    kappa = float(np.mean(run.omega) / np.mean(run.v)) if np.mean(run.v) else float('nan')
    side = 'left' if np.mean(run.omega) > 0 else 'right'
    out = [
        Criterion(
            f'turning bias, {side}',
            f'{bias:+.1%} over {laps:.2f} laps',
            f'under {TURN_BIAS_LIMIT:.0%} needs no correction',
            'PASS' if abs(bias) <= TURN_BIAS_LIMIT else 'WARN',
            'above 3%: the odometry needs the steering-indexed speed correction',
        ),
        Criterion(
            'curvature',
            f'{kappa:+.3f} 1/m (radius {1 / abs(kappa):.3f} m)' if kappa else 'n/a',
            f'taped radius {radius_m:g} m',
            'INFO',
        ),
    ]
    if run.steer.size:
        out.append(Criterion('steering command', f'{float(np.mean(run.steer)):+.3f}', '', 'INFO'))
    return out


# ---------------------------------------------------------------------------
# CPU (live)
# ---------------------------------------------------------------------------

PIT_NODE_PATH = '/lib/racecar_neo_ros2_driver/pit_node'


def pit_node_pid() -> int | None:
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                argv = (entry / 'cmdline').read_bytes().split(b'\0')
            except OSError:
                continue
            if len(argv) > 1 and argv[1].decode(errors='replace').endswith(PIT_NODE_PATH):
                return int(entry.name)
    return None


def cpu_seconds(pid: int) -> float:
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    ticks = int(fields[11]) + int(fields[12])  # utime + stime
    return ticks / os.sysconf('SC_CLK_TCK')


def cpu_criteria(percent: float, baseline: float | None) -> list[Criterion]:
    if baseline is None:
        return [Criterion('pit_node CPU', f'{percent:.1f}% of one core', 'no baseline', 'INFO')]
    added = percent - baseline
    return [
        Criterion(
            'odometry CPU',
            f'{percent:.1f}% ({added:+.1f} on {baseline:g})',
            f'under {CPU_ADDED_LIMIT:g}% added',
            'PASS' if added <= CPU_ADDED_LIMIT else 'FAIL',
            'measure the baseline on the same boot with odometry.enable: false',
        )
    ]


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------


def resolve_bag(arg: str | None) -> Path:
    root = resolve_log_root(load_config().get('DIR'))
    if arg:
        bag = Path(arg).expanduser()
        return bag if bag.exists() else root / arg
    bags = [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
    if not bags:
        raise FileNotFoundError(f'no bags in {root}')
    return max(bags, key=lambda p: p.stat().st_mtime)


def report(title: str, criteria: list[Criterion]) -> int:
    color = use_color()
    print(title)
    print('\n'.join(criteria_lines(criteria, color=color)))
    verdict = overall(criteria)
    print(f'Result: {verdict}')
    return 1 if verdict == 'FAIL' else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('circle', help='full circle back to a floor mark').add_argument(
        'bag', nargs='?'
    )
    straight = sub.add_parser('straight', help='taped straight run')
    straight.add_argument('bag', nargs='?')
    straight.add_argument('--tape-m', type=float, required=True, help='taped distance, m')
    turn = sub.add_parser('turn', help='laps at fixed steering around a taped radius')
    turn.add_argument('bag', nargs='?')
    turn.add_argument('--radius-m', type=float, required=True, help='rear-axle circle radius, m')
    cpu = sub.add_parser('cpu', help='sample pit_node CPU live')
    cpu.add_argument('--seconds', type=float, default=30.0)
    cpu.add_argument('--baseline', type=float, help='pit_node %% without odometry, same boot')
    args = parser.parse_args(argv)

    if args.cmd == 'cpu':
        pid = pit_node_pid()
        if pid is None:
            print('pit_node is not running', file=sys.stderr)
            return 1
        start = cpu_seconds(pid)
        time.sleep(args.seconds)
        percent = (cpu_seconds(pid) - start) / args.seconds * 100.0
        return report(
            f'pit_node (pid {pid}) over {args.seconds:g} s', cpu_criteria(percent, args.baseline)
        )

    try:
        bag = resolve_bag(args.bag)
        run = read_bag(bag)
        if args.cmd == 'circle':
            criteria = circle_criteria(run)
        elif args.cmd == 'straight':
            criteria = straight_criteria(run, args.tape_m)
        else:
            criteria = turn_criteria(run, args.radius_m)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1
    return report(f'{args.cmd}: {bag}', criteria)


if __name__ == '__main__':
    sys.exit(main())
