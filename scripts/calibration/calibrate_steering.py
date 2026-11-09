#!/usr/bin/env python3
"""
Steering trim and command-to-curvature table, referenced to the lidar.

The `steering` step of `racecar calibrate`. The car drives itself while the
operator holds RB, one pair of legs per steering command: with the wheels set
and the car still, it drives forward at 0.25 and stops, then backs up about
the same distance at the same steering, so it ends near where it started.
Straight first, then every left command in rising amplitude, then every right
one, then a second set at 0.5 that checks the table at speed. Every run is
driven first; between runs the operator may drive the car back to its mark
with the controller (LB). Then every run is analyzed, and the runs that could
not be measured are offered to drive again. Lidar scans at rest before and
after each leg give its chord and heading change; curvature = 2 sin(heading
/ 2) / chord per meter traveled, positive to the left in both directions. No
encoder distance is used.

The forward legs give the table written to the car profile's
params/steering_table.yaml (racecar_neo_ros2_driver/steering.py reads it);
the reverse legs are written as a second table only when they differ from
forward by more than REVERSE_MATCH.

The trim is the drift at command 0 divided by the slope near center (from
+/-0.25; at +/-0.5 the curvature already flattens toward full lock and
differs by side). throttle_node adds steering_trim to every command, so the
runs measure with the trim in use and the new trim is the old one plus that
command. A change smaller than the servo resolves is not written. Written to
the car profile's params/throttle.yaml. The curved runs also report the encoder's
over-read in turns as a gain for odometry.turn_gain, which is not written.

--check drives the pairs at 0 and +/-0.5: heading drift at 0 within 1 deg/m
passes; up to 5 deg/m warns, naming servo and linkage play (up to 3 deg/m
between runs); beyond is a mechanical fault. The +/-0.5 legs must match the
stored table within TABLE_TOLERANCE. The trim throttle_node runs with must
match the file.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    Criterion,
    finish_check,
    node_params,
    read_line,
    write_local_yaml,
)
from calibrate_dynamics import (  # noqa: E402
    Bag,
    command_edges,
    Driver,
    laser_pose,
    read_bag,
    rest_scan,
    speed_tag,
    WHEELBASE_M,
)
from calibrate_odom import best_match, match_criterion, resolve_turns  # noqa: E402
from racecar_neo_ros2_driver import car_profile, steering  # noqa: E402
from scan_match import (  # noqa: E402
    base_motion,
    compose,
    invert,
    laser_motion,
    MatchResult,
)

CAL_NAME = 'throttle'
NODE = 'throttle_node'
TOOL = 'calibrate_steering.py'
DRIVE_SPEED = 0.25
# Denser near center and around 0.5, where the curvature bends toward full lock.
COMMANDS = (0.0, 0.125, -0.125, 0.25, -0.25, 0.375, -0.375, 0.5, -0.5, 0.75, -0.75, 1.0, -1.0)
# The speed check: the same pairs at this speed for these commands.
FAST_SPEED = 0.5
FAST_COMMANDS = (0.5, -0.5, 1.0, -1.0)
# Reverse is written as its own table past this difference from forward; the
# speed check reports past the same fraction.
REVERSE_MATCH = 0.10
SPEED_MATCH = 0.10
# Below this curvature (1/m) the comparison is absolute, not relative.
MATCH_FLOOR = 0.2
# --check: the +/-0.5 legs against the stored table.
TABLE_CHECK_COMMANDS = (0.0, 0.5, -0.5)
TABLE_TOLERANCE = 0.10
STEERING_TABLE = steering.NAME
# Commands used for the trim fit; full lock is past where curvature is linear.
# The trim comes from the drift at 0 and the slope from +/-CENTER_COMMAND: past
# that the curvature flattens toward full lock and differs by side.
CENTER_COMMAND = 0.25
# Smallest trim change the servo resolves (about 5 us of pulse); a measured
# trim of +0.019 changed the drift by under 0.1 deg/m on the reference car.
MIN_TRIM_STEP = 0.01
# Fit of the encoder over-read in turns, k x (1/cos(delta) - 1): only runs with
# enough steering to carry it.
TURN_FIT_MIN_X = 0.05
MAX_TRIM = 0.2
# Straight drift at a fixed trim varied from -2.7 to +0.4 deg/m between runs on
# the reference car (servo and linkage play). Drift past DRIFT_PASS still
# warns, naming that play; past DRIFT_WARN it is beyond play.
DRIFT_PASS, DRIFT_WARN = 1.0, 5.0  # deg/m
HARDWARE_PLAY_DEG_PER_M = 3.0
PARAM_TIMEOUT = 3.0

PLACE = 'Place the car in the middle of the clear area. Mark where it stands.'


def drive_seconds(command: float) -> float:
    # Long enough to measure, short enough that the turn stays under about
    # 100 deg so the two rest scans still overlap.
    return 3.0 if command == 0.0 else 2.0 if abs(command) <= 0.25 else 1.3


def segments(command: float) -> list[tuple[float, float, float]]:
    """Wheels set while still, one drive, stop; the steering never changes."""
    return [
        (0.0, command, 2.0),
        (DRIVE_SPEED, command, drive_seconds(command)),
        (0.0, command, 3.0),
    ]


# Backing up from rest, the car covers next to nothing for about this long
# (0.48 s at speed 0.25 on the reference car, firmware v0.9.2); the dynamics'
# reverse rise time can read far shorter.
REVERSE_START_SEC = 0.48


def reverse_seconds(forward_sec: float, speed: float, dynamics: dict[str, Any]) -> float:
    """
    Return how long to back up to cover the forward leg's distance.

    From the car's dynamics (steady speed and the time to 90% of it, each
    way), with at least REVERSE_START_SEC to get moving backward; the forward
    time when they are missing.
    """
    try:
        v_f = float(dynamics[f'steady_speed_mps_{speed_tag(speed)}'])
        v_r = abs(float(dynamics[f'steady_speed_mps_{speed_tag(-speed)}']))
        r_f = float(dynamics[f'rise_90_s_{speed_tag(speed)}'])
        r_r = float(dynamics[f'rise_90_s_{speed_tag(-speed)}'])
    except (KeyError, TypeError, ValueError):
        return forward_sec
    if v_f <= 0 or v_r <= 0:
        return forward_sec
    distance = v_f * max(forward_sec - r_f / 2.0, 0.0)
    return round(max(distance / v_r + max(r_r / 2.0, REVERSE_START_SEC), 0.5), 2)


def pair_segments(
    command: float, speed: float, dynamics: dict[str, Any], reverse_scale: float = 1.0
) -> list[tuple[float, float, float]]:
    """
    Wheels set while still, forward, stop, back up the same way, stop.

    `reverse_scale` stretches the reverse leg; the run learns it from the
    legs it has measured (next_reverse_scale).
    """
    forward = drive_seconds(command)
    back = round(reverse_seconds(forward, speed, dynamics) * reverse_scale, 2)
    return [
        (0.0, command, 2.0),
        (speed, command, forward),
        (0.0, command, 3.0),
        (-speed, command, back),
        (0.0, command, 3.0),
    ]


def run_name(command: float) -> str:
    """Return a bag-safe name: 0.25 -> 'l025', -0.125 -> 'r0125', 0 -> 'c0'."""
    if command == 0.0:
        return 'c0'
    tag = f'{abs(command):.3f}'.replace('.', '').rstrip('0').ljust(2, '0')
    return ('l' if command > 0 else 'r') + tag


def describe(command: float) -> str:
    side = 'straight' if command == 0.0 else 'left' if command > 0 else 'right'
    return f'steering {command:+g} ({side})'


# The learned reverse stretch stays within these bounds.
REVERSE_SCALE_LIMITS = (0.7, 2.0)


def next_reverse_scale(scale: float, forward: Arc, reverse: Arc) -> float:
    """Return the reverse stretch that would have made the last reverse leg match its forward."""
    if not forward.lidar_arc_m > 0 or not reverse.lidar_arc_m > 0:
        return scale
    lo, hi = REVERSE_SCALE_LIMITS
    return round(min(hi, max(lo, scale * forward.lidar_arc_m / reverse.lidar_arc_m)), 3)


def plan_order(commands: tuple[float, ...], speed: float) -> list[tuple[float, float]]:
    """Return straight first, then the left commands rising, then the right ones."""
    left = sorted(c for c in commands if c > 0)
    right = sorted((c for c in commands if c < 0), reverse=True)
    straight = [0.0] if 0.0 in commands else []
    return [(c, speed) for c in straight + left + right]


def parse_pair_name(name: str) -> tuple[float, float] | None:
    """Return (command, speed) from a bag folder name ending in pair_name(), or None."""
    import re

    m = re.search(r'steer_(c0|[lr]\d+)_s(\d+)$', name)
    if not m:
        return None

    def value(digits: str) -> float:
        return int(digits) / 10 ** (len(digits) - 1)

    run, tag = m.groups()
    command = 0.0 if run == 'c0' else value(run[1:]) * (1 if run[0] == 'l' else -1)
    return command, value(tag)


def pair_name(command: float, speed: float) -> str:
    """Return a bag-safe name for a pair: 'l025_s025', 'c0_s05'."""
    return f'{run_name(command)}_s{speed_tag(speed)}'


def arc_curvature(heading_rad: float, chord_m: float) -> float:
    """Return the curvature (1/m, + left) of an arc with this heading change and chord."""
    if chord_m <= 0.0:
        return math.nan
    return 2.0 * math.sin(heading_rad / 2.0) / chord_m


@dataclass(frozen=True)
class Arc:
    command: float
    curvature: float  # 1/m, + left
    heading_deg: float
    chord_m: float
    match: MatchResult
    odom_heading_deg: float = math.nan  # /odom/integrated over the same run
    odom_chord_m: float = math.nan
    encoder_m: float = math.nan  # /odom/integrated path length over the run
    lidar_arc_m: float = math.nan  # arc length from the lidar chord and heading
    speed: float = DRIVE_SPEED  # /drive speed of the leg; negative backs up
    motion: tuple[float, float, float] = (0.0, 0.0, 0.0)  # base_link start -> end


@dataclass(frozen=True)
class TrimFit:
    trim_delta: float  # command that drives straight, in the current trim's terms
    drift_deg_per_m: float  # curvature at command 0, deg/m
    gain_left: float  # curvature per unit command near center, 1/m
    gain_right: float


def fit_trim(arcs: list[Arc]) -> TrimFit | None:
    """
    Return the trim from the drift at 0 and the center slope; None without them.

    Needs the run at 0 and at least one of +/-CENTER_COMMAND.
    """
    by = {a.command: a.curvature for a in arcs}
    if 0.0 not in by:
        return None
    k0 = by[0.0]
    left = (by[CENTER_COMMAND] - k0) / CENTER_COMMAND if CENTER_COMMAND in by else math.nan
    right = (k0 - by[-CENTER_COMMAND]) / CENTER_COMMAND if -CENTER_COMMAND in by else math.nan
    slopes = [g for g in (left, right) if math.isfinite(g) and g > 0]
    if not slopes:
        return None
    return TrimFit(
        trim_delta=float(-k0 / np.mean(slopes)),
        drift_deg_per_m=math.degrees(k0),
        gain_left=float(left),
        gain_right=float(right),
    )


def measure_arc(bag: Bag, laser: Any, command: float) -> Arc | None:
    """
    Return the arc of one constant-steering run, from its two rest scans.

    `command` is the steering sent on /drive; /motor carries it with the trim added.
    """
    # The run's own drive is the last motion: the operator may have driven the
    # car into place with the gamepad while the recording waited for RB.
    edges = command_edges(bag)
    launch = next((e for e in reversed(edges) if e[1] != 0.0), None)
    if launch is None:
        return None
    end = float(bag.odom_t[-1])
    return measure_leg(bag, laser, command, launch[0], launch[1], (end - 2.0, end))


def measure_pair(bag: Bag, laser: Any, command: float) -> tuple[Arc | None, Arc | None]:
    """Return the forward and reverse arcs of a pair run (pair_segments)."""
    # The pair is the last motion in the bag (see measure_arc): the last
    # reverse launch, and the last forward launch before it.
    edges = command_edges(bag)
    starts = [
        e for prev, e in zip(edges, edges[1:], strict=False) if prev[1] == 0.0 and e[1] != 0.0
    ]
    rev = next((e for e in reversed(starts) if e[1] < 0.0), None)
    fwd = next((e for e in reversed(starts) if e[1] > 0.0 and rev and e[0] < rev[0]), None)
    if fwd is None or rev is None:
        return None, None
    end = float(bag.odom_t[-1])
    return (
        measure_leg(bag, laser, command, fwd[0], fwd[1], (rev[0] - 1.5, rev[0] - 0.1)),
        measure_leg(bag, laser, command, rev[0], rev[1], (end - 2.0, end)),
    )


def measure_leg(
    bag: Bag,
    laser: Any,
    command: float,
    launch_t: float,
    speed: float,
    end_window: tuple[float, float],
) -> Arc | None:
    """Return one leg's arc: rest scans just before launch_t and in end_window."""
    start = rest_scan(bag, launch_t - 1.5, launch_t - 0.1)
    end = rest_scan(bag, *end_window)
    if len(start) < 50 or len(end) < 50:
        return None
    i0 = int(np.searchsorted(bag.odom_t, launch_t - 0.1))
    i1 = min(int(np.searchsorted(bag.odom_t, end_window[1], side='right')), len(bag.odom_t)) - 1
    if i1 <= i0:
        return None
    a, b = tuple(bag.odom[i0]), tuple(bag.odom[i1])
    rel = compose(invert(a), b)
    heading = float(bag.odom[i1, 2] - bag.odom[i0, 2])
    guesses = [
        laser_motion((rel[0], rel[1], heading), laser),
        laser_motion((0, 0, heading), laser),
    ]
    match = best_match(start, end, guesses)
    motion = base_motion(match.pose, laser)
    true = resolve_turns(motion[2], heading)
    if true is None:
        return None
    chord = math.hypot(motion[0], motion[1])
    half = true / 2.0
    arc_m = chord * (half / math.sin(half)) if abs(half) > 1e-6 else chord
    # Per meter traveled: backing up turns the heading the other way.
    direction = 1.0 if speed > 0 else -1.0
    return Arc(
        command,
        direction * arc_curvature(true, chord),
        math.degrees(true),
        chord,
        match,
        math.degrees(heading),
        math.hypot(rel[0], rel[1]),
        float(bag.path_m[i1] - bag.path_m[i0]),
        abs(arc_m),
        speed,
        (float(motion[0]), float(motion[1]), float(true)),
    )


@dataclass(frozen=True)
class TurnFit:
    gain: float  # k for odometry.turn_gain
    straight_ratio: float  # encoder / lidar on the straight runs (the session's baseline)
    runs: int


def fit_turn_gain(arcs: list[Arc], current_gain: float, wheelbase: float) -> TurnFit | None:
    """
    Fit k in encoder / lidar = baseline (1 + k (1/cos(delta) - 1)) over the curved runs.

    The baseline is the straight runs of the same session, so a speed_scale
    error cancels. /odom/integrated already divides by the current gain's
    factor, which is put back first. None without a straight run and two
    curved runs that carry the fit.
    """
    straight = [
        a.encoder_m / a.lidar_arc_m for a in arcs if a.command == 0.0 and a.lidar_arc_m > 0
    ]
    if not straight:
        return None
    base = float(np.mean(straight))
    xs, ys = [], []
    for a in arcs:
        if a.command == 0.0 or not a.lidar_arc_m > 0:
            continue
        x = 1.0 / math.cos(math.atan(wheelbase * abs(a.curvature))) - 1.0
        if x < TURN_FIT_MIN_X:
            continue
        ratio = (a.encoder_m / a.lidar_arc_m) / base * (1.0 + current_gain * x)
        xs.append(x)
        ys.append(ratio - 1.0)
    if len(xs) < 2:
        return None
    xv, yv = np.array(xs), np.array(ys)
    return TurnFit(float(xv @ yv / (xv @ xv)), base, len(xs))


# The odometry heading is calibrated to well under 1%; a scan match further
# off than this has locked onto the wrong alignment.
MAX_HEADING_DISAGREEMENT_DEG = 10.0
# The encoder reads up to about 4% high in turns; past this the match is wrong.
MAX_CHORD_DISAGREEMENT = 0.15
# Backing up at tight steering the wheels slip at the start and the encoder
# read up to twice the lidar's distance, so a reverse leg's lidar chord may be
# this short a share of the odometry's (a wrong match in a forward leg once
# read a third).
MIN_REVERSE_CHORD_SHARE = 0.4


def arc_problem(arc: Arc) -> str:
    """Return why an arc's measurement cannot be trusted, or '' when it can."""
    gate = match_criterion('scan match', arc.match)
    if gate.status != 'PASS':
        return f'Scan match too poor ({gate.measured}).'
    gap = abs(arc.heading_deg - arc.odom_heading_deg)
    if gap > MAX_HEADING_DISAGREEMENT_DEG:
        return (
            f'The lidar measured {arc.heading_deg:+.1f} deg and the odometry '
            f'{arc.odom_heading_deg:+.1f} deg.'
        )
    ratio = arc.chord_m / arc.odom_chord_m
    low = MIN_REVERSE_CHORD_SHARE if arc.speed < 0 else 1.0 - MAX_CHORD_DISAGREEMENT
    if ratio > 1.0 + MAX_CHORD_DISAGREEMENT or ratio < low:
        return (
            f'The lidar measured a {arc.chord_m:.2f} m chord and the odometry '
            f'{arc.odom_chord_m:.2f} m.'
        )
    return ''


def drift_criterion(drift: float) -> Criterion:
    status = 'PASS' if abs(drift) <= DRIFT_PASS else 'WARN' if abs(drift) <= DRIFT_WARN else 'FAIL'
    return Criterion(
        'straight drift',
        f'{drift:+.2f} deg/m ({"left" if drift > 0 else "right"})',
        f'within {DRIFT_PASS} deg/m (warn to {DRIFT_WARN})',
        status,
        (
            f'servo and linkage play alone moves the drift by up to {HARDWARE_PLAY_DEG_PER_M:g} '
            'deg/m between runs; recalibrate only if it stays: racecar calibrate steering'
            if status == 'WARN'
            else 'beyond servo and linkage play: check the servo horn and the steering '
            'linkage, then racecar calibrate steering'
        ),
    )


def trim_criterion(file_trim: float, live: float | None) -> Criterion:
    if live is None:
        return Criterion(
            'trim in use', f'file {file_trim:+g}', 'throttle_node not reachable', 'INFO'
        )
    if not math.isclose(file_trim, live, abs_tol=1e-6):
        return Criterion(
            'trim in use',
            f'throttle_node {live:+g}; file {file_trim:+g}',
            'throttle_node matches the file',
            'FAIL',
            'restart teleop to load the file: racecar service restart teleop',
        )
    return Criterion('trim in use', f'{file_trim:+g}', '', 'INFO')


def table_lines(arcs: list[Arc]) -> list[str]:
    lines = ['Command  Curvature   Radius    Wheel angle (bicycle)']
    for a in sorted(arcs, key=lambda a: a.command):
        radius = 1.0 / a.curvature if a.curvature else math.inf
        angle = math.degrees(math.atan(WHEELBASE_M * a.curvature))
        lines.append(
            f'{a.command:+6.2f}  {a.curvature:+7.3f}/m  {radius:+7.3f} m  {angle:+6.1f} deg'
        )
    return lines


def differs(a: float, b: float, fraction: float) -> bool:
    """Return True when two curvatures differ by more than `fraction` (absolute near zero)."""
    return abs(a - b) > fraction * max(abs(a), abs(b), MATCH_FLOOR)


def compare_lines(
    title: str, base: dict[float, float], other: dict[float, float], fraction: float
) -> tuple[list[str], bool]:
    """Return lines comparing two {command: curvature} sets and whether any differs."""
    lines, any_off = [title], False
    for c in sorted(set(base) & set(other)):
        off = differs(base[c], other[c], fraction)
        any_off |= off
        lines.append(
            f'{c:+6.3f}  {base[c]:+7.3f}/m  {other[c]:+7.3f}/m'
            + (f'  differs by more than {fraction:.0%}' if off else '')
        )
    return lines, any_off


# At full lock the curvature is flat; a later command reading lower by up to
# this share of the flat value is measurement scatter and is pooled.
SATURATION_TOL = 0.08


def pool_flat(commands: list[float], curvature: list[float]) -> tuple[list[float], list[str], str]:
    """
    Make the curvature non-decreasing by pooling neighbors that fall.

    Returns the pooled curvature, one line per pooled stretch, and a problem
    when a fall is past SATURATION_TOL (the table is then not written).
    """
    blocks: list[list[float]] = []  # [sum, count, first index]
    for i, k in enumerate(curvature):
        blocks.append([k, 1, i])
        while len(blocks) > 1 and blocks[-2][0] / blocks[-2][1] > blocks[-1][0] / blocks[-1][1]:
            k2, n2, _ = blocks.pop()
            blocks[-1][0] += k2
            blocks[-1][1] += n2
    out: list[float] = []
    notes: list[str] = []
    for total, count, first in blocks:
        mean = total / count
        out += [round(mean, 4)] * int(count)
        if count == 1:
            continue
        idx = range(int(first), int(first) + int(count))
        spread = max(abs(curvature[j] - mean) for j in idx)
        names = ', '.join(f'{commands[j]:+g} ({curvature[j]:+.3f})' for j in idx)
        if spread > SATURATION_TOL * max(abs(mean), MATCH_FLOOR):
            return out, notes, f'curvature falls across {names} by more than {SATURATION_TOL:.0%}'
        notes.append(f'Flat at {mean:+.3f}/m (full lock): {names}')
    return out, notes, ''


def build_table(
    forward: dict[float, float],
    reverse: dict[float, float] | None,
    trim_used: float,
    trim_delta: float,
    measured: str,
) -> steering.SteeringTable:
    """
    Return the table in the terms of the trim that will be in use.

    A command c sent with trim t gives the wheels c + t; with the new trim
    t + d the same wheels need c - d.
    """
    cmds = sorted(forward)
    return steering.SteeringTable(
        tuple(round(c - trim_delta, 4) for c in cmds),
        tuple(forward[c] for c in cmds),
        tuple(reverse[c] for c in cmds) if reverse else None,
        DRIVE_SPEED,
        round(trim_used + trim_delta, 4),
        measured,
    )


def table_criteria(
    stored: steering.SteeringTable | None,
    results: dict[tuple[float, float], tuple[Arc, Arc]],
    trim: float,
) -> list[Criterion]:
    """Compare the measured legs at +/-0.5 with the stored table."""
    if stored is None:
        return [
            Criterion('steering table', 'not measured', '', 'FAIL', 'racecar calibrate steering')
        ]
    out = []
    for (command, _speed), arcs in sorted(results.items()):
        if command == 0.0:
            continue
        # The stored commands are in the terms of the trim it was written for.
        wheel = command + trim - stored.trim
        for arc, reverse in ((arcs[0], False), (arcs[1], True)):
            want = stored.curvature_for(wheel, reverse=reverse)
            off = differs(arc.curvature, want, TABLE_TOLERANCE)
            out.append(
                Criterion(
                    f'table {command:+g} {"reverse" if reverse else "forward"}',
                    f'{arc.curvature:+.3f}/m (table {want:+.3f}/m)',
                    f'within {TABLE_TOLERANCE:.0%}',
                    'FAIL' if off else 'PASS',
                    'the steering changed: racecar calibrate steering',
                )
            )
    return out


class SteeringCalibrator:
    def __init__(self, check: bool, report: str | None, bags: list[Path] | None = None) -> None:
        self.check = check
        self.report = report
        self.bags = bags or []
        self.trim = float(node_params(CAL_NAME, NODE).get('steering_trim', 0.0))
        self.driver = Driver()
        self.log = self.driver.log
        self.laser = laser_pose()
        self.dynamics = car_profile.read_yaml(car_profile.param_path('dynamics'))
        # Per run: how much longer the reverse leg must run to come back,
        # learned from that run's last analysis.
        self.reverse_scale: dict[tuple[float, float], float] = {}

    def drive_pairs(self, plan: list[tuple[float, float]]) -> dict[tuple[float, float], Path]:
        """Drive every (command, speed) run in order; return each run's bag."""
        log = self.log
        bags: dict[tuple[float, float], Path] = {}
        for i, key in enumerate(plan):
            command, speed = key
            while True:
                log.info(f'Run {i + 1} of {len(plan)}: {describe(command)}, speed {speed:g}')
                bag = self.driver.drive(
                    pair_name(command, speed),
                    pair_segments(command, speed, self.dynamics, self.reverse_scale.get(key, 1.0)),
                    prefix='steer',
                )
                if bag is not None:
                    bags[key] = bag
                    break
                log.info('The run stopped before the end. It runs again.')
        return bags

    def analyze(
        self, bags: dict[tuple[float, float], Path]
    ) -> tuple[dict[tuple[float, float], tuple[Arc, Arc]], list[tuple[float, float]]]:
        """Measure every run's two legs; return the measured runs and the failed ones."""
        log = self.log
        done: dict[tuple[float, float], tuple[Arc, Arc]] = {}
        failed: list[tuple[float, float]] = []
        log.info(f'Analyzing {len(bags)} runs...')
        for i, (key, path) in enumerate(bags.items()):
            command, speed = key
            label = f'{i + 1} of {len(bags)}, {describe(command)}, speed {speed:g}'
            try:
                fwd, rev = measure_pair(read_bag(path), self.laser, command)
            except ValueError as exc:
                log.error(f'{label}: {exc}')
                failed.append(key)
                continue
            problems = [
                f'{leg} leg: ' + (arc_problem(arc) if arc else 'the lidar could not measure it')
                for leg, arc in (('forward', fwd), ('reverse', rev))
                if arc is None or arc_problem(arc)
            ]
            if problems or fwd is None or rev is None:
                log.error(f'{label}: ' + ' '.join(problems))
                failed.append(key)
                continue
            self.reverse_scale[key] = next_reverse_scale(
                self.reverse_scale.get(key, 1.0), fwd, rev
            )
            slip = rev.chord_m / rev.odom_chord_m
            log.info(
                f'{label}: forward {fwd.curvature:+.3f}/m, reverse {rev.curvature:+.3f}/m'
                + (f' (wheels slipped backing up: {1 - slip:.0%})' if slip < 0.85 else '')
            )
            done[key] = (fwd, rev)
        return done, failed

    def recorded_pairs(self) -> dict[tuple[float, float], tuple[Arc, Arc]]:
        """Analyze the bags given with --bags (the newest per run); nothing is driven."""
        bags: dict[tuple[float, float], Path] = {}
        for path in sorted(self.bags):
            key = parse_pair_name(path.name)
            if key is None:
                self.log.error(f'{path.name}: not a steering run; skipped.')
                continue
            bags[key] = path
        results, failed = self.analyze(bags)
        if failed:
            self.log.info(
                'Not measured: ' + ', '.join(f'{c:+g} at {sp:g}' for c, sp in failed) + '.'
            )
        return results

    def run_pairs(
        self, plan: list[tuple[float, float]]
    ) -> dict[tuple[float, float], tuple[Arc, Arc]]:
        """
        Drive every run, then analyze them all, then offer the failed ones again.

        The operator may finish without the failed runs (s at the prompt).
        """
        log = self.log
        log.info(PLACE)
        log.info('Each run drives forward on an arc, then backs up along it.')
        log.info('Between runs, you can drive the car back to its mark with the controller (LB).')
        read_line('Press Enter when the car is in place. ')
        results, failed = self.analyze(self.drive_pairs(plan))
        while failed:
            names = ', '.join(f'{c:+g} at {sp:g}' for c, sp in failed)
            log.info(f'{len(failed)} runs need to be driven again: steering {names}.')
            answer = read_line('Press Enter to drive them, or type s to finish without them. ')
            if answer.strip().lower() == 's':
                break
            again, failed = self.analyze(self.drive_pairs(failed))
            results.update(again)
        return results

    def live_trim(self) -> float | None:
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters

        node = self.driver.node
        client = node.create_client(GetParameters, f'/{NODE}/get_parameters')
        if not client.wait_for_service(timeout_sec=PARAM_TIMEOUT):
            return None
        future = client.call_async(GetParameters.Request(names=['steering_trim']))
        self.driver.rclpy.spin_until_future_complete(node, future, timeout_sec=PARAM_TIMEOUT)
        result = future.result() if future.done() else None
        if not result or not result.values:
            return None
        value = result.values[0]
        return value.double_value if value.type == ParameterType.PARAMETER_DOUBLE else None

    def run(self) -> int:
        try:
            return self._check() if self.check else self._calibrate()
        finally:
            self.driver.close()

    def _calibrate(self) -> int:
        log = self.log
        plan = plan_order(COMMANDS, DRIVE_SPEED) + plan_order(FAST_COMMANDS, FAST_SPEED)
        results = self.recorded_pairs() if self.bags else self.run_pairs(plan)
        base = {c: arcs for (c, sp), arcs in results.items() if sp == DRIVE_SPEED}
        forward = {c: arcs[0].curvature for c, arcs in base.items()}
        reverse = {c: arcs[1].curvature for c, arcs in base.items()}
        for line in table_lines([arcs[0] for arcs in base.values()]):
            log.info(line)
        rev_lines, reverse_differs = compare_lines(
            'Command  Forward    Reverse', forward, reverse, REVERSE_MATCH
        )
        for line in rev_lines:
            log.info(line)
        fast = {
            (c, leg): arcs[leg].curvature
            for (c, sp), arcs in results.items()
            if sp == FAST_SPEED
            for leg in (0, 1)
        }
        for leg, name in ((0, 'forward'), (1, 'reverse')):
            at_speed = {c: k for (c, lg), k in fast.items() if lg == leg}
            slow = forward if leg == 0 else reverse
            speed_lines, speed_differs = compare_lines(
                f'Command  {DRIVE_SPEED:g}       {FAST_SPEED:g} ({name})',
                slow,
                at_speed,
                SPEED_MATCH,
            )
            for line in speed_lines:
                log.info(line)
            if speed_differs:
                log.warning(
                    f'Steering {name} at speed {FAST_SPEED:g} differs from {DRIVE_SPEED:g} by '
                    f'more than {SPEED_MATCH:.0%}. The table is for speed {DRIVE_SPEED:g}.'
                )

        fit = fit_trim([arcs[0] for arcs in base.values()])
        if fit is None:
            log.error(
                f'The trim needs the pair at steering 0 and one at +/-{CENTER_COMMAND:g}. '
                'Nothing written.'
            )
            return 1
        log.info(
            f'Drift at steering 0: {fit.drift_deg_per_m:+.2f} deg/m. Gain near center: left '
            f'{fit.gain_left:.2f}/m, right {fit.gain_right:.2f}/m per unit command.'
        )
        self._report_turn_gain([arcs[0] for arcs in base.values()])
        delta = fit.trim_delta if abs(fit.trim_delta) >= MIN_TRIM_STEP else 0.0
        new_trim = self.trim + delta
        if abs(new_trim) > MAX_TRIM:
            log.error(
                f'Trim {new_trim:+.4f} is beyond +/-{MAX_TRIM}: check the servo horn and the '
                'steering linkage. Nothing written.'
            )
            return 1

        cmds = sorted(forward)
        problem = ''
        for series, name in ((forward, 'forward'), (reverse, 'reverse')):
            if name == 'reverse' and not reverse_differs:
                continue
            pooled, notes, problem = pool_flat(cmds, [series[c] for c in cmds])
            for note in notes:
                log.info(f'{name.capitalize()}: {note}')
            if problem:
                problem = f'{name}: {problem}'
                break
            series.update(zip(cmds, pooled, strict=True))
            problem = steering.table_problem(cmds, pooled)
            if problem:
                problem = f'{name}: {problem}'
                break
        if problem:
            log.error(f'Steering table: {problem}. Nothing written.')
            return 1
        needed = {-1.0, 1.0}
        if not needed <= set(cmds):
            log.error('Steering table: full lock on both sides is needed. Nothing written.')
            return 1
        missing = sorted(set(COMMANDS) - set(cmds))
        if missing:
            log.warning(
                'Steering table without '
                + ', '.join(f'{c:+g}' for c in missing)
                + '; the table interpolates across them.'
            )

        written = []
        if delta:
            written += write_local_yaml(
                CAL_NAME, NODE, {'steering_trim': round(new_trim, 4)}, TOOL, merge=True
            )
            log.info(f'steering_trim: {self.trim:+.4f} -> {new_trim:+.4f}')
        else:
            log.info(
                f'The trim would change by {fit.trim_delta:+.4f}, below what the servo '
                f'resolves ({MIN_TRIM_STEP}). The trim stays at {self.trim:+.4f}.'
            )
        table = build_table(
            forward,
            reverse if reverse_differs else None,
            self.trim,
            delta,
            datetime.now().strftime('%Y-%m-%d %H:%M'),
        )
        written += write_local_yaml(STEERING_TABLE, '/**', steering.params_of(table), TOOL)
        right, left = table.max_curvature()
        log.info(
            f'Steering table written: full lock {1 / left:.3f} m left, {-1 / right:.3f} m right '
            f'radius; reverse {"has its own table" if reverse_differs else "uses the same table"}.'
        )
        log.info(backup_reminder(written))
        return 0

    def _report_turn_gain(self, arcs: list[Arc]) -> None:
        """Log the encoder over-read in turns as a gain for odometry.turn_gain; not written."""
        odom = node_params('odom', 'pit_node')
        current = float(odom.get('odometry.turn_gain', 0.0))
        wheelbase = float(odom.get('odometry.wheelbase_m', WHEELBASE_M))
        fit = fit_turn_gain(arcs, current, wheelbase)
        if fit is None:
            return
        self.log.info(
            f'Encoder over-read in turns: gain {fit.gain:.3f} from {fit.runs} runs; '
            f'straight runs read {fit.straight_ratio - 1:+.2%}. '
            f'odometry.turn_gain stays at {current:.3f}.'
        )

    def _check(self) -> int:
        log = self.log
        criteria = []
        stored = steering.load()
        results = self.run_pairs(plan_order(TABLE_CHECK_COMMANDS, DRIVE_SPEED))
        straight = results.get((0.0, DRIVE_SPEED))
        if straight is None:
            criteria.append(Criterion('straight drift', 'no result', '', 'FAIL', 'see above'))
        else:
            arc = straight[0]
            criteria.append(match_criterion('scan match', arc.match))
            gap = abs(arc.heading_deg - arc.odom_heading_deg)
            criteria.append(
                Criterion(
                    'lidar vs odometry',
                    f'heading {arc.heading_deg:+.1f} vs {arc.odom_heading_deg:+.1f} deg',
                    f'within {MAX_HEADING_DISAGREEMENT_DEG:.0f} deg',
                    'PASS' if gap <= MAX_HEADING_DISAGREEMENT_DEG else 'FAIL',
                    'the scan match is wrong: run the check again',
                )
            )
            criteria.append(drift_criterion(math.degrees(arc.curvature)))
        criteria += table_criteria(stored, results, self.trim)
        criteria.append(trim_criterion(self.trim, self.live_trim()))
        return finish_check(log, criteria, self.report)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true', help='check the trim in use')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    parser.add_argument(
        '--bags',
        nargs='+',
        type=Path,
        metavar='BAG',
        help='analyze these recorded runs (steer_* bag folders) instead of driving',
    )
    args = parser.parse_args(argv)
    return SteeringCalibrator(args.check, args.report, args.bags).run()


if __name__ == '__main__':
    sys.exit(main())
