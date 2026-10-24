#!/usr/bin/env python3
"""
Steering trim and command-to-curvature table, referenced to the lidar.

The `steering` step of `racecar calibrate`. The car drives one short run per
steering command by itself while the operator holds RB. In each run the
wheels are set with the car still, the car drives at speed 0.25 and stops,
and the steering is never changed, so the car moves on a single arc. Two
lidar scans at rest, before and after, give the arc's chord and heading
change; curvature = 2 sin(heading / 2) / chord. No encoder distance is used.

The trim is the drift at command 0 divided by the slope near center (from
+/-0.25; at +/-0.5 the curvature already flattens toward full lock and
differs by side). throttle_node adds steering_trim to every command, so the
runs measure with the trim in use and the new trim is the old one plus that
command. A change smaller than the servo resolves is not written. Written to
the car profile's params/throttle.yaml. The curved runs also report the encoder's
over-read in turns as a gain for odometry.turn_gain, which is not written.

--check drives straight at command 0: heading drift within 1 deg/m passes; up
to 5 deg/m warns, naming servo and linkage play (up to 3 deg/m between runs);
beyond is a mechanical fault. The trim throttle_node runs with must match the
file.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
    run_sequence,
    write_local_yaml,
)
from calibrate_dynamics import (  # noqa: E402
    Bag,
    command_edges,
    Driver,
    laser_pose,
    read_bag,
    rest_scan,
    WHEELBASE_M,
)
from calibrate_odom import best_match, match_criterion, resolve_turns  # noqa: E402
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
COMMANDS = (0.0, 0.25, -0.25, 0.5, -0.5, 1.0, -1.0)
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

PLACE = 'Place the car at the middle of one edge of the clear area, pointing across it.'


def segments(command: float) -> list[tuple[float, float, float]]:
    """Wheels set while still, one drive, stop; the steering never changes."""
    # Long enough to measure, short enough that the turn stays under about
    # 100 deg so the two rest scans still overlap.
    drive_sec = 3.0 if command == 0.0 else 2.0 if abs(command) <= 0.25 else 1.0
    return [(0.0, command, 2.0), (DRIVE_SPEED, command, drive_sec), (0.0, command, 3.0)]


def run_name(command: float) -> str:
    """Return a bag-safe name: 0.25 -> 'l025', -0.5 -> 'r05', 0 -> 'c0'."""
    if command == 0.0:
        return 'c0'
    tag = f'{abs(command):.2f}'.replace('.', '').rstrip('0').ljust(2, '0')
    return ('l' if command > 0 else 'r') + tag


def describe(command: float) -> str:
    side = 'straight' if command == 0.0 else 'left' if command > 0 else 'right'
    return f'steering {command:+g} ({side})'


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
    edges = command_edges(bag)
    launch = next((e for e in edges if e[1] != 0.0), None)
    if launch is None:
        return None
    start = rest_scan(bag, launch[0] - 1.5, launch[0] - 0.1)
    end = rest_scan(bag, bag.odom_t[-1] - 2.0, bag.odom_t[-1])
    if len(start) < 50 or len(end) < 50:
        return None
    i0 = int(np.searchsorted(bag.odom_t, launch[0] - 0.1))
    a, b = tuple(bag.odom[i0]), tuple(bag.odom[-1])
    rel = compose(invert(a), b)
    heading = float(bag.odom[-1, 2] - bag.odom[i0, 2])
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
    return Arc(
        command,
        arc_curvature(true, chord),
        math.degrees(true),
        chord,
        match,
        math.degrees(heading),
        math.hypot(rel[0], rel[1]),
        float(bag.path_m[-1] - bag.path_m[i0]),
        abs(arc_m),
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
    x, y = np.array(xs), np.array(ys)
    return TurnFit(float(x @ y / (x @ x)), base, len(xs))


# The odometry heading is calibrated to well under 1%; a scan match further
# off than this has locked onto the wrong alignment.
MAX_HEADING_DISAGREEMENT_DEG = 10.0
# The encoder reads up to about 4% high in turns; beyond this the match is wrong.
MAX_CHORD_DISAGREEMENT = 0.15


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
    if abs(arc.chord_m / arc.odom_chord_m - 1.0) > MAX_CHORD_DISAGREEMENT:
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


class SteeringCalibrator:
    def __init__(self, check: bool, report: str | None) -> None:
        self.check = check
        self.report = report
        self.trim = float(node_params(CAL_NAME, NODE).get('steering_trim', 0.0))
        self.driver = Driver()
        self.log = self.driver.log
        self.laser = laser_pose()

    def measure(self, command: float) -> Arc | None:
        bag = self.driver.drive(run_name(command), segments(command), prefix='steer')
        if bag is None:
            return None
        self.log.info('Analyzing the run...')
        arc = measure_arc(read_bag(bag), self.laser, command)
        if arc is None:
            self.log.error('The lidar could not measure this run. Run it again.')
            return None
        self.log.info(
            f'Turned {arc.heading_deg:+.1f} deg over a {arc.chord_m:.3f} m chord: '
            f'curvature {arc.curvature:+.3f}/m (scan match {arc.match.rms_m * 1e3:.1f} mm, '
            f'{arc.match.inliers:.0%}).'
        )
        return arc

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

        def show(i: int) -> None:
            log.info(f'Run {i + 1} of {len(COMMANDS)}: {describe(COMMANDS[i])}')
            log.info(PLACE)

        def run_one(i: int) -> Arc | None:
            arc = self.measure(COMMANDS[i])
            if arc is None:
                return None
            problem = arc_problem(arc)
            if problem:
                log.error(f'{problem} Run it again.')
                return None
            return arc

        arcs = list(run_sequence(len(COMMANDS), show, run_one).values())
        if not arcs:
            log.info('Nothing measured.')
            return 0
        for line in table_lines(arcs):
            log.info(line)
        fit = fit_trim(arcs)
        if fit is None:
            log.error(
                f'The trim needs the run at steering 0 and one at +/-{CENTER_COMMAND:g}. '
                'Nothing written.'
            )
            return 1
        log.info(
            f'Drift at steering 0: {fit.drift_deg_per_m:+.2f} deg/m. Gain near center: left '
            f'{fit.gain_left:.2f}/m, right {fit.gain_right:.2f}/m per unit command.'
        )
        self._report_turn_gain(arcs)
        if abs(fit.trim_delta) < MIN_TRIM_STEP:
            log.info(
                f'The trim would change by {fit.trim_delta:+.4f}, below what the servo '
                f'resolves ({MIN_TRIM_STEP}). The trim stays at {self.trim:+.4f}.'
            )
            return 0
        new = self.trim + fit.trim_delta
        if abs(new) > MAX_TRIM:
            log.error(
                f'Trim {new:+.4f} is beyond +/-{MAX_TRIM}: check the servo horn and the '
                'steering linkage. Nothing written.'
            )
            return 1
        written = write_local_yaml(
            CAL_NAME, NODE, {'steering_trim': round(new, 4)}, TOOL, merge=True
        )
        log.info(f'steering_trim: {self.trim:+.4f} -> {new:+.4f}')
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
        log.info(f'Straight drive at {describe(0.0)}')
        log.info(PLACE)
        arc = self.measure(0.0)
        if arc is None:
            criteria.append(Criterion('straight drift', 'no result', '', 'FAIL', 'see above'))
        else:
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
        criteria.append(trim_criterion(self.trim, self.live_trim()))
        return finish_check(log, criteria, self.report)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true', help='check the trim in use')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(argv)
    return SteeringCalibrator(args.check, args.report).run()


if __name__ == '__main__':
    sys.exit(main())
