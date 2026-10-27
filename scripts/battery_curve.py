#!/usr/bin/env python3
"""
Battery discharge analysis from battery guard logs and capture CSVs.

    python3 scripts/battery_curve.py LOG [LOG ...] [--r-internal OHM] [--out FILE]

LOG is a battery guard log (<data root>/health/battery/*_guard.csv, columns
time,volts,amps,...) or a capture CSV (t,volts,amps). Logs passed together are
one battery: they are joined in time order, so a log split by a guard restart
reads as one run. Rows without a reading (guard event rows) are skipped.

The report covers:

    run         length, share of time under load, mean current idle and
                under load, where the floor was reached
    charge      mAh and Wh to the floor, and the capacity that implies on the
                curve in config/battery.yaml
    r_internal  voltage drop per amp: a straight-line fit of volts on amps in
                each 20 s window where the current moved by 1 A or more
    curve       the curve's charge left against the charge actually left, at
                each 10% of the run
    estimate    the battery guard's own estimator replayed over the log:
                minutes left against the actual minutes to the floor, and the
                error where it first reads 15 and 5 minutes (the warning and
                the estimate shutdown)

--r-internal replays the estimate with a candidate value in place of the
configured one.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import csv
from dataclasses import dataclass, replace
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import battery_guard  # noqa: E402, I100  sibling script, after the path insert
from racecar_neo_ros2_driver import car_profile  # noqa: E402
from racecar_neo_ros2_driver.battery import (  # noqa: E402
    battery_soc,
    BatteryConfig,
    detect_cells,
)

CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
# Samples further apart than this are a gap; charge across it counts at the
# last current.
GAP_SEC = 5.0
# Above this current the car counts as under load (idle is about 2.3 A).
LOAD_A = 3.0
# r_internal windows: driving never holds a current steady for long, so the
# drop per amp is fitted inside short windows, where the charge barely moves.
FIT_SEC = 20
FIT_MIN_SPREAD_A = 1.0
# A window where volts do not follow amps (a glitch, a charger) is dropped.
FIT_MIN_R2 = 0.5
# Where the estimate first reads these minutes, the error against the actual
# minutes to the floor decides whether the estimate may shut the car down.
MARKS_MIN = (15.0, 5.0)
MARK_LIMIT_MIN = 3.0

Row = tuple[float, float, float]  # (time, volts, amps)


@dataclass
class Replay:
    times: list[float]
    minutes: list[float | None]
    floor_t: float | None


def load(paths: Sequence[Path]) -> list[Row]:
    """Read and join logs; return (time, volts, amps) rows in time order, one per time."""
    rows: dict[float, Row] = {}
    for path in paths:
        with open(path, newline='') as f:
            for rec in csv.DictReader(f):
                t = rec.get('time') or rec.get('t')
                v, a = rec.get('volts'), rec.get('amps')
                if not t or not v or not a:
                    continue
                rows[float(t)] = (float(t), float(v), float(a))
    return [rows[t] for t in sorted(rows)]


def config(r_internal: float | None = None) -> BatteryConfig:
    params = car_profile.node_params('battery', 'battery_guard', CONFIG_DIR)
    cfg = BatteryConfig.from_params(params)
    return cfg if r_internal is None else replace(cfg, r_internal_ohm=r_internal)


def drawn_mah(rows: Sequence[Row]) -> list[float]:
    """Charge drawn up to each row, in mAh; a gap counts at the last current."""
    out = [0.0]
    for (t0, _, a0), (t1, _, _) in zip(rows, rows[1:]):
        out.append(out[-1] + a0 * max(0.0, t1 - t0) / 3.6)
    return out


def drawn_wh(rows: Sequence[Row]) -> float:
    pairs = zip(rows, rows[1:])
    return sum(v0 * a0 * max(0.0, t1 - t0) for (t0, v0, a0), (t1, _, _) in pairs) / 3600


def replay(rows: Sequence[Row], cfg: BatteryConfig) -> Replay:
    """
    Run the guard over the rows; record minutes left until the floor countdown starts.

    The estimate shutdown is off here, so the replay runs on to the floor it
    is measured against.
    """
    guard = battery_guard.Guard(replace(cfg, shutdown_enabled=False))
    times: list[float] = []
    minutes: list[float | None] = []
    floor_t = None
    for row in rows:
        guard.add(row)
        events = guard.tick(row[0])
        times.append(row[0])
        minutes.append(guard.last.minutes_left if guard.last.valid else None)
        if 'countdown floor' in events:
            floor_t = row[0]
            break
    return Replay(times, minutes, floor_t)


def mark_errors(rep: Replay, marks: Sequence[float] = MARKS_MIN) -> dict[float, float | None]:
    """
    Return actual minus estimated minutes where the estimate first reads each mark.

    Positive: the car ran longer than the estimate said. None when the
    estimate never reached the mark or the floor was never reached.
    """
    out: dict[float, float | None] = {}
    for mark in marks:
        out[mark] = None
        if rep.floor_t is None:
            continue
        for t, m in zip(rep.times, rep.minutes):
            if m is not None and m <= mark:
                out[mark] = (rep.floor_t - t) / 60.0 - mark
                break
    return out


def fit_windows(rows: Sequence[Row]) -> list[tuple[float, float]]:
    """Return (start time, ohms) per window whose current spans FIT_MIN_SPREAD_A and fits well."""
    found: list[tuple[float, float]] = []
    for i in range(0, len(rows) - FIT_SEC + 1, FIT_SEC):
        stop = i + FIT_SEC
        window = rows[i:stop]
        if window[-1][0] - window[0][0] > FIT_SEC + GAP_SEC:
            continue
        amps = [r[2] for r in window]
        if max(amps) - min(amps) < FIT_MIN_SPREAD_A:
            continue
        mean_a = statistics.fmean(amps)
        mean_v = statistics.fmean(r[1] for r in window)
        sxx = sum((a - mean_a) ** 2 for a in amps)
        sxy = sum((r[2] - mean_a) * (r[1] - mean_v) for r in window)
        syy = sum((r[1] - mean_v) ** 2 for r in window)
        if syy == 0 or sxy * sxy / (sxx * syy) < FIT_MIN_R2:
            continue
        found.append((window[0][0], -sxy / sxx))
    return found


def curve_table(
    rows: Sequence[Row], mah: Sequence[float], cfg: BatteryConfig, cells: int, end: int
) -> list[tuple[int, float, float, float, float]]:
    """
    Return (percent of run, volts, amps, curve %, actual %) at each 10% of the charge drawn.

    Actual % puts the floor where the curve puts it, and scales the rest by the
    charge drawn: the capacity this run implies.
    """
    floor_soc = battery_soc(cfg.floor_volts(cells), cells, cfg)
    total = mah[end]
    start_soc = battery_soc(statistics.median(r[1] for r in rows[:60]), cells, cfg)
    table = []
    for pct in range(0, 101, 10):
        target = total * pct / 100.0
        i = next(k for k in range(end + 1) if mah[k] >= target or k == end)
        lo, hi = max(0, i - 30), i + 30
        near = rows[lo:hi]
        volts = statistics.median(r[1] for r in near)
        amps = statistics.fmean(r[2] for r in near)
        curve = battery_soc(volts, cells, cfg)
        actual = floor_soc + (start_soc - floor_soc) * (1.0 - pct / 100.0)
        table.append((pct, volts, amps, curve, actual))
    return table


def report(paths: Sequence[Path], r_internal: float | None = None) -> str:
    rows = load(paths)
    if len(rows) < 120:
        return f'Too few readings: {len(rows)}.'
    cfg = config(r_internal)
    cells = cfg.cells or detect_cells(max(r[1] for r in rows[:30])) or 2
    rep = replay(rows, cfg)
    end = len(rep.times) - 1 if rep.floor_t else len(rows) - 1
    run = rows[: end + 1]
    mah = drawn_mah(rows)
    busy = [r[2] for r in run if r[2] > LOAD_A]
    idle = [r[2] for r in run if r[2] <= LOAD_A]
    start_soc = battery_soc(statistics.median(r[1] for r in rows[:60]), cells, cfg)
    if rep.floor_t:
        end_soc = battery_soc(cfg.floor_volts(cells), cells, cfg)
        where = 'at the floor'
    else:
        end_soc = battery_soc(statistics.median(r[1] for r in run[-60:]), cells, cfg)
        where = 'at the end of the log'
    used = start_soc - end_soc
    lines = [
        '# Battery run',
        '',
        'Logs: ' + ', '.join(p.name for p in paths),
        '',
        '## Run',
        '',
        f'- Length: {(run[-1][0] - run[0][0]) / 60:.1f} min, {len(run)} readings, '
        f'{cells}S, {run[0][1]:.2f} V to {run[-1][1]:.2f} V',
        f'- Floor ({cfg.floor_volts(cells):.2f} V, 30 s median): '
        + ('reached' if rep.floor_t else 'not reached; figures run to the end of the log'),
        f'- Under load (over {LOAD_A:.1f} A): {100 * len(busy) / len(run):.0f}% of the time, '
        f'mean {statistics.fmean(busy) if busy else 0:.2f} A; idle mean '
        f'{statistics.fmean(idle) if idle else 0:.2f} A',
        '',
        '## Charge',
        '',
        f'- Drawn {where.replace("at", "to", 1)}: {mah[end]:.0f} mAh, {drawn_wh(run):.1f} Wh',
        f'- Curve from {start_soc:.0f}% to {end_soc:.0f}% {where}: '
        + (
            f'capacity {mah[end] / (used / 100):.0f} mAh (configured {cfg.capacity_mah:.0f})'
            if used >= cfg.learn_min_drop_pct
            else 'too little used to measure the capacity'
        ),
    ]
    fits = fit_windows(run)
    lines += ['', '## r_internal', '']
    if fits:
        ohms = sorted(f[1] for f in fits)
        q = statistics.quantiles(ohms, n=4) if len(ohms) >= 4 else [ohms[0], ohms[0], ohms[-1]]
        lines.append(
            f'- {len(fits)} windows: median {statistics.median(ohms) * 1000:.0f} mOhm, '
            f'middle half {q[0] * 1000:.0f} to {q[2] * 1000:.0f} mOhm '
            f'(configured {cfg.r_internal_ohm * 1000:.0f})'
        )
    else:
        lines.append(f'- No {FIT_SEC} s window with the current moving {FIT_MIN_SPREAD_A:.0f} A.')
    lines += [
        '',
        '## Curve',
        '',
        '| Run % | Volts | Amps | Curve % | Actual % |',
        '| --- | --- | --- | --- | --- |',
    ]
    lines += [
        f'| {p} | {v:.3f} | {a:.2f} | {c:.1f} | {act:.1f} |'
        for p, v, a, c, act in curve_table(rows, mah, cfg, cells, end)
    ]
    errors = mark_errors(rep)
    lines += ['', '## Estimate', '']
    lines.append(f'- r_internal in the replay: {cfg.r_internal_ohm * 1000:.0f} mOhm')
    for mark, err in errors.items():
        if err is None:
            lines.append(f'- {mark:.0f} min: not reached')
            continue
        verdict = 'within' if abs(err) <= MARK_LIMIT_MIN else 'outside'
        lines.append(
            f'- {mark:.0f} min: actual {mark + err:.1f} min to the floor, error {err:+.1f} min '
            f'({verdict} {MARK_LIMIT_MIN:.0f} min)'
        )
    return '\n'.join(lines) + '\n'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--r-internal', type=float, help='ohms, in place of the configured value')
    parser.add_argument('--out', type=Path, help='also write the report here')
    args = parser.parse_args(argv)
    text = report(args.logs, args.r_internal)
    print(text, end='')
    if args.out:
        args.out.write_text(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
