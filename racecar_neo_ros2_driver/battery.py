"""
Battery charge and time-left estimate from /battery/state samples.

Charge comes from a measured discharge curve: 2S battery voltage under the
car's load against the share of usable charge left. The PIT firmware's LED bar
uses the same table, so the bar and the estimate agree. Voltages are per cell
where a battery's cell count matters: the cell count is read from the first
voltage (detect_cells) or set in the config, and the curve and limits scale
with it.

- Charge: median voltage over the last soc_window_sec, each reading raised
  by r_internal_ohm times its current above curve_amps, mapped through the
  curve. The curve was measured at curve_amps, so a reading at that current
  is taken as is, and the sag while driving is added back. The median keeps
  a throttle spike from moving it.
- Power: mean of volts times amps over window_sec.
- Time left: the charge above the floor times the capacity over the mean
  current in the same window, so it reaches 0 when the guard turns the car
  off.
- Capacity: CapacityLearner counts the charge drawn since the first reading
  and compares it with the drop on the curve, so a battery that holds more or
  less than capacity_mah is measured during the run.

No ROS dependency, so it is unit-testable. Samples are (time s, volts, amps),
oldest first, amps positive on discharge.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields
from itertools import pairwise
from statistics import median
from typing import Any

Sample = tuple[float, float, float]

# (mV, charge left %) under load for a 2S battery, highest first.
_CURVE = (
    (8400, 100),
    (8200, 100),
    (8110, 95),
    (8050, 90),
    (7860, 80),
    (7740, 70),
    (7620, 60),
    (7545, 50),
    (7485, 40),
    (7450, 30),
    (7395, 20),
    (7305, 10),
    (7280, 5),
    (7195, 2),
    (7000, 0),
)
CURVE_MV = tuple(mv for mv, _ in _CURVE)
CURVE_PCT = tuple(pct for _, pct in _CURVE)

# Per-cell voltage that identifies the cell count at the first reading. The
# ranges for 1 to 6 cells do not overlap, so one voltage gives one count.
DETECT_CELL_VOLTS = (3.3, 4.35)
MAX_CELLS = 6
# A battery change without a reboot (a bench supply carrying the Pi) shows as
# charge rising this far above the start; learning starts over.
LEARN_RESET_RISE_PCT = 10.0


@dataclass(frozen=True)
class BatteryConfig:
    """Estimator and guard settings; the parameters of config/battery.yaml."""

    capacity_mah: float = 4700.0
    cells: int = 0
    curve_cells: int = 2
    curve_mv: tuple[int, ...] = CURVE_MV
    curve_pct: tuple[int, ...] = CURVE_PCT
    r_internal_ohm: float = 0.030
    curve_amps: float = 2.3
    window_sec: float = 300.0
    soc_window_sec: float = 60.0
    min_samples: int = 30
    min_current_a: float = 0.1
    stale_sec: float = 5.0
    plausible_min_cell_volts: float = 3.0
    plausible_max_cell_volts: float = 4.3
    floor_cell_volts: float = 3.55
    learn_min_drop_pct: float = 20.0
    learn_full_drop_pct: float = 40.0
    learn_min_ratio: float = 0.5
    learn_max_ratio: float = 1.2
    warn_min: float = 15.0
    shutdown_min: float = 5.0
    shutdown_enabled: bool = True

    def __post_init__(self) -> None:
        mv, pct = self.curve_mv, self.curve_pct
        if len(mv) != len(pct) or len(mv) < 2:
            raise ValueError('curve_mv and curve_pct need the same length, at least 2')
        if any(a <= b for a, b in pairwise(mv)):
            raise ValueError('curve_mv must fall strictly')
        if any(a < b for a, b in pairwise(pct)) or not 0 <= pct[-1] <= pct[0] <= 100:
            raise ValueError('curve_pct must not rise and must stay within 0 to 100')
        if not 0 <= self.cells <= MAX_CELLS or not 1 <= self.curve_cells <= MAX_CELLS:
            raise ValueError(
                f'cells must be 0 (detect) to {MAX_CELLS}; curve_cells 1 to {MAX_CELLS}'
            )
        if not self.learn_min_drop_pct < self.learn_full_drop_pct:
            raise ValueError('learn_min_drop_pct must be under learn_full_drop_pct')

    @classmethod
    def from_params(cls, params: Mapping[str, Any]) -> BatteryConfig:
        """Build from ROS parameters; unknown keys are ignored, missing ones keep defaults."""
        known = {f.name for f in fields(cls)}
        values = {k: v for k, v in params.items() if k in known}
        for key in ('curve_mv', 'curve_pct'):
            if key in values:
                values[key] = tuple(int(x) for x in values[key])
        return cls(**values)

    def floor_volts(self, cells: int) -> float:
        return self.floor_cell_volts * cells

    def plausible(self, volts: float, cells: int) -> bool:
        return (
            self.plausible_min_cell_volts * cells < volts < self.plausible_max_cell_volts * cells
        )


@dataclass(frozen=True)
class Estimate:
    """
    One estimate.

    `valid` is False when the newest sample is stale or out of the plausible
    range; every value is then None. A valid estimate still has None values
    while the window holds too few samples, and None minutes while the mean
    current is under min_current_a.
    """

    valid: bool
    volts: float | None = None
    soc: float | None = None
    power_w: float | None = None
    minutes_left: float | None = None


def detect_cells(volts: float) -> int | None:
    """Return the cell count whose per-cell voltage fits DETECT_CELL_VOLTS, or None."""
    low, high = DETECT_CELL_VOLTS
    for n in range(1, MAX_CELLS + 1):
        if low <= volts / n <= high:
            return n
    return None


def soc_percent(volts: float, curve_mv: Sequence[int], curve_pct: Sequence[int]) -> float:
    """Return the charge left (%) at `volts` under load; clamped to the curve's ends."""
    mv = volts * 1000.0
    if mv >= curve_mv[0]:
        return float(curve_pct[0])
    for (hi, lo), (p_hi, p_lo) in zip(pairwise(curve_mv), pairwise(curve_pct), strict=True):
        if mv >= lo:
            return p_lo + (mv - lo) / (hi - lo) * (p_hi - p_lo)
    return float(curve_pct[-1])


def battery_soc(volts: float, cells: int, cfg: BatteryConfig) -> float:
    """Return the charge left (%) of a `cells` battery at `volts`, on the curve scaled per cell."""
    return soc_percent(volts * cfg.curve_cells / cells, cfg.curve_mv, cfg.curve_pct)


def estimate(
    samples: Sequence[Sample],
    now: float,
    cfg: BatteryConfig,
    cells: int,
    capacity_mah: float | None = None,
) -> Estimate:
    """
    Return the estimate at `now` from samples, oldest first.

    `capacity_mah` overrides cfg.capacity_mah, for a learned capacity.
    """
    if not samples:
        return Estimate(valid=False)
    t_last, v_last, _ = samples[-1]
    if now - t_last > cfg.stale_sec or not cfg.plausible(v_last, cells):
        return Estimate(valid=False)
    window = [s for s in samples if s[0] >= now - cfg.window_sec]
    if len(window) < cfg.min_samples:
        return Estimate(valid=True, volts=v_last)

    since = now - cfg.soc_window_sec
    lift = cfg.r_internal_ohm
    recent = [v + (a - cfg.curve_amps) * lift for t, v, a in window if t >= since]
    soc = battery_soc(median(recent), cells, cfg)
    power = sum(v * a for _, v, a in window) / len(window)
    amps = sum(a for _, _, a in window) / len(window)
    minutes = None
    if amps >= cfg.min_current_a:
        above_floor = max(0.0, soc - battery_soc(cfg.floor_volts(cells), cells, cfg))
        mah = cfg.capacity_mah if capacity_mah is None else capacity_mah
        minutes = above_floor / 100.0 * mah / 1000.0 / amps * 60.0
    return Estimate(valid=True, volts=v_last, soc=soc, power_w=power, minutes_left=minutes)


class CapacityLearner:
    """
    Capacity of the battery in the car, measured during the run.

    Counts the charge drawn since the first reading. Once the curve has fallen
    learn_min_drop_pct from its first value, the measured capacity is that
    charge over the fraction used; the result moves from capacity_mah to the
    measured value as the drop reaches learn_full_drop_pct, clamped to
    learn_min_ratio to learn_max_ratio of capacity_mah. A gap between samples
    is filled at the last current.
    """

    def __init__(self, cfg: BatteryConfig) -> None:
        self._cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._last: tuple[float, float] | None = None  # (time, amps)
        self._mah = 0.0
        self._soc_start: float | None = None
        self._soc_now: float | None = None

    def update(self, t: float, amps: float, soc: float | None) -> None:
        """Add a sample; `soc` is the current estimate's charge, None until it has one."""
        if self._last is not None:
            dt = max(0.0, t - self._last[0])
            self._mah += self._last[1] * dt / 3.6
        self._last = (t, amps)
        if soc is None:
            return
        if self._soc_start is not None and soc > self._soc_start + LEARN_RESET_RISE_PCT:
            self.reset()
            self._last = (t, amps)
        if self._soc_start is None:
            self._soc_start = soc
            self._mah = 0.0
        self._soc_now = soc

    @property
    def drop_pct(self) -> float:
        if self._soc_start is None or self._soc_now is None:
            return 0.0
        return self._soc_start - self._soc_now

    @property
    def measured_mah(self) -> float | None:
        """Charge drawn over the fraction of the curve used, once the drop is large enough."""
        if self.drop_pct < self._cfg.learn_min_drop_pct:
            return None
        return self._mah / (self.drop_pct / 100.0)

    @property
    def capacity_mah(self) -> float:
        cfg = self._cfg
        measured = self.measured_mah
        if measured is None:
            return cfg.capacity_mah
        measured = min(
            max(measured, cfg.learn_min_ratio * cfg.capacity_mah),
            cfg.learn_max_ratio * cfg.capacity_mah,
        )
        span = cfg.learn_full_drop_pct - cfg.learn_min_drop_pct
        w = min(1.0, (self.drop_pct - cfg.learn_min_drop_pct) / span)
        return (1.0 - w) * cfg.capacity_mah + w * measured
