"""
Battery charge and time-left estimate from /battery/state samples.

Charge comes from a measured discharge curve: battery voltage under the car's
load against the share of usable charge left, down to the floor voltage. The
PIT firmware's LED bar uses the same table, so the bar and the estimate agree.

- Charge: median voltage over the last soc_window_sec, plus amps times
  r_internal_ohm (0 by default; the curve is already under load), mapped
  through the curve. The median keeps a throttle spike from moving it.
- Power: mean of volts times amps over window_sec.
- Time left: charge share times capacity over the mean current in the same
  window.

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

# (mV, charge left %) under load, highest first.
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


@dataclass(frozen=True)
class BatteryConfig:
    """Estimator and guard settings; the parameters of config/battery.yaml."""

    capacity_mah: float = 4700.0
    curve_mv: tuple[int, ...] = CURVE_MV
    curve_pct: tuple[int, ...] = CURVE_PCT
    r_internal_ohm: float = 0.0
    window_sec: float = 300.0
    soc_window_sec: float = 60.0
    min_samples: int = 30
    min_current_a: float = 0.1
    stale_sec: float = 5.0
    plausible_min_volts: float = 6.0
    plausible_max_volts: float = 8.6
    warn_min: float = 15.0
    shutdown_min: float = 5.0
    floor_volts: float = 7.0
    shutdown_enabled: bool = False

    def __post_init__(self) -> None:
        mv, pct = self.curve_mv, self.curve_pct
        if len(mv) != len(pct) or len(mv) < 2:
            raise ValueError('curve_mv and curve_pct need the same length, at least 2')
        if any(a <= b for a, b in pairwise(mv)):
            raise ValueError('curve_mv must fall strictly')
        if any(a < b for a, b in pairwise(pct)) or not 0 <= pct[-1] <= pct[0] <= 100:
            raise ValueError('curve_pct must not rise and must stay within 0 to 100')

    @classmethod
    def from_params(cls, params: Mapping[str, Any]) -> BatteryConfig:
        """Build from ROS parameters; unknown keys are ignored, missing ones keep defaults."""
        known = {f.name for f in fields(cls)}
        values = {k: v for k, v in params.items() if k in known}
        for key in ('curve_mv', 'curve_pct'):
            if key in values:
                values[key] = tuple(int(x) for x in values[key])
        return cls(**values)


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


def soc_percent(volts: float, curve_mv: Sequence[int], curve_pct: Sequence[int]) -> float:
    """Return the charge left (%) at `volts` under load; clamped to the curve's ends."""
    mv = volts * 1000.0
    if mv >= curve_mv[0]:
        return float(curve_pct[0])
    for (hi, lo), (p_hi, p_lo) in zip(pairwise(curve_mv), pairwise(curve_pct), strict=True):
        if mv >= lo:
            return p_lo + (mv - lo) / (hi - lo) * (p_hi - p_lo)
    return float(curve_pct[-1])


def estimate(samples: Sequence[Sample], now: float, cfg: BatteryConfig) -> Estimate:
    """Return the estimate at `now` from samples, oldest first."""
    if not samples:
        return Estimate(valid=False)
    t_last, v_last, _ = samples[-1]
    if now - t_last > cfg.stale_sec or not (
        cfg.plausible_min_volts < v_last < cfg.plausible_max_volts
    ):
        return Estimate(valid=False)
    window = [s for s in samples if s[0] >= now - cfg.window_sec]
    if len(window) < cfg.min_samples:
        return Estimate(valid=True, volts=v_last)

    recent = [v + a * cfg.r_internal_ohm for t, v, a in window if t >= now - cfg.soc_window_sec]
    soc = soc_percent(median(recent), cfg.curve_mv, cfg.curve_pct)
    power = sum(v * a for _, v, a in window) / len(window)
    amps = sum(a for _, _, a in window) / len(window)
    minutes = None
    if amps >= cfg.min_current_a:
        minutes = soc / 100.0 * cfg.capacity_mah / 1000.0 / amps * 60.0
    return Estimate(valid=True, volts=v_last, soc=soc, power_w=power, minutes_left=minutes)
