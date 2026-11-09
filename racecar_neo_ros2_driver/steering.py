"""
The car's steering command-to-curvature table.

`racecar calibrate steering` measures it with the lidar and writes it to the
car profile's params/steering_table.yaml, under `/**` so any node can load
it: steering_table.command, steering_table.curvature_per_m (forward), and
steering_table.curvature_per_m_reverse when reverse differs. Curvature is in
1/m, positive to the left; commands are the /drive steering in [-1, 1].
The steering is not linear in the command, so code that needs a turn radius
reads this table rather than scaling the command.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

NAME = 'steering_table'
PREFIX = 'steering_table.'


@dataclass(frozen=True)
class SteeringTable:
    command: tuple[float, ...]  # increasing, -1..1
    curvature: tuple[float, ...]  # 1/m forward, at each command
    curvature_reverse: tuple[float, ...] | None = None  # when reverse differs
    speed: float = 0.25  # /drive speed the table was measured at
    trim: float = 0.0  # steering_trim in use while measuring
    measured: str = ''

    def _k(self, reverse: bool) -> tuple[float, ...]:
        return self.curvature_reverse if reverse and self.curvature_reverse else self.curvature

    def curvature_for(self, command: float, reverse: bool = False) -> float:
        """Return the curvature (1/m) a steering command gives; clamped to the table's ends."""
        return float(np.interp(command, self.command, self._k(reverse)))

    def command_for(self, curvature: float, reverse: bool = False) -> float:
        """
        Return the steering command for a curvature (1/m); full lock past the table's ends.

        Where the curvature is flat (the wheels at full lock before the
        command's end), the command closest to center is used.
        """
        k, c = _rising(self._k(reverse), self.command)
        return float(np.interp(curvature, k, c))

    def max_curvature(self, reverse: bool = False) -> tuple[float, float]:
        """Return the (right, left) full-lock curvature, 1/m; right is negative."""
        k = self._k(reverse)
        return k[0], k[-1]


def _rising(
    curvature: tuple[float, ...], command: tuple[float, ...]
) -> tuple[list[float], list[float]]:
    """Keep, of each run of equal curvature, the command closest to center."""
    k_out: list[float] = []
    c_out: list[float] = []
    for k, c in zip(curvature, command, strict=True):
        if k_out and k == k_out[-1]:
            if abs(c) < abs(c_out[-1]):
                c_out[-1] = c
            continue
        k_out.append(k)
        c_out.append(c)
    return k_out, c_out


def table_problem(command: list[float], curvature: list[float]) -> str:
    """Return why a measured table cannot be used, or '' when it can."""
    if len(command) != len(curvature) or len(command) < 3:
        return 'the table needs at least three commands with a curvature each'
    if any(b <= a for a, b in zip(command, command[1:], strict=False)):
        return 'the commands are not in increasing order'
    pairs = list(zip(command, curvature, strict=True))
    for (c0, k0), (c1, k1) in zip(pairs, pairs[1:], strict=False):
        if k1 < k0:
            return (
                f'curvature falls from command {c0:+g} ({k0:+.3f}/m) ' f'to {c1:+g} ({k1:+.3f}/m)'
            )
    if curvature[-1] <= curvature[0]:
        return 'curvature does not rise across the table'
    return ''


def params_of(table: SteeringTable) -> dict[str, Any]:
    """Return the ROS parameters that store `table`."""
    out: dict[str, Any] = {
        PREFIX + 'command': [float(c) for c in table.command],
        PREFIX + 'curvature_per_m': [round(float(k), 4) for k in table.curvature],
        PREFIX + 'speed': float(table.speed),
        PREFIX + 'trim': float(table.trim),
        PREFIX + 'measured': table.measured,
    }
    if table.curvature_reverse:
        out[PREFIX + 'curvature_per_m_reverse'] = [
            round(float(k), 4) for k in table.curvature_reverse
        ]
    return out


def _flat(d: dict[str, Any], prefix: str = '') -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in d.items():
        if isinstance(value, dict):
            out.update(_flat(value, f'{prefix}{key}.'))
        else:
            out[f'{prefix}{key}'] = value
    return out


def from_params(params: dict[str, Any]) -> SteeringTable | None:
    """Return the table in a {name: value} parameter dict (flat or nested), or None."""
    p = _flat(params)
    command = p.get(PREFIX + 'command')
    curvature = p.get(PREFIX + 'curvature_per_m')
    if not command or not curvature or table_problem(list(command), list(curvature)):
        return None
    reverse = p.get(PREFIX + 'curvature_per_m_reverse') or None
    if reverse and (len(reverse) != len(command) or table_problem(list(command), list(reverse))):
        reverse = None
    return SteeringTable(
        tuple(float(c) for c in command),
        tuple(float(k) for k in curvature),
        tuple(float(k) for k in reverse) if reverse else None,
        float(p.get(PREFIX + 'speed', 0.25)),
        float(p.get(PREFIX + 'trim', 0.0)),
        str(p.get(PREFIX + 'measured', '')),
    )


def load(path: Path | None = None) -> SteeringTable | None:
    """Return the car's table from its profile, or None when it has not been measured."""
    import yaml

    if path is None:
        from .car_profile import param_path

        path = param_path(NAME)
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    for section in data.values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            table = from_params(section['ros__parameters'] or {})
            if table is not None:
                return table
    return None
