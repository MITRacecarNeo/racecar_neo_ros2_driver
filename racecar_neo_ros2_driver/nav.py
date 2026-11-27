"""
Navigation math shared by the Nav2 drive bridge and the Nav2 launch.

The bridge turns Nav2's velocity command (speed v, turn rate w) into the
normalized /drive command the mux carries on its autonomy source:

- Speed: the firmware maps the motor command past its deadband onto a
  closed-loop target between min_mps (the floor) and max_mps
  (throttleToTargetMps in the firmware's drive_map.cpp), and throttle_node
  scales /drive speed by max_speed_forward/backward onto /motor. The bridge
  inverts both. A speed under the floor cannot be held, so it is a stop.
- Steering: the path curvature w / v goes through the car's steering table
  (racecar_neo_ros2_driver/steering.py), the reverse table when backing up.

The launch reads the footprint from config/frames.yaml and the minimum
turning radius from the steering table.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
from typing import Any

from .steering import SteeringTable

FLOOR_MARGIN = 0.002


@dataclass(frozen=True)
class SpeedMap:
    """The firmware's speed band and the throttle caps between /drive and /motor."""

    deadband: float = 0.01  # firmware cmd_deadband
    min_mps: float = 0.5  # firmware PID_CTRL::min_speed_mps: the slowest held speed
    max_mps: float = 1.0  # firmware PID_CTRL::max_speed_mps at a full motor command
    cap_forward: float = 1.0  # throttle_node max_speed_forward
    cap_backward: float = 1.0  # throttle_node max_speed_backward

    def drive_speed(self, mps: float) -> float:
        """Return the /drive speed in [-1, 1] that targets `mps`; 0 under the floor."""
        if abs(mps) < self.min_mps or mps == 0.0:
            return 0.0
        target = min(abs(mps), self.max_mps)
        span = self.max_mps - self.min_mps
        motor = self.deadband + (target - self.min_mps) / span * (1.0 - self.deadband)
        # The firmware engages only past its deadband, so the floor sits just above it.
        motor = max(motor, self.deadband + FLOOR_MARGIN)
        cap = self.cap_forward if mps > 0 else self.cap_backward
        return math.copysign(min(motor / cap, 1.0), mps)

    def speed_of(self, drive: float) -> float:
        """Return the m/s the firmware targets for a /drive speed; the inverse of drive_speed."""
        cap = self.cap_forward if drive > 0 else self.cap_backward
        motor = min(abs(drive) * cap, 1.0)
        if motor <= self.deadband:
            return 0.0
        scale = (motor - self.deadband) / (1.0 - self.deadband)
        return math.copysign(self.min_mps + scale * (self.max_mps - self.min_mps), drive)


def drive_steering(mps: float, turn_rate: float, table: SteeringTable) -> float:
    """
    Return the /drive steering for Nav2's (speed, turn rate); 0 when not moving.

    Curvature is turn rate over speed, so it keeps its meaning in reverse; the
    reverse table gives the command there.
    """
    if abs(mps) < 1e-3:
        return 0.0
    curvature = turn_rate / mps
    return max(-1.0, min(1.0, table.command_for(curvature, reverse=mps < 0)))


def min_turning_radius(table: SteeringTable) -> float:
    """Return the tightest turn the car makes in every direction (both ways, both sides), m."""
    ends = [abs(k) for reverse in (False, True) for k in table.max_curvature(reverse) if k]
    return 1.0 / min(ends) if ends else 1.0


def footprint(frames: dict[str, Any]) -> list[list[float]]:
    """Return the car's outline in base_link, meters, from frames.yaml's footprint_* keys."""
    front = float(frames['footprint_front_x_mm']) / 1000.0
    rear = float(frames['footprint_rear_x_mm']) / 1000.0
    left = float(frames['footprint_left_y_mm']) / 1000.0
    right = float(frames['footprint_right_y_mm']) / 1000.0
    return [[front, left], [front, right], [rear, right], [rear, left]]


def footprint_text(points: list[list[float]]) -> str:
    """Return a footprint as the string Nav2's costmap parameter takes."""
    return '[' + ', '.join(f'[{x:.4f}, {y:.4f}]' for x, y in points) + ']'


def nav2_params(
    base: dict[str, Any], table: SteeringTable | None, frames: dict[str, Any]
) -> dict[str, Any]:
    """
    Return Nav2's parameter tree with this car's footprint and turning radius set.

    `base` is config/nav2.yaml (with any car profile override merged on top);
    it is not changed. The costmaps live inside the controller and planner
    servers, so the values go into the tree rather than onto a node.
    """
    tree = copy.deepcopy(base)
    shape = footprint_text(footprint(frames))
    for costmap in ('local_costmap', 'global_costmap'):
        tree[costmap][costmap]['ros__parameters']['footprint'] = shape
    if table is not None:
        planner = tree['planner_server']['ros__parameters']['GridBased']
        planner['minimum_turning_radius'] = round(min_turning_radius(table), 3)
    return tree


def merge(base: dict[str, Any], top: dict[str, Any]) -> dict[str, Any]:
    """Return `base` with `top` merged in, key by key at every depth."""
    out = copy.deepcopy(base)
    for key, value in top.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out
