"""Small pose and time conversions shared by the tools and nodes."""

from __future__ import annotations

import math
from typing import Any


def yaw_of(q: Any) -> float:
    """Return the yaw in radians (-pi, pi] of a quaternion with x, y, z, w fields."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_sec(stamp: Any) -> float:
    """Return a builtin_interfaces Time (sec, nanosec) as float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9
