"""QoS profiles the nodes and tools share."""

from __future__ import annotations

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# State a late subscriber still needs: the last message is kept and resent.
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def sensor(depth: int = 1) -> QoSProfile:
    """Return best-effort, volatile, keep-last `depth`: only fresh samples matter."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# The newest sample only, for control-rate streams.
SENSOR_LATEST = sensor(1)
