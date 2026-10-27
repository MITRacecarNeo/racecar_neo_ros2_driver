"""
Spin a node on rclpy's EventsExecutor.

The default executor rebuilds its wait set over every entity of the node on
each wake-up, which at 200 Hz inputs costs more than the callbacks. The
EventsExecutor queues events instead. It sits under rclpy.experimental in
Jazzy, so a missing import falls back to the default executor.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


def spin(node: Node) -> None:
    try:
        from rclpy.experimental import EventsExecutor
    except ImportError:
        rclpy.spin(node)
        return
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
