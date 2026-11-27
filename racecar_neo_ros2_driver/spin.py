"""
Spin a node on rclpy's EventsExecutor.

The default executor rebuilds its wait set over every entity of the node on
each wake-up, which at 200 Hz inputs costs more than the callbacks. The
EventsExecutor queues events instead. It sits under rclpy.experimental in
Jazzy, so a missing import falls back to the default executor.
"""

from __future__ import annotations

from collections.abc import Callable

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


def run_node[N: Node](
    factory: Callable[[], N],
    args: list[str] | None = None,
    *,
    spinner: Callable[[Node], None] = spin,
    before_destroy: Callable[[N], None] | None = None,
) -> None:
    """
    Run a node's process: start rclpy, build the node, spin until Ctrl-C or shutdown.

    `before_destroy` runs on the node first on the way out (pit_node closes
    its serial link there).
    """
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = factory()
    try:
        spinner(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if before_destroy is not None:
            before_destroy(node)
        node.destroy_node()
        rclpy.try_shutdown()
