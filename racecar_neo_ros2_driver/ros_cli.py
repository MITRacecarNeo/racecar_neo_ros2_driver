"""
ROS plumbing for the command-line tools: a node for one job, a service call with timeouts.

rclpy is imported inside the functions, so a tool can load this module
without a sourced ROS environment.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import time
from typing import Any

POLL_SEC = 0.1


@contextmanager
def ros_node(name: str, **node_kwargs: Any) -> Iterator[Any]:
    """Start rclpy and yield a node; destroy it and shut rclpy down on the way out."""
    import rclpy

    rclpy.init()
    node = rclpy.create_node(name, **node_kwargs)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def call_service(
    node: Any, srv_type: Any, name: str, request: Any, *, wait_sec: float, timeout_sec: float
) -> tuple[str, Any]:
    """
    Call a service from a node nothing else spins; return (status, response).

    status is 'unavailable' when the service does not appear within wait_sec,
    'timeout' when no answer comes within timeout_sec, else 'ok'.
    """
    import rclpy

    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=wait_sec):
            return 'unavailable', None
        future = client.call_async(request)
        end = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=POLL_SEC)
        return ('ok', future.result()) if future.done() else ('timeout', None)
    finally:
        node.destroy_client(client)
