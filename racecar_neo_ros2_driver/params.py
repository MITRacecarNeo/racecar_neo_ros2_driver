"""Declare a node's parameters from one table of defaults."""

from __future__ import annotations

from typing import Any

from rclpy.node import Node


def declare_params(node: Node, defaults: dict[str, Any]) -> dict[str, Any]:
    """
    Declare every name in `defaults` with its default; return {name: value in effect}.

    The value is what the launch YAML set, else the default. A default's type
    fixes the parameter's type, as with declare_parameter.
    """
    declared = node.declare_parameters('', list(defaults.items()))
    return {p.name: p.value for p in declared}
