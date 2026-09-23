#!/usr/bin/env python3
"""
Publish dot matrix self-test patterns to /dotmatrix/pixels or /dotmatrix/text.

Requires dotmatrix_node to be running (the patterns flow through the same
ROS topics a user-facing publisher would).
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

HEIGHT = 8
MODULE_WIDTH = 8
FONT_MIN_SECONDS = 8.0
# 6-char chunks in TINY_FONT render 23-24 px wide, filling the 24-px display
# without scrolling.
FONT_CHUNKS = ['ABCDEF', 'GHIJKL', 'MNOPQR', 'STUVWX', 'YZ0123', '456789']


def _checkerboard(width: int) -> list[int]:
    return [((r + c) & 1) for r in range(HEIGHT) for c in range(width)]


def _all_on(width: int) -> list[int]:
    return [1] * (HEIGHT * width)


def _sweep_frame(width: int, lit_col: int) -> list[int]:
    return [1 if c == lit_col else 0 for r in range(HEIGHT) for c in range(width)]


def _module_id(width: int, cascaded: int) -> list[int]:
    # Module N lights row N (clamped to the last row) across its 8 columns.
    lit = set()
    for m in range(cascaded):
        row = min(m, HEIGHT - 1)
        for c in range(m * MODULE_WIDTH, min((m + 1) * MODULE_WIDTH, width)):
            lit.add((row, c))
    return [1 if (r, c) in lit else 0 for r in range(HEIGHT) for c in range(width)]


class PatternPublisher(Node):
    def __init__(self) -> None:
        super().__init__('dmatrix_pattern_publisher')
        self.pix_pub = self.create_publisher(UInt8MultiArray, '/dotmatrix/pixels', 1)
        self.txt_pub = self.create_publisher(String, '/dotmatrix/text', 1)

    def publish_pixels(self, flat_data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = list(flat_data)
        self.pix_pub.publish(msg)

    def publish_text(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.txt_pub.publish(msg)


def _hold(node: PatternPublisher, data: list[int], duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish_pixels(data)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.5)


def run_checkerboard(node: PatternPublisher, args: argparse.Namespace) -> None:
    node.get_logger().info(f'checkerboard for {args.duration:.1f}s')
    _hold(node, _checkerboard(args.width), args.duration)


def run_all_on(node: PatternPublisher, args: argparse.Namespace) -> None:
    node.get_logger().info(f'all-on for {args.duration:.1f}s')
    _hold(node, _all_on(args.width), args.duration)


def run_sweep(node: PatternPublisher, args: argparse.Namespace) -> None:
    node.get_logger().info(f'column sweep for {args.duration:.1f}s')
    deadline = time.monotonic() + args.duration
    col = 0
    while time.monotonic() < deadline:
        node.publish_pixels(_sweep_frame(args.width, col))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.08)
        col = (col + 1) % args.width


def run_module_id(node: PatternPublisher, args: argparse.Namespace) -> None:
    node.get_logger().info(f'module identifier for {args.duration:.1f}s')
    _hold(node, _module_id(args.width, args.cascaded), args.duration)


def run_font_scroll(node: PatternPublisher, args: argparse.Namespace) -> None:
    duration_s = max(args.duration, FONT_MIN_SECONDS)
    node.get_logger().info(f'font chunks (A-Z 0-9) for {duration_s:.1f}s')
    per_chunk = max(0.7, duration_s / len(FONT_CHUNKS))
    deadline = time.monotonic() + duration_s
    idx = 0
    while time.monotonic() < deadline:
        node.publish_text(FONT_CHUNKS[idx])
        idx = (idx + 1) % len(FONT_CHUNKS)
        chunk_end = time.monotonic() + per_chunk
        while time.monotonic() < chunk_end and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
    node.publish_text('')


PATTERNS = {
    'checkerboard': run_checkerboard,
    'all-on': run_all_on,
    'sweep': run_sweep,
    'module-id': run_module_id,
    'font': run_font_scroll,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'pattern',
        choices=sorted(PATTERNS) + ['all'],
        help='Which pattern to run, or "all" to run each in sequence',
    )
    parser.add_argument(
        '--width', type=int, default=24, help='Display width in pixels (default 24 = 3 modules)'
    )
    parser.add_argument(
        '--cascaded', type=int, default=3, help='Cascaded module count (used by module-id pattern)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=4.0,
        help=f'Seconds to run each pattern (default 4; font at least {FONT_MIN_SECONDS:.0f})',
    )
    args = parser.parse_args(argv)
    names = list(PATTERNS) if args.pattern == 'all' else [args.pattern]

    rclpy.init()
    node = PatternPublisher()
    # Give discovery a moment so the first publish isn't lost.
    time.sleep(0.5)
    try:
        for name in names:
            PATTERNS[name](node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
