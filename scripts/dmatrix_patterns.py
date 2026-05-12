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


def _checkerboard(width: int) -> list:
    return [((r + c) & 1) for r in range(HEIGHT) for c in range(width)]


def _all_on(width: int) -> list:
    return [1] * (HEIGHT * width)


def _sweep_frame(width: int, lit_col: int) -> list:
    return [1 if c == lit_col else 0 for r in range(HEIGHT) for c in range(width)]


def _module_id(width: int, cascaded: int) -> list:
    # Light a different row in each 8-px module so module ordering is obvious.
    # Module N (0-indexed) gets row N lit across its 8 columns.
    rows = [['.'] * width for _ in range(HEIGHT)]
    for m in range(cascaded):
        row = min(m, HEIGHT - 1)
        for c in range(m * 8, min((m + 1) * 8, width)):
            rows[row][c] = 'X'
    return [1 if rows[r][c] == 'X' else 0 for r in range(HEIGHT) for c in range(width)]


class PatternPublisher(Node):
    def __init__(self):
        super().__init__('dmatrix_pattern_publisher')
        self.pix_pub = self.create_publisher(UInt8MultiArray, '/dotmatrix/pixels', 1)
        self.txt_pub = self.create_publisher(String, '/dotmatrix/text', 1)

    def publish_pixels(self, flat_data):
        msg = UInt8MultiArray()
        msg.data = list(flat_data)
        self.pix_pub.publish(msg)

    def publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.txt_pub.publish(msg)

    def clear_text(self):
        self.publish_text('')


def run_checkerboard(node, width, duration_s):
    node.get_logger().info('checkerboard for %.1fs' % duration_s)
    deadline = time.monotonic() + duration_s
    data = _checkerboard(width)
    while time.monotonic() < deadline:
        node.publish_pixels(data)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.5)


def run_all_on(node, width, duration_s):
    node.get_logger().info('all-on for %.1fs' % duration_s)
    deadline = time.monotonic() + duration_s
    data = _all_on(width)
    while time.monotonic() < deadline:
        node.publish_pixels(data)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.5)


def run_sweep(node, width, duration_s):
    node.get_logger().info('column sweep for %.1fs' % duration_s)
    deadline = time.monotonic() + duration_s
    col = 0
    while time.monotonic() < deadline:
        node.publish_pixels(_sweep_frame(width, col))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.08)
        col = (col + 1) % width


def run_module_id(node, width, cascaded, duration_s):
    node.get_logger().info('module identifier for %.1fs' % duration_s)
    deadline = time.monotonic() + duration_s
    data = _module_id(width, cascaded)
    while time.monotonic() < deadline:
        node.publish_pixels(data)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.5)


def run_font_scroll(node, duration_s):
    # 6-char chunks in TINY_FONT render at 23-24 px static, filling all
    # three cascaded modules of the 24-px viewport with no scrolling.
    chunks = ['ABCDEF', 'GHIJKL', 'MNOPQR',
              'STUVWX', 'YZ0123', '456789']
    node.get_logger().info('font chunks (A-Z 0-9) for %.1fs' % duration_s)
    per_chunk = max(0.7, duration_s / len(chunks))
    deadline = time.monotonic() + duration_s
    idx = 0
    while time.monotonic() < deadline:
        node.publish_text(chunks[idx])
        idx = (idx + 1) % len(chunks)
        chunk_end = time.monotonic() + per_chunk
        while time.monotonic() < chunk_end and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
    node.clear_text()


PATTERNS = {
    'checkerboard': run_checkerboard,
    'all-on': run_all_on,
    'sweep': run_sweep,
    'module-id': run_module_id,
    'font': run_font_scroll,
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'pattern', choices=sorted(PATTERNS) + ['all'],
        help='Which pattern to run, or "all" to run each in sequence',
    )
    parser.add_argument('--width', type=int, default=24,
                        help='Display width in pixels (default 24 = 3 modules)')
    parser.add_argument('--cascaded', type=int, default=3,
                        help='Cascaded module count (used by module-id pattern)')
    parser.add_argument('--duration', type=float, default=4.0,
                        help='Seconds to run each pattern (default 4)')
    args = parser.parse_args(argv)

    rclpy.init()
    node = PatternPublisher()
    # Give discovery a moment so the first publish isn't lost.
    time.sleep(0.5)

    try:
        if args.pattern == 'all':
            run_checkerboard(node, args.width, args.duration)
            run_all_on(node, args.width, args.duration)
            run_sweep(node, args.width, args.duration)
            run_module_id(node, args.width, args.cascaded, args.duration)
            # Font test needs longer (~0.6s per 3-char chunk × 13 chunks).
            run_font_scroll(node, max(args.duration, 8.0))
        elif args.pattern == 'font':
            run_font_scroll(node, max(args.duration, 8.0))
        elif args.pattern == 'module-id':
            run_module_id(node, args.width, args.cascaded, args.duration)
        else:
            PATTERNS[args.pattern](node, args.width, args.duration)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
