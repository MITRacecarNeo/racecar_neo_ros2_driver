"""
Dot-matrix rasterizer that composites the idle/student display into a frame.

Renders the same content the old Pi-SPI node did -- student pixels or text, an
intro splash, and the drive-mode glyph + label -- but publishes an 8x24 frame on
/dotmatrix/frame instead of driving SPI. pit_node forwards the frame to the
Teensy, which owns the MAX7219 since v0.3.0. The Teensy keeps a minimal splash +
letter glyph of its own as a fallback for when the Pi driver is down.
"""

import time

from luma.core.legacy import text
from luma.core.legacy.font import proportional, TINY_FONT as _LUMA_TINY_FONT
from PIL import Image, ImageDraw
from racecar_neo_ros2_driver.mux_node import MuxMode, select_mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8MultiArray

# Local copy of TINY_FONT with a more legible diagonal-stroke 'N' (luma's stock
# glyph reads as a notched pillar on an 8-px-tall matrix), so "MAN" is clear.
TINY_FONT = list(_LUMA_TINY_FONT)
TINY_FONT[ord('N')] = [62, 4, 8, 62]


# 8x8 mode glyphs, one row per string. '.' = off, 'X' = on.
GLYPH_IDLE = (
    '........',
    '.XX..XX.',
    '.XX..XX.',
    '.XX..XX.',
    '.XX..XX.',
    '.XX..XX.',
    '.XX..XX.',
    '........',
)
GLYPH_TELEOP = (
    '..XXXX..',
    '.X....X.',
    'X..XX..X',
    'X.X..X.X',
    'X.X..X.X',
    'X..XX..X',
    '.X....X.',
    '..XXXX..',
)
GLYPH_AUTO = (
    '.X......',
    '.XX.....',
    '.XXX....',
    '.XXXX...',
    '.XXXXX..',
    '.XXXX...',
    '.XXX....',
    '.XX.....',
)

MODE_GLYPH_BITMAP = {
    MuxMode.IDLE: GLYPH_IDLE,
    MuxMode.GAMEPAD: GLYPH_TELEOP,
    MuxMode.AUTONOMY: GLYPH_AUTO,
}

# User-facing labels shown to the right of the glyph in TINY_FONT. GAMEPAD ->
# "MAN": "MANUAL" renders at 23 px and overflows the 16-px label region on a
# 24-px display; "MAN" fits at 11 px. IDLE / AUTO render at 15 px and fit.
MODE_LABEL = {
    MuxMode.IDLE: 'IDLE',
    MuxMode.GAMEPAD: 'MAN',
    MuxMode.AUTONOMY: 'AUTO',
}


def mode_glyph(mode: MuxMode):
    """Return the 8x8 bitmap (tuple of row strings) for the given mux mode."""
    return MODE_GLYPH_BITMAP.get(mode, GLYPH_IDLE)


def mode_label(mode: MuxMode) -> str:
    """Return the short user-facing label paired with the glyph."""
    return MODE_LABEL.get(mode, 'IDLE')


def draw_glyph(draw, glyph, origin_x: int, origin_y: int = 0):
    """Paint an 8-row bitmap onto a PIL/luma draw at (origin_x, origin_y)."""
    for row_idx, row in enumerate(glyph):
        for col_idx, cell in enumerate(row):
            if cell == 'X':
                draw.point((origin_x + col_idx, origin_y + row_idx), fill='white')


_RENDERED_WIDTH_CACHE: dict = {}


def rendered_text_width(message: str, font, height: int = 8) -> int:
    """Pixel width of `message` as luma's `text()` actually paints it (memoized)."""
    key = (message, id(font), height)
    cached = _RENDERED_WIDTH_CACHE.get(key)
    if cached is not None:
        return cached
    scratch = Image.new('1', (max(1, len(message)) * 16, height))
    text(ImageDraw.Draw(scratch), (0, 0), message, fill='white', font=font)
    bbox = scratch.getbbox()
    width = 0 if bbox is None else bbox[2]
    _RENDERED_WIDTH_CACHE[key] = width
    return width


def decode_pixel_array(data, expected_height: int, expected_width: int):
    """
    Decode a flat 0/1 (or 0/255) byte sequence into row strings of '.'/'X'.

    Produces `expected_height` rows. Pads short rows/frames with off, truncates
    long ones. Raises ValueError if there is not even one full row.
    """
    values = list(data)
    if len(values) < expected_width:
        raise ValueError(
            f'pixel array has {len(values)} entries; need at least '
            f'{expected_width} (one full row of {expected_width} pixels)'
        )
    rows = []
    for r in range(expected_height):
        start, end = r * expected_width, (r + 1) * expected_width
        chunk = values[start:end] if start < len(values) else []
        if len(chunk) < expected_width:
            chunk = chunk + [0] * (expected_width - len(chunk))
        rows.append(''.join('X' if v else '.' for v in chunk))
    return rows


def scroll_offset(
    elapsed: float, total_width: int, viewport_width: int, scroll_period_s: float
) -> int:
    """Pixel offset for a left-scrolling message; 0 when it fits the viewport."""
    if total_width <= viewport_width or scroll_period_s <= 0:
        return 0
    travel = total_width + viewport_width
    phase = (elapsed % scroll_period_s) / scroll_period_s
    return int(phase * travel) - viewport_width


class DotMatrixNode(Node):
    def __init__(self):
        super().__init__('dotmatrix_node')

        self.declare_parameter('cascaded', 3)  # 8-px modules -> width
        self.declare_parameter('refresh_rate_hz', 15.0)
        self.declare_parameter('scroll_period_sec', 4.0)
        self.declare_parameter('pixels_timeout_sec', 5.0)
        self.declare_parameter('text_timeout_sec', 6.0)
        self.declare_parameter('splash_message', '>>> Welcome to RACECAR Neo! >>>')
        self.declare_parameter('splash_period_sec', 8.0)
        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)
        self.declare_parameter('frame_topic', '/dotmatrix/frame')

        self._height = 8
        self._width = int(self.get_parameter('cascaded').value) * 8
        refresh_rate = float(self.get_parameter('refresh_rate_hz').value)
        self._scroll_period = float(self.get_parameter('scroll_period_sec').value)
        self._pixels_timeout = float(self.get_parameter('pixels_timeout_sec').value)
        self._text_timeout = float(self.get_parameter('text_timeout_sec').value)
        self._splash_message = self.get_parameter('splash_message').value
        self._splash_period = float(self.get_parameter('splash_period_sec').value)
        self._gamepad_btn = int(self.get_parameter('gamepad_enable_button').value)
        self._auto_btn = int(self.get_parameter('autonomy_enable_button').value)
        self._font = proportional(TINY_FONT)

        # Precompute per-mode label x so short labels (e.g. "MAN") center in the
        # 16-px region to the right of the 8-px glyph.
        label_region_x = 8
        label_region_w = self._width - label_region_x
        self._label_origin = {
            mode: label_region_x
            + max(0, (label_region_w - rendered_text_width(mode_label(mode), self._font)) // 2)
            for mode in MuxMode
        }

        self._user_text = ''
        self._text_start = time.monotonic()
        self._text_stamp = 0.0
        self._pixels_rows: list = []
        self._pixels_stamp = 0.0
        self._mode = MuxMode.IDLE
        self._splash_start = time.monotonic()
        self._splash_done = not self._splash_message

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            UInt8MultiArray, self.get_parameter('frame_topic').value, qos
        )
        self.create_subscription(String, '/dotmatrix/text', self._text_cb, qos)
        self.create_subscription(UInt8MultiArray, '/dotmatrix/pixels', self._pixels_cb, qos)
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)
        self.create_timer(1.0 / refresh_rate, self._render)

        self.get_logger().info(
            f'DotMatrix rasterizer ready: {self._width}x{self._height}, '
            f'refresh={refresh_rate}Hz -> {self.get_parameter("frame_topic").value}'
        )

    def _text_cb(self, msg: String):
        # Stamp every message for the freshness timeout (re-publishing the same
        # text keeps it up); reset the scroll origin only when the text changes.
        self._text_stamp = time.monotonic()
        if msg.data != self._user_text:
            self._user_text = msg.data
            self._text_start = self._text_stamp

    def _pixels_cb(self, msg: UInt8MultiArray):
        try:
            self._pixels_rows = decode_pixel_array(
                msg.data, expected_height=self._height, expected_width=self._width
            )
            self._pixels_stamp = time.monotonic()
        except ValueError as e:
            self.get_logger().warn(f'Invalid /dotmatrix/pixels message: {e}')

    def _joy_cb(self, msg: Joy):
        self._mode = select_mode(msg.buttons, self._gamepad_btn, self._auto_btn)

    def _new_image(self):
        """Return a blank (1-bit) width x height image and its draw handle."""
        img = Image.new('1', (self._width, self._height))
        return img, ImageDraw.Draw(img)

    def _to_frame(self, img) -> list:
        """Flatten a width x height 1-bit image to a row-major 0/1 list."""
        px = img.load()
        return [1 if px[c, r] else 0 for r in range(self._height) for c in range(self._width)]

    def _rows_to_frame(self, rows) -> list:
        frame = []
        for r in range(self._height):
            row = rows[r] if r < len(rows) else ''
            for c in range(self._width):
                frame.append(1 if c < len(row) and row[c] == 'X' else 0)
        return frame

    def _text_frame(self, message: str) -> list:
        width = rendered_text_width(message, self._font)
        offset = (
            0
            if width <= self._width
            else scroll_offset(
                time.monotonic() - self._text_start, width, self._width, self._scroll_period
            )
        )
        img, draw = self._new_image()
        text(draw, (-offset, 1), message, fill='white', font=self._font)
        return self._to_frame(img)

    def _scroll_frame(self, message: str, elapsed: float, period: float) -> list:
        width = rendered_text_width(message, self._font)
        offset = scroll_offset(elapsed, width, self._width, period)
        img, draw = self._new_image()
        text(draw, (-offset, 1), message, fill='white', font=self._font)
        return self._to_frame(img)

    def _glyph_label_frame(self) -> list:
        img, draw = self._new_image()
        draw_glyph(draw, mode_glyph(self._mode), 0, 0)
        label_x = self._label_origin.get(self._mode, 8)
        if label_x < self._width:
            text(draw, (label_x, 1), mode_label(self._mode), fill='white', font=self._font)
        return self._to_frame(img)

    def _render(self):
        # Priority: fresh pixels > text > splash (one pass) > glyph + label.
        # Idle always renders a frame so the Teensy shows the rich display.
        now = time.monotonic()
        if self._pixels_rows and (now - self._pixels_stamp) <= self._pixels_timeout:
            frame = self._rows_to_frame(self._pixels_rows)
        elif self._user_text and (now - self._text_stamp) <= self._text_timeout:
            frame = self._text_frame(self._user_text)
        elif self._splash_message and not self._splash_done:
            elapsed = now - self._splash_start
            if elapsed >= self._splash_period:
                self._splash_done = True
                frame = self._glyph_label_frame()
            else:
                frame = self._scroll_frame(self._splash_message, elapsed, self._splash_period)
        else:
            frame = self._glyph_label_frame()
        self._pub.publish(UInt8MultiArray(data=frame))


def main(args=None):
    rclpy.init(args=args)
    node = DotMatrixNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
