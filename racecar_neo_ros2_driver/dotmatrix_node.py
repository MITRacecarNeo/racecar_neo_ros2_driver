"""
MAX7219 dot matrix driver — renders /dotmatrix/text, falls back to a mode glyph.

Falls back to a centered 8x8 pictographic glyph for the current mux mode
(IDLE = pause bars, TELEOP = steering wheel, AUTO = play triangle) when
/dotmatrix/text is empty. Long text scrolls horizontally; short text static.
"""

import time

from luma.core.interface.serial import noop, spi
from luma.core.legacy import text
from luma.core.legacy.font import proportional, TINY_FONT as _LUMA_TINY_FONT
from luma.core.render import canvas
from luma.led_matrix.device import max7219
from racecar_neo_ros2_driver.mux_node import MuxMode, select_mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8MultiArray


# Local copy of TINY_FONT with the 'N' replaced by a more legible diagonal-stroke
# version. Luma's stock N (cols [60, 2, 62, 0]) reads as a notched pillar
# without a clear diagonal; this one uses 4 columns with a proper diagonal so
# "MAN" is recognizable on the 8-px-tall dot matrix.
TINY_FONT = list(_LUMA_TINY_FONT)
TINY_FONT[ord('N')] = [62, 4, 8, 62]  # 4 cols, diagonal stroke from top-left to bottom-right


# 8x8 bitmaps, one row per string. '.' = off, 'X' = on.
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

# User-facing labels — shown to the right of the glyph in TINY_FONT.
# GAMEPAD → "MAN" (abbreviated): "MANUAL" rendered at 23 px overflows the
# 16-px label region on a 24-px display; "MAN" fits at 11 px. IDLE / AUTO
# both render at 15 px and fit exactly. Internal enum stays GAMEPAD.
MODE_LABEL = {
    MuxMode.IDLE: 'IDLE',
    MuxMode.GAMEPAD: 'MAN',
    MuxMode.AUTONOMY: 'AUTO',
}


def mode_glyph(mode: MuxMode):
    """8x8 bitmap (tuple of row strings) for the given mux mode."""
    return MODE_GLYPH_BITMAP.get(mode, GLYPH_IDLE)


def mode_label(mode: MuxMode) -> str:
    """Short user-facing label paired with the glyph."""
    return MODE_LABEL.get(mode, 'IDLE')


def draw_glyph(draw, glyph, origin_x: int, origin_y: int = 0):
    """Paint an 8-row bitmap onto a luma canvas at (origin_x, origin_y)."""
    for row_idx, row in enumerate(glyph):
        for col_idx, cell in enumerate(row):
            if cell == 'X':
                draw.point((origin_x + col_idx, origin_y + row_idx), fill='white')


def rendered_text_width(message: str, font, height: int = 8) -> int:
    """
    Pixel width of `message` as luma's `text()` actually paints it.

    `text_pixel_width` overcounts because the per-char trailing zero column is
    intentional inter-glyph padding that luma's renderer strips from the end.
    For exact centering we need the true painted width — measured by rendering
    into a scratch 1-bit PIL canvas and finding the rightmost lit column.
    """
    from PIL import Image, ImageDraw
    scratch = Image.new('1', (max(1, len(message)) * 16, height))
    rendered_text_draw = ImageDraw.Draw(scratch)
    text(rendered_text_draw, (0, 0), message, fill='white', font=font)
    bbox = scratch.getbbox()
    return 0 if bbox is None else bbox[2]


def text_pixel_width(message: str, font) -> int:
    """
    Total pixel width of `message` rendered in `font` (proportional spacing).

    `font` may be a raw glyph list (e.g. TINY_FONT) or a `proportional(...)`
    wrapper; both support `font[i]` indexing for 0..255.
    """
    if not message:
        return 0
    width = 0
    for ch in message:
        code = ord(ch)
        try:
            glyph = font[code if 0 <= code < 256 else ord(' ')]
        except (IndexError, KeyError):
            glyph = font[ord(' ')]
        width += len(glyph) + 1  # +1 column gap between glyphs
    return max(0, width - 1)


def decode_pixel_array(data, expected_height: int, expected_width: int):
    """
    Decode a flat 0/1 byte sequence into a list of `expected_height` row strings.

    Accepts any iterable of ints (UInt8MultiArray.data, list, bytes). Treats
    any non-zero value as 'on' so callers can use 0/1 or 0/255. Returns rows
    as strings of '.' and 'X' so they can be passed to `draw_glyph`.

    Pads short rows with '.' and truncates long ones to `expected_width`.
    Pads missing rows with all-off, truncates extra rows. Raises ValueError
    if `data` has fewer than `expected_width` entries (must have at least one
    full row) — that catches a publisher sending the wrong message entirely.
    """
    values = list(data)
    if len(values) < expected_width:
        raise ValueError(
            f'pixel array has {len(values)} entries; need at least '
            f'{expected_width} (one full row of {expected_width} pixels)'
        )
    rows = []
    for r in range(expected_height):
        start = r * expected_width
        end = start + expected_width
        chunk = values[start:end] if start < len(values) else []
        if len(chunk) < expected_width:
            chunk = chunk + [0] * (expected_width - len(chunk))
        rows.append(''.join('X' if v else '.' for v in chunk))
    return rows


def scroll_offset(elapsed: float, total_width: int, viewport_width: int,
                  scroll_period_s: float) -> int:
    """
    Pixel offset for a left-scrolling message, looping with a pause off-screen.

    Returns 0 (no scroll) when the message fits in the viewport.
    """
    if total_width <= viewport_width:
        return 0
    travel = total_width + viewport_width  # off-right → off-left
    if scroll_period_s <= 0:
        return 0
    phase = (elapsed % scroll_period_s) / scroll_period_s
    return int(phase * travel) - viewport_width


class DotMatrixNode(Node):
    def __init__(self):
        super().__init__('dotmatrix_node')

        self.declare_parameter('spi_port', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('cascaded', 4)
        self.declare_parameter('block_orientation', -90)
        self.declare_parameter('rotate', 0)
        self.declare_parameter('contrast', 4)
        self.declare_parameter('refresh_rate_hz', 15.0)
        self.declare_parameter('scroll_period_sec', 4.0)
        self.declare_parameter('pixels_timeout_sec', 5.0)
        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)

        spi_port = self.get_parameter('spi_port').value
        spi_device = self.get_parameter('spi_device').value
        cascaded = self.get_parameter('cascaded').value
        orientation = self.get_parameter('block_orientation').value
        rotate = self.get_parameter('rotate').value
        contrast = self.get_parameter('contrast').value
        refresh_rate = self.get_parameter('refresh_rate_hz').value
        self._scroll_period = self.get_parameter('scroll_period_sec').value
        self._pixels_timeout = self.get_parameter('pixels_timeout_sec').value
        self._gamepad_btn = self.get_parameter('gamepad_enable_button').value
        self._auto_btn = self.get_parameter('autonomy_enable_button').value

        serial = spi(port=spi_port, device=spi_device, gpio=noop())
        self._device = max7219(
            serial,
            cascaded=cascaded,
            block_orientation=orientation,
            rotate=rotate,
        )
        self._device.contrast(contrast)
        # TINY_FONT for everything — narrow enough that 3-4 chars fit static
        # on a 24-px display, and shares the patched 'N' glyph used by labels.
        self._font = proportional(TINY_FONT)
        self._label_font = self._font
        self._viewport_width = self._device.width

        # Precompute per-mode x offsets so short labels (e.g. "MAN" at 11 px)
        # center inside the 16-px label region instead of sitting flush-left.
        label_region_x = 8
        label_region_w = self._viewport_width - label_region_x
        self._label_origin = {}
        for mode in MuxMode:
            w = rendered_text_width(mode_label(mode), self._label_font)
            self._label_origin[mode] = label_region_x + max(0, (label_region_w - w) // 2)

        self._user_text = ''
        self._latest_joy: Joy = None
        self._mode = MuxMode.IDLE
        self._start_time = time.monotonic()
        self._pixels_rows: list = []     # row-string list when active, else []
        self._pixels_stamp = 0.0          # monotonic time of last pixel message

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(String, '/dotmatrix/text', self._text_cb, qos)
        self.create_subscription(
            UInt8MultiArray, '/dotmatrix/pixels', self._pixels_cb, qos
        )
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)

        self.create_timer(1.0 / refresh_rate, self._render)

        self.get_logger().info(
            f'DotMatrix ready: {cascaded}x cascaded, {self._viewport_width}px wide, '
            f'refresh={refresh_rate}Hz'
        )

    def _text_cb(self, msg: String):
        if msg.data != self._user_text:
            self._user_text = msg.data
            self._start_time = time.monotonic()

    def _joy_cb(self, msg: Joy):
        self._latest_joy = msg
        new_mode = select_mode(msg.buttons, self._gamepad_btn, self._auto_btn)
        if new_mode != self._mode:
            self._mode = new_mode
            if not self._user_text:
                self._start_time = time.monotonic()

    def _pixels_cb(self, msg: UInt8MultiArray):
        try:
            self._pixels_rows = decode_pixel_array(
                msg.data, expected_height=8, expected_width=self._viewport_width
            )
            self._pixels_stamp = time.monotonic()
        except ValueError as e:
            self.get_logger().warn(f'Invalid /dotmatrix/pixels message: {e}')

    def _render(self):
        # Priority: custom pixels (if fresh) > /dotmatrix/text > glyph+label.
        if self._pixels_rows and (
            time.monotonic() - self._pixels_stamp
        ) <= self._pixels_timeout:
            with canvas(self._device) as draw:
                for row_idx, row in enumerate(self._pixels_rows):
                    for col_idx, cell in enumerate(row):
                        if cell == 'X':
                            draw.point((col_idx, row_idx), fill='white')
            return

        if not self._user_text:
            # Glyph on module 1 (x=0..7), label centered in modules 2-3
            # (x=8..23) in TINY_FONT. Per-mode origins are precomputed so
            # short labels like "MAN" (11 px) center instead of going flush-left.
            glyph = mode_glyph(self._mode)
            label = mode_label(self._mode)
            label_x = self._label_origin.get(self._mode, 8)
            with canvas(self._device) as draw:
                draw_glyph(draw, glyph, 0, 0)
                if label_x < self._viewport_width:
                    text(draw, (label_x, 1), label, fill='white',
                         font=self._label_font)
            return

        message = self._user_text
        # rendered_text_width gives the true painted width; text_pixel_width
        # over-counts by 1-3 px per char (trailing zero columns) and would
        # mis-classify static-fitting messages as needing to scroll.
        width = rendered_text_width(message, self._font)
        if width <= self._viewport_width:
            with canvas(self._device) as draw:
                text(draw, (0, 1), message, fill='white', font=self._font)
            return

        elapsed = time.monotonic() - self._start_time
        offset = scroll_offset(
            elapsed, width, self._viewport_width, self._scroll_period
        )
        with canvas(self._device) as draw:
            text(draw, (-offset, 1), message, fill='white', font=self._font)

    def destroy_node(self):
        try:
            self._device.clear()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


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
