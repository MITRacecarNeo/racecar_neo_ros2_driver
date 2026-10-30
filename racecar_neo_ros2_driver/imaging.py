"""
Camera frames for the pages the car serves: decode, scale, encode, color depth.

Frames arrive as sensor_msgs/Image. Every reader honors the message's row
stride (`step`), so a padded row never shears the picture.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

# Depth coloring. Near reads bright and far reads dark, on the perceptually
# uniform inferno ramp: a single ordered scale, so a step in color is a step
# in distance, and it survives being read by a color blind student. The ramp
# starts at DEPTH_FLOOR rather than 0 so the darkest real reading is still
# above black. No return is flat gray, so the stereo blind strip on the left
# and floor closer than the minimum range do not read as a dark object.
DEPTH_FLOOR = 24
DEPTH_NEAR_M = 0.15  # below the D435i minimum range; nothing valid is nearer
NO_RETURN_BGR = (102, 81, 80)


def _rows(msg: Any, dtype: Any, channels: int) -> np.ndarray:
    """Return the pixel rows, padding dropped; a step of 0 means unpadded rows."""
    item = np.dtype(dtype).itemsize
    step = msg.step or msg.width * channels * item
    buf = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, step // item)
    img = buf[:, : msg.width * channels]
    return img.reshape(msg.height, msg.width, channels) if channels > 1 else img


def bgr(msg: Any) -> np.ndarray | None:
    """Return a color or mono frame as a BGR (or gray) array, or None for other encodings."""
    if msg.encoding == 'jpeg':
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = _rows(msg, np.uint8, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        return _rows(msg, np.uint8, 1)
    return None


def depth_m(msg: Any) -> np.ndarray | None:
    """
    Return a depth frame in meters, 0 where there is no return.

    16UC1 (and mono16) is millimeters, the D435i default; 32FC1 is meters,
    which some drivers publish instead.
    """
    if msg.encoding in ('16UC1', 'mono16'):
        return _rows(msg, np.uint16, 1).astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        return np.nan_to_num(_rows(msg, np.float32, 1), nan=0.0)
    return None


def fit_width(img: np.ndarray, width: int, nearest: bool = False) -> np.ndarray:
    """
    Scale `img` to `width`, keeping its aspect.

    Depth takes nearest: averaging a valid reading against a zero would invent
    a surface halfway to a hole in the depth image.
    """
    h, w = img.shape[:2]
    size = (width, max(1, round(width * h / w)))
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST if nearest else cv2.INTER_AREA)


def jpeg(img: np.ndarray, quality: int = 80) -> bytes:
    return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])[1].tobytes()


def colorize_depth(meters: np.ndarray, far_m: float) -> np.ndarray:
    """
    Turn depth in meters into a BGR picture.

    Bright is near, dark is far, gray is no return. A reading beyond far_m
    clamps to the dark end rather than dropping out, so a far wall still
    reads as a wall.
    """
    valid = meters > 0
    near = min(DEPTH_NEAR_M, far_m)
    span = max(far_m - near, 1e-3)
    norm = np.clip((meters - near) / span, 0.0, 1.0)
    ramp = (DEPTH_FLOOR + (1.0 - norm) * (255 - DEPTH_FLOOR)).astype(np.uint8)
    out = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    out[~valid] = NO_RETURN_BGR
    return out


def depth_preview(msg: Any, width: int, far_m: float, quality: int = 80) -> bytes | None:
    """Return a colored depth JPEG `width` wide, or None for an encoding that is not depth."""
    meters = depth_m(msg)
    if meters is None:
        return None
    return jpeg(colorize_depth(fit_width(meters, width, nearest=True), far_m), quality)
