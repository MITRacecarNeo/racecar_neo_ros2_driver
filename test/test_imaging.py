"""Tests for racecar_neo_ros2_driver/imaging.py: frame decode, scale, encode, depth colors."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from racecar_neo_ros2_driver import imaging


def msg(encoding, rows, step=None, dtype=np.uint8):
    """Build an Image-like message; a `step` past the row width pads each row with zeros."""
    arr = np.asarray(rows, dtype=dtype)
    h, w = arr.shape[:2]
    row = arr.reshape(h, -1).view(np.uint8)
    step = step or row.shape[1]
    padded = np.zeros((h, step), np.uint8)
    padded[:, : row.shape[1]] = row
    return SimpleNamespace(height=h, width=w, step=step, encoding=encoding, data=padded.tobytes())


class TestDecode:
    def test_rgb_turns_bgr(self):
        img = imaging.bgr(msg('rgb8', [[[1, 2, 3], [4, 5, 6]]]))
        assert img.shape == (1, 2, 3) and img[0, 0].tolist() == [3, 2, 1]

    def test_padded_rows_do_not_shear(self):
        # A step wider than the row: the padding must not leak into the next row.
        rows = [[[10, 10, 10], [20, 20, 20]], [[30, 30, 30], [40, 40, 40]]]
        img = imaging.bgr(msg('bgr8', rows, step=8))
        assert img[1, 0].tolist() == [30, 30, 30] and img[1, 1].tolist() == [40, 40, 40]

    def test_mono(self):
        img = imaging.bgr(msg('mono8', [[7, 8, 9]], step=4))
        assert img.tolist() == [[7, 8, 9]]

    def test_jpeg(self):
        data = cv2.imencode('.jpg', np.full((4, 6, 3), 128, np.uint8))[1].tobytes()
        img = imaging.bgr(SimpleNamespace(encoding='jpeg', data=data))
        assert img.shape == (4, 6, 3)

    def test_other_encodings(self):
        assert imaging.bgr(msg('16UC1', [[1]], dtype=np.uint16)) is None
        assert imaging.depth_m(msg('rgb8', [[[1, 2, 3]]])) is None


class TestDepth:
    def test_millimeters_to_meters_with_padding(self):
        m = imaging.depth_m(msg('16UC1', [[500, 0], [1500, 2000]], step=6, dtype=np.uint16))
        assert m.tolist() == [[0.5, 0.0], [1.5, 2.0]]

    def test_float_meters_and_nan(self):
        m = imaging.depth_m(msg('32FC1', [[1.25, np.nan]], dtype=np.float32))
        assert m.tolist() == [[1.25, 0.0]]

    def test_ramp_orders_near_over_far(self):
        # A no-return pixel is gray rather than sharing the dark end with a far wall.
        out = imaging.colorize_depth(np.array([[0.0, 0.5, 3.9, 0.0]], np.float32), 4.0)
        assert out[0][1].sum() > out[0][2].sum() > 0
        assert tuple(out[0][0]) == imaging.NO_RETURN_BGR == tuple(out[0][3])

    def test_beyond_the_ramp_stays_visible(self):
        out = imaging.colorize_depth(np.array([[99.0]], np.float32), 4.0)
        assert tuple(out[0][0]) != imaging.NO_RETURN_BGR

    def test_nearest_scaling_invents_no_surface(self):
        # Area scaling would average 2 m with a hole into a 1 m surface.
        m = np.array([[2.0, 0.0] * 4] * 2, np.float32)
        small = imaging.fit_width(m, 4, nearest=True)
        assert set(np.unique(small).tolist()) <= {0.0, 2.0}

    def test_preview_is_a_jpeg_of_the_width(self):
        data = imaging.depth_preview(msg('16UC1', [[1000] * 8] * 6, dtype=np.uint16), 4, 4.0)
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert img.shape[:2] == (3, 4)
        assert imaging.depth_preview(msg('rgb8', [[[1, 2, 3]]]), 4, 4.0) is None


class TestScale:
    @pytest.mark.parametrize('width, height', [(320, 240), (100, 75)])
    def test_keeps_the_aspect(self, width, height):
        out = imaging.fit_width(np.zeros((480, 640, 3), np.uint8), width)
        assert out.shape[:2] == (height, width)

    def test_jpeg_round_trip(self):
        data = imaging.jpeg(np.full((8, 8, 3), 200, np.uint8), 90)
        assert cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape == (8, 8, 3)
