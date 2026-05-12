"""Unit tests for edgetpu_node pure helpers."""

import numpy as np
import pytest
from racecar_neo_ros2_driver.edgetpu_node import (
    image_msg_to_rgb,
    load_labels,
    map_output_tensors,
    resize_rgb,
)
from sensor_msgs.msg import Image


def _make_image(width, height, encoding, data):
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = width * 3
    msg.data = bytes(data)
    return msg


class TestImageMsgToRgb:
    def test_rgb8_round_trip(self):
        # Single pixel: red.
        msg = _make_image(1, 1, 'rgb8', [255, 0, 0])
        arr = image_msg_to_rgb(msg)
        assert arr.shape == (1, 1, 3)
        assert arr[0, 0].tolist() == [255, 0, 0]

    def test_bgr8_is_swapped_to_rgb(self):
        # Single pixel: stored as B=255, G=0, R=0 → should come back as RGB (0, 0, 255).
        msg = _make_image(1, 1, 'bgr8', [255, 0, 0])
        arr = image_msg_to_rgb(msg)
        assert arr[0, 0].tolist() == [0, 0, 255]

    def test_rejects_unknown_encoding(self):
        msg = _make_image(1, 1, 'yuv422', [0, 0, 0])
        with pytest.raises(ValueError):
            image_msg_to_rgb(msg)

    def test_preserves_dimensions(self):
        # 2x3 image, all green pixels in rgb8.
        data = [0, 255, 0] * 6
        msg = _make_image(3, 2, 'rgb8', data)
        arr = image_msg_to_rgb(msg)
        assert arr.shape == (2, 3, 3)
        assert (arr[..., 1] == 255).all()


class TestResizeRgb:
    def test_resize_changes_shape(self):
        src = np.full((10, 10, 3), 128, dtype=np.uint8)
        out = resize_rgb(src, 5, 5)
        assert out.shape == (5, 5, 3)
        assert out.dtype == np.uint8

    def test_resize_preserves_uniform_color(self):
        src = np.full((20, 20, 3), 42, dtype=np.uint8)
        out = resize_rgb(src, 8, 8)
        assert (out == 42).all()

    def test_resize_upscales_too(self):
        src = np.full((4, 4, 3), 100, dtype=np.uint8)
        out = resize_rgb(src, 16, 16)
        assert out.shape == (16, 16, 3)


class TestLoadLabels:
    def test_one_per_line(self, tmp_path):
        f = tmp_path / 'labels.txt'
        f.write_text('cat\ndog\nbird\n')
        assert load_labels(str(f)) == {0: 'cat', 1: 'dog', 2: 'bird'}

    def test_blank_lines_skipped_but_indices_consume_position(self, tmp_path):
        # `enumerate` on the file object increments for every line including
        # blanks, but blanks are dropped from the result. Reflects current
        # behavior so the test fails loudly if we ever change it.
        f = tmp_path / 'labels.txt'
        f.write_text('cat\n\ndog\n')
        labels = load_labels(str(f))
        assert labels == {0: 'cat', 2: 'dog'}

    def test_strips_whitespace(self, tmp_path):
        f = tmp_path / 'labels.txt'
        f.write_text('  cat  \n  dog  \n')
        assert load_labels(str(f)) == {0: 'cat', 1: 'dog'}


class TestMapOutputTensors:
    @staticmethod
    def _od(shape):
        # Mimic the dict shape returned by tflite get_output_details().
        return {'shape': np.array(shape)}

    def test_efficientdet_lite_layout(self):
        # EfficientDet-Lite0 emits (1,25) scores, (1,25,4) boxes, (1,) count, (1,25) classes.
        details = [
            self._od((1, 25)),       # scores
            self._od((1, 25, 4)),    # boxes
            self._od((1,)),          # count
            self._od((1, 25)),       # classes
        ]
        boxes, scores, classes, count = map_output_tensors(details)
        assert boxes == 1
        assert scores == 0
        assert classes == 3
        assert count == 2

    def test_missing_boxes_raises(self):
        details = [self._od((1, 25)), self._od((1, 25)), self._od((1,))]
        with pytest.raises(ValueError):
            map_output_tensors(details)

    def test_missing_scores_raises(self):
        details = [self._od((1, 25, 4)), self._od((1,))]
        with pytest.raises(ValueError):
            map_output_tensors(details)
