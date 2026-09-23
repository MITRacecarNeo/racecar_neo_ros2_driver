"""Unit tests for edgetpu_node pure helpers."""

from pathlib import Path
import re

import numpy as np
import pytest
from racecar_neo_ros2_driver.edgetpu_node import (
    EdgeTPUNode,
    image_msg_to_rgb,
    load_labels,
    map_output_tensors,
    resize_rgb,
    roles_from_names,
)
from sensor_msgs.msg import Image

MODELS = Path(__file__).resolve().parents[1] / 'models'


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
        # Single pixel: stored as B=255, G=0, R=0 -> should come back as RGB (0, 0, 255).
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
        # Blank lines are dropped but still advance the class index.
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
            self._od((1, 25)),  # scores
            self._od((1, 25, 4)),  # boxes
            self._od((1,)),  # count
            self._od((1, 25)),  # classes
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


class _Gate:
    """The rate gate alone, without a node, a TPU or a ROS graph."""

    def __init__(self, rate_hz):
        self._inference_period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._last_inference_at = 0.0
        self._frames_dropped = 0

    _rate_limited = EdgeTPUNode._rate_limited


class TestInferenceRateCap:
    def test_frames_inside_the_interval_are_dropped(self):
        # 15 Hz against a 60 fps camera: one frame in four is inferred.
        g = _Gate(15.0)
        kept = [t for t in range(24) if not g._rate_limited(now=100.0 + t / 60.0)]
        assert len(kept) == 6
        assert g._frames_dropped == 18

    def test_the_first_frame_is_never_dropped(self):
        g = _Gate(15.0)
        assert g._rate_limited(now=100.0) is False

    def test_the_interval_is_measured_from_the_last_inference(self):
        """Not from the last frame, or a fast camera would starve the gate."""
        g = _Gate(10.0)
        assert g._rate_limited(now=100.0) is False  # inferred
        assert g._rate_limited(now=100.05) is True  # too soon
        assert g._rate_limited(now=100.09) is True  # still too soon
        # 100.10 - 100.0 falls just short of 0.1 in binary float, so the
        # first frame past the interval is 100.11.
        assert g._rate_limited(now=100.11) is False  # a full interval on

    def test_zero_disables_the_cap(self):
        g = _Gate(0.0)
        assert [g._rate_limited(now=100.0 + t / 60.0) for t in range(5)] == [False] * 5
        assert g._frames_dropped == 0

    def test_a_slow_camera_is_never_gated(self):
        # 5 fps under a 15 Hz cap: every frame is already late.
        g = _Gate(15.0)
        assert [g._rate_limited(now=100.0 + t / 5.0) for t in range(5)] == [False] * 5


class TestShippedConfig:
    def test_the_shipped_config_caps_at_15(self):
        cfg = (MODELS.parent / 'config' / 'edgetpu.yaml').read_text()
        assert re.search(r'^\s*inference_rate_hz:\s*15\.0\s*$', cfg, re.M)

    def test_the_diagnostic_nominal_matches_the_cap(self):
        """A nominal above the cap would fail a healthy car."""
        cfg = (MODELS.parent / 'config' / 'edgetpu.yaml').read_text()
        rate = float(re.search(r'inference_rate_hz:\s*([\d.]+)', cfg).group(1))
        diag = (MODELS.parent / 'scripts' / 'diagnose.py').read_text()
        nominal = float(
            re.search(r"TopicSpec\('/edgetpu/inference',[^,]+,\s*([\d.]+)", diag).group(1)
        )
        assert nominal == rate

    def test_the_shipped_score_threshold(self):
        cfg = (MODELS.parent / 'config' / 'edgetpu.yaml').read_text()
        assert re.search(r'^\s*score_threshold:\s*0\.4\s*$', cfg, re.M)


def _named(shape, suffix):
    return {'shape': np.array(shape), 'name': f'StatefulPartitionedCall:{suffix}'}


class TestRolesFromNames:
    """
    The name suffix is what separates scores from classes.

    Both share the shape (1, N), so shape alone can only guess by position,
    and the two models this repository ships report them in opposite orders.
    """

    def test_names_beat_position(self):
        # The COCO model's order: boxes, classes, scores, count. Position
        # would call index 1 the scores; the name says it is the classes.
        details = [_named((1, 25, 4), 3), _named((1, 25), 2), _named((1, 25), 1), _named((1,), 0)]
        boxes, scores, classes, count = map_output_tensors(details)
        assert (boxes, scores, classes, count) == (0, 2, 1, 3)

    def test_shapes_must_agree_with_the_names(self):
        # A name claiming :3 on a tensor that is not (1, N, 4) is not
        # trustworthy, so the mapping falls back to shape.
        details = [_named((1, 25), 3), _named((1, 25), 2), _named((1, 25, 4), 1), _named((1,), 0)]
        assert roles_from_names(details) is None

    def test_unnamed_outputs_fall_back_to_shape(self):
        details = [
            {'shape': np.array((1, 25, 4))},
            {'shape': np.array((1, 25))},
            {'shape': np.array((1, 25))},
            {'shape': np.array((1,))},
        ]
        assert roles_from_names(details) is None
        assert map_output_tensors(details) == (0, 1, 2, 3)

    def test_duplicate_suffixes_are_rejected(self):
        details = [_named((1, 25, 4), 3), _named((1, 25), 1), _named((1, 25), 1), _named((1,), 0)]
        assert roles_from_names(details) is None


class TestShippedModels:
    """
    Pin the mapping against the real files, not a hand-written stub.

    Both models are EdgeTPU-compiled, so they cannot be run without the
    accelerator, but their output metadata reads on any machine.
    """

    @staticmethod
    def _details(name):
        tflite = pytest.importorskip('tflite_runtime.interpreter')
        path = MODELS / name
        if not path.is_file():
            pytest.skip(f'{name} not present')
        return tflite.Interpreter(model_path=str(path)).get_output_details()

    @pytest.mark.parametrize(
        'model',
        [
            'efficientdet_lite0_320_coco_edgetpu.tflite',
            'efficientdet_lite0_generic_edgetpu.tflite',
        ],
    )
    def test_scores_and_classes_are_not_swapped(self, model):
        details = self._details(model)
        _, scores, classes, _ = map_output_tensors(details)
        assert details[scores]['name'].endswith(':1')
        assert details[classes]['name'].endswith(':2')

    def test_coco_labels_cover_every_class_the_model_can_emit(self):
        labels = load_labels(str(MODELS / 'coco_labels.txt'))
        assert len(labels) == 90
        assert labels[0] == 'person'
        assert labels[2] == 'car'
