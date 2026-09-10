"""
Coral EdgeTPU object-detection node.

Subscribes to the colour camera topic, resizes each frame to the model's input shape,
runs inference on the USB EdgeTPU, and publishes
vision_msgs/Detection2DArray on /edgetpu/inference plus a per-second
diagnostic_msgs/DiagnosticArray on /diagnostics.

Inference is rate limited by inference_rate_hz, independently of the camera.
The colour stream runs at 60 fps and nothing downstream consumes detections
faster than the dashboards draw them, so inferring on every frame spent TPU
time and current for results nobody read. Frames above the rate are dropped
before the decode, which is where the cost is.

Numpy-only image path — no cv_bridge or cv2 dependency.
"""

import os
import re
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
from PIL import Image as PILImage
from pycoral.utils.edgetpu import list_edge_tpus, make_interpreter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)


def load_labels(path: str) -> dict:
    """Parse a one-class-per-line labels file into {index: name}."""
    labels = {}
    with open(path) as f:
        for i, line in enumerate(f):
            line = line.strip()
            if line:
                labels[i] = line
    return labels


def image_msg_to_rgb(msg: Image) -> np.ndarray:
    """
    Decode a sensor_msgs/Image into an (H, W, 3) uint8 RGB array.

    Supports rgb8 / bgr8 encodings only; the RealSense color stream on
    /camera/color publishes rgb8.
    """
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]
    return arr


def resize_rgb(rgb: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Bilinear-resize an (H, W, 3) uint8 array via PIL — Coral expects uint8."""
    pil = PILImage.fromarray(rgb, mode='RGB')
    return np.asarray(pil.resize((target_w, target_h), PILImage.BILINEAR))


# TFLite's detection postprocess emits four outputs in a fixed order, boxes,
# classes, scores, count, but get_output_details() does not report them in
# that order and two of them share the shape (1, N). The export encodes the
# true order in the tensor name suffix, reversed, so :3 is boxes and :0 is
# count. This is the same order pycoral's detect.get_objects() assumes.
_ROLE_BY_NAME_SUFFIX = {3: 'boxes', 2: 'classes', 1: 'scores', 0: 'count'}


def roles_from_names(output_details):
    """
    Match output tensors to roles by their name suffix.

    Returns (boxes, scores, classes, count) indices, or None when the names
    do not parse or the shapes disagree with the roles they name.
    """
    roles = {}
    for i, od in enumerate(output_details):
        m = re.search(r':(\d+)$', od.get('name', ''))
        if not m:
            return None
        role = _ROLE_BY_NAME_SUFFIX.get(int(m.group(1)))
        if role is None or role in roles:
            return None
        roles[role] = i
    if set(roles) != set(_ROLE_BY_NAME_SUFFIX.values()):
        return None
    # A name is only worth trusting when the shape agrees with it.
    if tuple(output_details[roles['boxes']]['shape'])[-1:] != (4,):
        return None
    if tuple(output_details[roles['count']]['shape']) != (1,):
        return None
    if len(tuple(output_details[roles['scores']]['shape'])) != 2:
        return None
    if len(tuple(output_details[roles['classes']]['shape'])) != 2:
        return None
    return roles['boxes'], roles['scores'], roles['classes'], roles['count']


def map_output_tensors(output_details):
    """
    Match an SSD-style model's output tensors to (boxes, scores, classes, count) indices.

    EfficientDet-Lite and other SSD-family models have four outputs but
    get_output_details() does not order them consistently: this repository's
    two models report them in different orders. Prefer the name suffix, which
    distinguishes scores from classes; fall back to shape when a model does
    not carry the standard names, where the two (1, N) tensors can only be
    told apart by position:
      - boxes:   (1, N, 4)
      - scores:  (1, N)
      - classes: (1, N), same shape as scores; whichever comes second
      - count:   (1,)
    """
    named = roles_from_names(output_details)
    if named is not None:
        return named

    idx_boxes = idx_scores = idx_classes = idx_count = None
    for i, od in enumerate(output_details):
        shape = tuple(od['shape'])
        if len(shape) == 3 and shape[-1] == 4:
            idx_boxes = i
        elif shape == (1,):
            idx_count = i
        elif len(shape) == 2 and idx_scores is None:
            idx_scores = i
        elif len(shape) == 2:
            idx_classes = i
    if idx_boxes is None or idx_scores is None:
        raise ValueError(
            f'Cannot identify SSD output layout: '
            f'{[tuple(od["shape"]) for od in output_details]}'
        )
    return idx_boxes, idx_scores, idx_classes, idx_count


class EdgeTPUNode(Node):
    def __init__(self):
        super().__init__('edgetpu_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('labels_path', '')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('max_detections', 0)
        self.declare_parameter('image_topic', '/camera/color')
        self.declare_parameter('inference_rate_hz', 15.0)
        self.declare_parameter('diagnostics_period_sec', 1.0)
        self.declare_parameter('image_timeout_sec', 5.0)

        model_path = self.get_parameter('model_path').value
        labels_path = self.get_parameter('labels_path').value
        self._score_threshold = self.get_parameter('score_threshold').value
        self._max_detections = self.get_parameter('max_detections').value
        image_topic = self.get_parameter('image_topic').value
        rate_hz = float(self.get_parameter('inference_rate_hz').value)
        # 0 disables the cap and infers on every frame.
        self._inference_period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        diag_period = self.get_parameter('diagnostics_period_sec').value
        self._image_timeout = self.get_parameter('image_timeout_sec').value

        if not model_path:
            self.get_logger().fatal('model_path parameter is required')
            raise SystemExit(1)

        pkg_share = get_package_share_directory('racecar_neo_ros2_driver')
        if not os.path.isabs(model_path):
            model_path = os.path.join(pkg_share, model_path)
        if labels_path and not os.path.isabs(labels_path):
            labels_path = os.path.join(pkg_share, labels_path)

        self._labels = {}
        if labels_path:
            try:
                self._labels = load_labels(labels_path)
            except FileNotFoundError:
                self.get_logger().warn(f'Labels file not found: {labels_path}')

        tpus = list_edge_tpus()
        if not tpus:
            self.get_logger().fatal('No EdgeTPU device detected')
            raise SystemExit(1)
        self.get_logger().info(
            f'EdgeTPU found: {tpus[0]["type"]} at {tpus[0]["path"]}'
        )

        # The M.2 Apex is bound at boot and loads on the first try; no USB
        # firmware-enumeration retry needed.
        self._interpreter = make_interpreter(model_path)
        self._interpreter.allocate_tensors()

        self._input_details = self._interpreter.get_input_details()[0]
        self._output_details = self._interpreter.get_output_details()
        _, self._model_h, self._model_w, _ = self._input_details['shape']

        (
            self._idx_boxes,
            self._idx_scores,
            self._idx_classes,
            self._idx_count,
        ) = map_output_tensors(self._output_details)

        self.get_logger().info(
            f'Model loaded: {os.path.basename(model_path)} '
            f'(input {self._model_w}x{self._model_h}, '
            f'{len(self._labels)} labels, threshold {self._score_threshold})'
        )

        self._inference_count = 0
        self._detection_count = 0
        self._last_inference_ms = 0.0
        self._avg_inference_ms = 0.0
        self._last_image_time = None
        self._last_inference_at = 0.0
        self._frames_dropped = 0
        self._tpu_ok = True
        self._scores_checked = False

        self._det_pub = self.create_publisher(
            Detection2DArray, '/edgetpu/inference', 10
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.create_subscription(
            Image, image_topic, self._image_cb, qos_profile_sensor_data
        )
        self.create_timer(diag_period, self._publish_diagnostics)

        self.get_logger().info(
            f'Subscribed to {image_topic}, publishing /edgetpu/inference'
        )

    def _image_cb(self, msg: Image):
        # Stamped for every frame, before the rate gate: the stale-input
        # watchdog is asking whether the camera is alive, not whether this
        # node chose to infer.
        self._last_image_time = self.get_clock().now()

        if self._rate_limited():
            return

        try:
            rgb = image_msg_to_rgb(msg)
        except ValueError as e:
            self.get_logger().warn(f'Image decode failed: {e}')
            return

        img_h, img_w = rgb.shape[:2]
        resized = resize_rgb(rgb, self._model_w, self._model_h)
        input_tensor = np.expand_dims(resized, axis=0)

        try:
            self._interpreter.set_tensor(self._input_details['index'], input_tensor)
            t0 = time.monotonic()
            self._interpreter.invoke()
            elapsed_ms = (time.monotonic() - t0) * 1000
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'EdgeTPU inference failed: {e}')
            self._tpu_ok = False
            return

        self._tpu_ok = True
        self._last_inference_ms = elapsed_ms
        self._inference_count += 1
        if self._avg_inference_ms == 0.0:
            self._avg_inference_ms = elapsed_ms
        else:
            self._avg_inference_ms = 0.9 * self._avg_inference_ms + 0.1 * elapsed_ms

        scores = self._interpreter.get_tensor(
            self._output_details[self._idx_scores]['index']
        ).flatten()
        self._check_scores_look_like_scores(scores)
        boxes = self._interpreter.get_tensor(
            self._output_details[self._idx_boxes]['index']
        ).reshape(-1, 4)

        if self._idx_classes is not None:
            classes = self._interpreter.get_tensor(
                self._output_details[self._idx_classes]['index']
            ).flatten().astype(int)
        else:
            classes = np.zeros(len(scores), dtype=int)

        if self._idx_count is not None:
            count = int(self._interpreter.get_tensor(
                self._output_details[self._idx_count]['index']
            ).flatten()[0])
        else:
            count = len(scores)

        det_array = Detection2DArray()
        det_array.header = msg.header

        max_det = self._max_detections if self._max_detections > 0 else count
        n_det = 0
        for i in range(min(count, len(scores))):
            if scores[i] < self._score_threshold:
                continue
            if n_det >= max_det:
                break

            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis()
            class_id = int(classes[i])
            hyp.hypothesis.class_id = self._labels.get(class_id, str(class_id))
            hyp.hypothesis.score = float(scores[i])
            det.results.append(hyp)

            ymin, xmin, ymax, xmax = boxes[i]
            cx = float((xmin + xmax) / 2.0 * img_w)
            cy = float((ymin + ymax) / 2.0 * img_h)
            w = float((xmax - xmin) * img_w)
            h = float((ymax - ymin) * img_h)
            det.bbox = BoundingBox2D()
            det.bbox.center = Pose2D()
            det.bbox.center.position = Point2D(x=cx, y=cy)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = w
            det.bbox.size_y = h

            det_array.detections.append(det)
            n_det += 1

        self._detection_count += n_det
        self._det_pub.publish(det_array)

    def _rate_limited(self, now=None):
        """
        Report whether this frame falls inside the inference interval.

        Counts the drop and returns True when it does. Called before the
        decode, so a dropped frame costs nothing but the callback.
        """
        if not self._inference_period:
            return False
        now = time.monotonic() if now is None else now
        if now - self._last_inference_at < self._inference_period:
            self._frames_dropped += 1
            return True
        self._last_inference_at = now
        return False

    def _check_scores_look_like_scores(self, scores):
        """
        Warn once if the scores tensor does not hold confidences.

        Confidences are in [0, 1]; class ids are not. A model whose outputs
        defeat both mappings would otherwise threshold on the class id and
        label every box with its confidence, silently.
        """
        if self._scores_checked or not len(scores):
            return
        self._scores_checked = True
        if float(scores.max()) > 1.0 or float(scores.min()) < 0.0:
            self.get_logger().error(
                'Scores tensor holds values outside [0, 1] '
                f'(min {scores.min():.3f}, max {scores.max():.3f}); scores and '
                'classes are probably swapped for this model. Check the output '
                'tensor names against map_output_tensors().'
            )

    def _publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'EdgeTPU Inference'
        status.hardware_id = 'coral_edgetpu_usb'

        if not self._tpu_ok:
            status.level = DiagnosticStatus.ERROR
            status.message = 'EdgeTPU inference failed'
        elif self._last_image_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'No images received yet'
        else:
            age = (self.get_clock().now() - self._last_image_time).nanoseconds / 1e9
            if age > self._image_timeout:
                status.level = DiagnosticStatus.WARN
                status.message = f'No image for {age:.1f}s'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'Running ({self._avg_inference_ms:.1f} ms avg)'

        status.values = [
            KeyValue(key='inference_count', value=str(self._inference_count)),
            KeyValue(key='detection_count', value=str(self._detection_count)),
            KeyValue(key='last_inference_ms', value=f'{self._last_inference_ms:.1f}'),
            KeyValue(key='avg_inference_ms', value=f'{self._avg_inference_ms:.1f}'),
            KeyValue(key='model_input', value=f'{self._model_w}x{self._model_h}'),
            KeyValue(key='score_threshold', value=str(self._score_threshold)),
            KeyValue(key='inference_rate_hz',
                     value=f'{1.0 / self._inference_period:.1f}'
                           if self._inference_period else 'uncapped'),
            KeyValue(key='frames_dropped', value=str(self._frames_dropped)),
            KeyValue(key='tpu_ok', value=str(self._tpu_ok)),
        ]
        msg.status.append(status)
        self._diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeTPUNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
