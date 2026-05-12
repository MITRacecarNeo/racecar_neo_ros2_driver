"""
Coral EdgeTPU object-detection node.

Subscribes to /camera/forward, resizes each frame to the model's input shape,
runs inference on the USB EdgeTPU, and publishes
vision_msgs/Detection2DArray on /edgetpu/inference plus a per-second
diagnostic_msgs/DiagnosticArray on /diagnostics.

Numpy-only image path — no cv_bridge or cv2 dependency.
"""

import os
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

    Supports rgb8 / bgr8 encodings only — what gscam publishes after
    `videoconvert ! appsink caps=video/x-raw,format=RGB|BGR`.
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


def map_output_tensors(output_details):
    """
    Match an SSD-style model's output tensors to (boxes, scores, classes, count) indices.

    EfficientDet-Lite and other SSD-family models have four outputs but the
    order is not standardized. Identify them by shape:
      - boxes:   (1, N, 4)
      - scores:  (1, N)
      - classes: (1, N) — same shape as scores; whichever comes second
      - count:   (1,)
    """
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
        self.declare_parameter('image_topic', '/camera/forward')
        self.declare_parameter('diagnostics_period_sec', 1.0)
        self.declare_parameter('image_timeout_sec', 5.0)

        model_path = self.get_parameter('model_path').value
        labels_path = self.get_parameter('labels_path').value
        self._score_threshold = self.get_parameter('score_threshold').value
        self._max_detections = self.get_parameter('max_detections').value
        image_topic = self.get_parameter('image_topic').value
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

        # First load after a cold boot may fail while the Coral firmware enumerates
        # (1a6e:089a → 18d1:9302). Retry once after a short sleep.
        try:
            self._interpreter = make_interpreter(model_path)
        except ValueError:
            self.get_logger().warn(
                'First make_interpreter failed (Coral firmware load?); retrying'
            )
            time.sleep(1.5)
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
        self._tpu_ok = True

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
        self._last_image_time = self.get_clock().now()

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
