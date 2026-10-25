"""
Command, timeout, and actuation tests for teleop.py.

Runs without a ROS graph: Node.__init__ and the create_* factories are
stubbed, so the real TeleopNode.__init__ still sets every field and the
timer callback is driven by hand with an explicit clock.

    source /opt/ros/jazzy/setup.bash
    source /home/racecar/ros2_ws/install/setup.bash
    pytest -q
"""

import importlib.util
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

BASE = Path(__file__).resolve().parents[1] / 'scripts' / 'dashboards' / 'teleop_dashboard'
_spec = importlib.util.spec_from_file_location('teleop', BASE / 'teleop.py')
tp = importlib.util.module_from_spec(_spec)
sys.modules['teleop'] = tp
_spec.loader.exec_module(tp)

SHIPPED = None  # yaml as committed, restored after any test that writes it


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(autouse=True)
def params(monkeypatch, tmp_path):
    """Fresh parameters per test; saves go to a temporary per-car file."""
    global SHIPPED
    if SHIPPED is None:
        SHIPPED = (BASE / 'teleop.yaml').read_text()
    monkeypatch.setattr(tp, 'YAML_PATH', tmp_path / 'teleop.local.yaml')
    monkeypatch.setattr(tp, 'LOG_DIR', tmp_path / 'logs')
    tp._params.clear()
    tp._marks.clear()
    tp._rows.clear()
    tp._hist.clear()
    tp._cmd.update(fwd=0, turn=0, stamp=0.0)
    tp.load_params()
    tp._params.update(speed=0.5, angle=0.8)
    yield tp._params


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode, 'create_subscription', lambda self, *a, **k: None, raising=False
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    return tp.TeleopNode()


def drive(node, fwd, turn, at=100.0, tick=None):
    """Post a command at time `at`, then run the timer at `tick` (default: same instant)."""
    tp.set_cmd({'fwd': fwd, 'turn': turn}, now=at)
    node._actuate(now=at if tick is None else tick)
    return node._pub.msgs[-1]


# ---- parameters ----


def test_speed_is_capped_both_ways():
    assert tp.clamp_speed(5.0) == tp.MAX_SPEED
    assert tp.clamp_speed(-5.0) == -tp.MAX_SPEED


def test_shipped_yaml_cannot_drive():
    import yaml

    assert yaml.safe_load(SHIPPED)['speed'] == 0.0


def test_save_keeps_comments(params):
    params.update(speed=0.35, angle=0.6)
    tp.save_params()
    text = tp.YAML_PATH.read_text()
    assert (BASE / 'teleop.yaml').read_text() == SHIPPED  # defaults untouched
    assert 'the buttons do' in text  # comments survive
    tp._params.clear()
    tp.load_params()
    assert tp._params['speed'] == 0.35
    assert tp._params['angle'] == 0.6
    assert tp._params['camera_topic'] == '/camera/color'


def test_update_clamps(params):
    tp.update_params({'speed': 4.0, 'angle': -1.0})
    assert params['speed'] == tp.MAX_SPEED
    assert params['angle'] == 0.0
    assert tp.update_params({'speed': 1.0}) == []
    assert tp.update_params({'speed': 0.2}) == ['speed 0.2']


# ---- command mapping ----


def test_forward_publishes_the_speed_slider(node, params):
    msg = drive(node, 1, 0)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == 0.0
    assert tp._state['fwd'] == 1 and tp._state['held']


def test_back_is_negative_speed(node):
    assert drive(node, -1, 0).drive.speed == pytest.approx(-0.5)


def test_left_turns_left_on_the_wire(node):
    # Student convention: left is negative; the wire is negated, so left is
    # positive on /drive (physically verified on the sibling dashboards).
    assert drive(node, 0, -1).drive.steering_angle == pytest.approx(0.8)
    assert drive(node, 0, 1).drive.steering_angle == pytest.approx(-0.8)


def test_forward_and_left_at_once(node):
    msg = drive(node, 1, -1)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == pytest.approx(0.8)
    assert (tp._state['fwd'], tp._state['turn']) == (1, -1)


def test_axes_are_squashed_to_unit(node):
    tp.set_cmd({'fwd': 7, 'turn': -3})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (1, -1)
    tp.set_cmd({})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (0, 0)


def test_release_stops(node):
    drive(node, 1, 1)
    msg = drive(node, 0, 0)
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert not tp._state['held']


def test_speed_zero_means_no_motion_but_steering(node, params):
    params['speed'] = 0.0
    msg = drive(node, 1, 1)
    assert msg.drive.speed == 0.0
    assert msg.drive.steering_angle == pytest.approx(-0.8)


# ---- timeout ----


def test_stale_command_drives_zero(node, params):
    params['cmd_timeout'] = 0.5
    msg = drive(node, 1, -1, at=100.0, tick=100.4)
    assert msg.drive.speed == pytest.approx(0.5)
    assert not tp._state['timed_out']
    node._actuate(now=100.6)
    msg = node._pub.msgs[-1]
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert tp._state['timed_out']
    assert not tp._state['held']


def test_a_fresh_command_clears_the_timeout(node, params):
    params['cmd_timeout'] = 0.5
    drive(node, 1, 0, at=100.0, tick=101.0)
    assert tp._state['timed_out']
    drive(node, 1, 0, at=101.0)
    assert not tp._state['timed_out']
    assert node._pub.msgs[-1].drive.speed == pytest.approx(0.5)


def test_a_released_command_never_reads_as_timed_out(node):
    drive(node, 0, 0, at=100.0, tick=200.0)
    assert not tp._state['timed_out']


# ---- sensors ----


def make_scan(points=1080, span_deg=360.0, fill=2.0):
    """Build a scan shaped like this car's: 1080 points over 360 degrees."""
    msg = LaserScan()
    msg.angle_min = -math.radians(span_deg) / 2
    msg.angle_max = math.radians(span_deg) / 2
    msg.angle_increment = math.radians(span_deg) / points
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [fill] * points
    return msg


def place(msg, raw_deg, value):
    """Put `value` at the ray nearest raw angle `raw_deg`; return its index."""
    i = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment) % len(msg.ranges)
    msg.ranges[i] = value
    return i


def test_scan_is_thinned(node):
    node._scan_cb(make_scan(points=360, span_deg=360.0))
    assert len(tp._state['scan']) == 120


def test_mount_yaw_puts_the_nose_at_the_raw_180_edge():
    # Measured on hardware: the Neo's RPLIDAR faces aft, so an object in
    # front reports at raw 180. The neoracer read 0 there.
    assert tp.LIDAR_MOUNT_YAW_DEG == 180.0


def test_object_in_front_reads_as_zero_degrees(node):
    msg = make_scan()
    i = place(msg, 180.0, 1.0)
    assert node._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


def test_object_off_the_right_reads_as_positive(node):
    # Measured on hardware: an object off the car's right sits at raw +90.
    msg = make_scan()
    i = place(msg, 90.0, 1.0)
    assert node._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)


def test_left_and_right_are_not_mirrored(node):
    """A handedness flip would put the whole view in a mirror."""
    msg = make_scan()
    right = place(msg, 90.0, 2.0)
    left = place(msg, -90.0, 3.0)
    assert node._student_deg(msg, right) == pytest.approx(90.0, abs=0.5)
    assert node._student_deg(msg, left) == pytest.approx(-90.0, abs=0.5)


def test_mapping_is_a_yaw_not_a_flip(node):
    # student_old = -raw, student_new = 180 - raw: a constant 180 apart, so
    # ordering around the circle is preserved. This is the whole bug.
    msg = make_scan()
    for raw in (-150, -90, -30, 0, 30, 90, 150):
        i = place(msg, raw, 1.0)
        assert ((node._student_deg(msg, i) - (-raw)) % 360) == pytest.approx(180.0, abs=0.5)


def test_270_degree_scan_still_maps(node):
    """Nothing here assumes a sweep width; the angle comes from the message."""
    msg = make_scan(points=811, span_deg=270.0)
    i = place(msg, 90.0, 2.0)
    assert node._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)


def test_forward_facing_lidar_needs_no_offset(node, monkeypatch):
    monkeypatch.setattr(tp, 'LIDAR_MOUNT_YAW_DEG', 0.0)
    msg = make_scan()
    i = place(msg, 0.0, 1.5)
    assert node._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


def test_camera_preview_is_encoded(node):
    img = np.full((480, 640, 3), 90, np.uint8)
    msg = Image()
    msg.encoding = 'jpeg'
    msg.height, msg.width = 480, 640
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    node._last_preview = 0.0
    node._image_cb(msg)
    assert tp._preview[:2] == b'\xff\xd8'
    assert tp._state['res'] == [640, 480]


def test_depth_preview_is_encoded(node):
    raw = np.full((480, 640), 1500, np.uint16)  # a flat wall at 1.5 m
    msg = Image()
    msg.encoding = '16UC1'
    msg.height, msg.width = 480, 640
    msg.data = raw.tobytes()
    node._last_depth_preview = 0.0
    node._depth_cb(msg)
    assert tp._depth_preview[:2] == b'\xff\xd8'
    assert tp._state['depth_res'] == [640, 480]


def test_depth_preview_takes_meters_too(node):
    msg = Image()
    msg.encoding = '32FC1'
    msg.height, msg.width = 480, 640
    msg.data = np.full((480, 640), 1.5, np.float32).tobytes()
    node._last_depth_preview = 0.0
    node._depth_cb(msg)
    assert tp._depth_preview[:2] == b'\xff\xd8'


def test_depth_ignores_an_encoding_it_cannot_read(node):
    tp._depth_preview = b''
    msg = Image()
    msg.encoding = 'bgr8'
    msg.height, msg.width = 480, 640
    msg.data = np.zeros((480, 640, 3), np.uint8).tobytes()
    node._last_depth_preview = 0.0
    node._depth_cb(msg)
    assert tp._depth_preview == b''


def test_a_colour_only_car_does_not_subscribe_to_depth(monkeypatch, params):
    """
    An empty depth_topic must not become a subscription.

    A dead topic would sit at 0 fps and read on the dashboard as a broken
    camera.
    """
    topics = []
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode,
        'create_subscription',
        lambda self, kind, topic, *a, **k: topics.append(topic),
        raising=False,
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    params['depth_topic'] = ''
    tp.TeleopNode()
    assert '/camera/depth' not in topics
    assert params['camera_topic'] in topics


def test_depth_ramp_orders_near_over_far():
    """
    Near must read brighter than far.

    A no-return pixel must be black rather than sharing the dark end with a
    distant wall.
    """
    frame = np.array([[0.0, 0.5, 3.9, 0.0]], np.float32)  # 0 = no return
    out = tp._colorize_depth(frame, 4.0)
    near, far = out[0][1].sum(), out[0][2].sum()
    assert near > far
    assert far > 0
    assert tuple(out[0][0]) == tp.NO_RETURN_BGR
    assert tuple(out[0][3]) == tp.NO_RETURN_BGR


def test_depth_beyond_the_ramp_stays_visible():
    """
    A wall past depth_max_m clamps to the dark end.

    It must not vanish into the no-return black.
    """
    out = tp._colorize_depth(np.array([[99.0]], np.float32), 4.0)
    assert tuple(out[0][0]) != tp.NO_RETURN_BGR


def make_detection(cx, cy, w, h, label='person', score=0.9):
    d = Detection2D()
    d.bbox = BoundingBox2D()
    d.bbox.center = Pose2D()
    d.bbox.center.position = Point2D(x=float(cx), y=float(cy))
    d.bbox.size_x, d.bbox.size_y = float(w), float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis = ObjectHypothesis()
    hyp.hypothesis.class_id = label
    hyp.hypothesis.score = float(score)
    d.results.append(hyp)
    return d


def det_array(*dets):
    msg = Detection2DArray()
    msg.detections.extend(dets)
    return msg


def test_detections_leave_as_fractions_of_the_frame(node):
    """Pixels in, fractions out: the browser scales them to its own preview."""
    tp._state['res'] = [640, 480]
    # Centred 320x240 box: a quarter in from each edge, half the frame wide.
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    x, y, w, h, label, score = tp._state['dets'][0]
    assert (x, y, w, h) == (0.25, 0.25, 0.5, 0.5)
    assert label == 'person'
    assert score == 0.9


def test_a_corner_detection_stays_inside_the_frame(node):
    tp._state['res'] = [640, 480]
    node._det_cb(det_array(make_detection(50, 40, 100, 80)))
    x, y, w, h = tp._state['dets'][0][:4]
    assert (x, y) == (0.0, 0.0)
    # Boxes ship rounded to 4 places: 0.0001 of a 368 px view is 0.04 px.
    assert w == pytest.approx(100 / 640, abs=5e-5)
    assert h == pytest.approx(80 / 480, abs=5e-5)


def test_detections_before_the_first_frame_are_dropped(node):
    """Without a frame size there is nothing to normalize against."""
    tp._state['res'] = [0, 0]
    tp._state['dets'] = []
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    assert tp._state['dets'] == []


def test_a_detection_with_no_hypothesis_still_draws(node):
    tp._state['res'] = [640, 480]
    d = Detection2D()
    d.bbox = BoundingBox2D()
    d.bbox.center = Pose2D()
    d.bbox.center.position = Point2D(x=320.0, y=240.0)
    d.bbox.size_x, d.bbox.size_y = 64.0, 48.0
    node._det_cb(det_array(d))
    assert tp._state['dets'][0][4:] == ['', 0.0]


def test_stale_detections_are_cleared(node):
    """A frozen box on a live frame would be a lie about what is out there."""
    tp._state['res'] = [640, 480]
    node._det_available = True
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    node._expire_detections(node._last_det + 0.1)
    assert tp._state['det_ok'] is True
    assert tp._state['dets']
    node._expire_detections(node._last_det + tp.DET_TIMEOUT_S + 0.1)
    assert tp._state['det_ok'] is False
    assert tp._state['dets'] == []


def test_a_car_with_no_detector_never_subscribes(monkeypatch, params):
    topics = []
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode,
        'create_subscription',
        lambda self, kind, topic, *a, **k: topics.append(topic),
        raising=False,
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    params['detections_topic'] = ''
    node = tp.TeleopNode()
    assert node._det_available is False
    assert '/edgetpu/inference' not in topics


def test_rows_and_history_record_the_command(node):
    node._enc = 1.25
    drive(node, 1, -1)
    assert tp._rows[-1].strip().split(',')[1:] == ['1', '-1', '0.500', '-0.800', '1.250']
    assert tp._hist[-1][1:] == [1.25, 0.5]


def test_thin_keeps_the_newest_row():
    rows = [[i, 0.0, 0.0] for i in range(1000)]
    sent = tp._thin(rows)
    assert len(sent) < len(rows)
    assert sent[-1] is rows[-1]


# ---- sensors, dot matrix, LED strip ----


def test_heading_is_zero_facing_north_and_counterclockwise_positive():
    assert tp.heading_deg([20.0, 0.0, -40.0]) == pytest.approx(0.0)
    # Field along -y in the body: north is to the car's right, so the car
    # faces 90 deg counterclockwise of north.
    assert tp.heading_deg([0.0, -20.0, -40.0]) == pytest.approx(90.0)


def test_matrix_pixels_are_192_zeros_and_ones():
    px = tp.matrix_pixels([5, 0, 255] + [1] * 300)
    assert len(px) == 192 and px[:3] == [1, 0, 1] and set(px) <= {0, 1}
    assert tp.matrix_pixels([]) == [0] * 192


def test_led_bytes_are_84_rgb_triples_clamped():
    data = tp.led_bytes([[300, -5, 10], [1, 2]])
    assert len(data) == 252 and data[:6] == bytes([255, 0, 10, 1, 2, 0])
    assert data[6:] == bytes(246)


def test_display_is_resent_only_while_the_page_polls(node):
    tp._display.update(kind=None, pixels=[], text='', leds=None)
    tp.set_display('text', 'hi')
    tp.set_display('leds', [[1, 2, 3]])
    tp._last_poll = 100.0
    node._send_display(100.5)
    assert node._pub_text.msgs[-1].data == 'hi'
    assert bytes(node._pub_leds.msgs[-1].data)[:3] == bytes([1, 2, 3])
    sent = len(node._pub_text.msgs), len(node._pub_leds.msgs)
    node._send_display(100.6)  # inside both re-send periods
    assert (len(node._pub_text.msgs), len(node._pub_leds.msgs)) == sent
    node._send_display(100.0 + tp.DISPLAY_HOLD_S + 1)  # the page went away
    assert (len(node._pub_text.msgs), len(node._pub_leds.msgs)) == sent


def test_release_stops_the_resends(node):
    tp.set_display('pixels', [1] * 192)
    tp.set_display('leds', [[9, 9, 9]])
    tp.set_display('release', None)
    tp._last_poll = 200.0
    node._send_display(200.1)
    assert node._pub_pixels.msgs == [] and node._pub_leds.msgs == []
