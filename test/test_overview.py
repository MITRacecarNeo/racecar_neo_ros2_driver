"""Overview feeds for the web dashboard: scripts/overview.py."""

import math
from types import SimpleNamespace

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def ov():
    return load_script('overview')


class FakeNode:
    def __init__(self):
        self.subs = []

    def create_subscription(self, msg_type, topic, cb, qos, raw=False):
        assert raw, 'overview feeds must not deserialize every message'
        handle = SimpleNamespace(topic=topic, cb=cb)
        self.subs.append(handle)
        return handle

    def destroy_subscription(self, handle):
        self.subs.remove(handle)


def test_scan_points_in_base_link(ov):
    # Lidar 0.2 m ahead, facing aft: its 0 rad beam points backward.
    pts = ov.scan_points([1.0, float('inf'), 0.0], 0.0, 0.1, (0.2, 0.0, math.pi))
    assert pts == [[-0.8, 0.0]]


def test_scan_points_drop_far_returns(ov):
    assert ov.scan_points([ov.SCAN_MAX_M + 1.0], 0.0, 0.1, (0.0, 0.0, 0.0)) == []


def test_subscriptions_open_on_request_and_close_when_idle(ov, monkeypatch):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: data, config_dir=None)
    assert node.subs == []
    feed.touch()
    cameras = {ov.FEEDS[n][0] for n in ov.CAMERAS}
    assert sorted(s.topic for s in node.subs) == sorted(
        t for t, _ in ov.FEEDS.values() if t not in cameras
    )
    feed.gate.last -= ov.IDLE_SEC + 1
    with feed._lock:
        assert not feed.gate.check()
    assert node.subs == [] and not feed.active


def fake_message(data, cls):
    zero = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(data=0.0, angular_velocity=zero)


def test_state_reports_feed_ages(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=fake_message, config_dir=None)
    feed.touch()
    next(s for s in node.subs if s.topic == '/encoder/speed').cb(b'x')
    state = feed.state()
    assert 'encoder' in state['ages'] and state['scan'] == []
    assert state['graph_sec'] == ov.GRAPH_SEC


def test_cameras_open_only_when_frames_are_requested(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: data, config_dir=None)
    feed.touch()
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and '/camera/depth' not in topics and '/scan' in topics
    for name in ov.CAMERAS:
        feed.touch_camera(name)
    assert {'/camera/color', '/camera/depth'} <= {s.topic for s in node.subs}
    feed.cameras['color'].last -= ov.CAMERA_IDLE_SEC + 1
    with feed._lock:
        assert feed.gate.check() and not feed.cameras['color'].check()
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and {'/scan', '/camera/depth'} <= topics


def test_a_color_panel_opens_only_the_color_camera(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: data, config_dir=None)
    feed.touch_camera('color')
    assert [s.topic for s in node.subs] == ['/camera/color'] and not feed.active
    # The Overview section closing leaves the panel's feed open.
    feed.touch()
    feed.gate.last -= ov.IDLE_SEC + 1
    with feed._lock:
        feed.gate.check()
    assert [s.topic for s in node.subs] == ['/camera/color']


def test_jpeg_by_width_and_never_stale(ov):
    frame = SimpleNamespace(
        height=48, width=64, encoding='rgb8', step=192, is_bigendian=0, data=bytes(48 * 64 * 3)
    )
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: frame, config_dir=None)
    assert feed.jpeg('color') is None
    feed.touch_camera('color')
    node.subs[0].cb(b'x')
    small, large = feed.jpeg('color'), feed.panel_jpeg()
    assert small[:2] == large[:2] == b'\xff\xd8'
    assert set(feed._jpeg) == {('color', ov.PREVIEW_WIDTH), ('color', ov.PANEL_WIDTH)}
    data, at = feed._latest['color']
    feed._latest['color'] = (data, at - ov.STALE_SEC - 0.1)
    assert feed.jpeg('color') is None and feed.panel_jpeg() is None
    feed.cameras['color'].last -= ov.CAMERA_IDLE_SEC + 1
    with feed._lock:
        feed.cameras['color'].check()
    assert feed._jpeg == {} and node.subs == []
