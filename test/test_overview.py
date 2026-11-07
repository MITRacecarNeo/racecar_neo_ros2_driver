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
    feed.touch_cameras()
    assert {'/camera/color', '/camera/depth'} <= {s.topic for s in node.subs}
    feed.cameras.last -= ov.CAMERA_IDLE_SEC + 1
    with feed._lock:
        assert feed.gate.check() and not feed.cameras.check()
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and '/scan' in topics
