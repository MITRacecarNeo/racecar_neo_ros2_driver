"""nav_link.py: a front end's side of the navigator."""

import json
import threading
import time

import pytest
from racecar_neo_ros2_driver import nav_link

pytest.importorskip('std_msgs')


class FakePub:
    def __init__(self):
        self.sent = []
        self.on_publish = lambda raw: None

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))
        self.on_publish(self.sent[-1])


class FakeNode:
    def __init__(self):
        self.pub = FakePub()
        self.subs = {}

    def create_publisher(self, kind, topic, qos):
        assert topic == '/nav/command'
        return self.pub

    def create_subscription(self, kind, topic, callback, qos):
        self.subs[topic] = callback


@pytest.fixture
def node():
    return FakeNode()


def state(**more):
    return json.dumps({'state': 'running', 'version': 7, 'answers': {}, **more})


def test_idle_without_the_navigator(node):
    link = nav_link.NavLink(node)
    view = link.view()
    assert view['state'] == 'idle' and view['mission'] is None and not link.underway()
    assert link.command('start') == (False, 'Load a map first.') and node.pub.sent == []


def test_view_of_a_mission(node):
    link = nav_link.NavLink(node)
    link.on_state(state())
    link.on_mission(json.dumps({'version': 7, 'mission': {'loops': 1}}))
    view = link.view()
    assert view['state'] == 'running' and view['mission'] == {'loops': 1}
    assert view['version'] == 7 and 'answers' not in view and link.underway()


def test_the_mission_stays_when_the_navigator_goes(node):
    now = [0.0]
    link = nav_link.NavLink(node, lambda: now[0])
    link.on_state(state())
    link.on_mission(json.dumps({'version': 7, 'mission': {'loops': 1}}))
    now[0] = 3.0
    assert not link.alive and not link.underway()
    assert link.view()['state'] == 'idle' and link.view()['mission'] == {'loops': 1}
    link.on_mission(json.dumps({'version': 9, 'mission': None}))  # a new navigator
    assert link.view()['version'] == 7


def test_command_returns_the_answer(node):
    link = nav_link.NavLink(node)
    link.on_state(state(state='idle'))

    def answer(raw):
        reply = state(answers={raw['id']: [False, 'Planner is driving.']})
        threading.Timer(0.05, link.on_state, [reply]).start()

    node.pub.on_publish = answer
    assert link.command('start', name='x') == (False, 'Planner is driving.')
    assert node.pub.sent[0]['verb'] == 'start' and node.pub.sent[0]['name'] == 'x'


def test_command_without_a_spin_thread(node):
    link = nav_link.NavLink(node)
    link.on_state(state(state='idle'))

    def spin(sec):
        link.on_state(state(answers={node.pub.sent[0]['id']: [True, '']}))

    assert link.command('stop', wait=spin) == (True, '')


def test_a_command_is_sent_again_until_answered(node, monkeypatch):
    monkeypatch.setattr(nav_link, 'RESEND_SEC', 0.05)
    link = nav_link.NavLink(node)
    link.on_state(state(state='idle'))

    def hears_the_second(raw):
        if len(node.pub.sent) == 2:
            link.on_state(state(answers={raw['id']: [True, '']}))

    node.pub.on_publish = hears_the_second
    assert link.command('stop', wait=time.sleep) == (True, '')
    assert node.pub.sent[0]['id'] == node.pub.sent[1]['id']


def test_notify_waits_for_nothing(node):
    link = nav_link.NavLink(node)
    link.notify('alive', owner='program-1')
    assert node.pub.sent[0]['verb'] == 'alive' and node.pub.sent[0]['owner'] == 'program-1'


def test_no_answer(node, monkeypatch):
    monkeypatch.setattr(nav_link, 'ANSWER_SEC', 0.1)
    link = nav_link.NavLink(node)
    link.on_state(state(answers={'other': [True, '']}))
    began = time.monotonic()
    assert link.command('stop') == (False, nav_link.NO_ANSWER)
    assert time.monotonic() - began < 1.0


@pytest.mark.parametrize('text', ['', '[1]', '{', 'null'])
def test_a_broken_message_changes_nothing(node, text):
    link = nav_link.NavLink(node)
    link.on_state(text)
    link.on_mission(text)
    assert not link.alive


def test_idle_names_every_field_the_runner_sends():
    from racecar_neo_ros2_driver.nav_runner import Commander

    sent = set(Commander().view(0.0)) | {'ready'}
    assert sent == set(nav_link.IDLE)
