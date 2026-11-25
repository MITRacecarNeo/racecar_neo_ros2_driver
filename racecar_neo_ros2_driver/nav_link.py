"""
A front end's side of the navigator (nav_server_node.py).

    link = NavLink(node)
    ok, message = link.command('start', owner='planner', name='Planner', mission=mission)
    link.view()['state']

Topics, all std_msgs/String with JSON:

    /nav/command   a command (nav_runner.py) with an id
    /nav/state     latched, 5 times a second: the mission's state and the answers
                   to the last commands by id
    /nav/mission   latched: the mission and its version, when it changes
    /nav/event     what happened, with the key the front end asked under (log)

command() blocks until the answer, so it is not for a ROS callback.
"""

from __future__ import annotations

from collections.abc import Callable
import json
import threading
import time
from typing import Any
import uuid

from .nav_runner import ACTIVE, NO_MAP

COMMAND_TOPIC = '/nav/command'
STATE_TOPIC = '/nav/state'
MISSION_TOPIC = '/nav/mission'
EVENT_TOPIC = '/nav/event'
NO_ANSWER = 'The navigator did not answer.'
STALE_SEC = 2.0
ANSWER_SEC = 2.0
RESEND_SEC = 0.5  # the navigator takes a command once, by its id
# A recorder whose subscription connects late still gets the first events.
EVENTS_KEPT = 200
IDLE: dict[str, Any] = {
    'state': 'idle', 'reason': '', 'index': 0, 'loop': 1, 'loops': 1, 'count': 0, 'left': None,
    'recoveries': 0, 'stalled': False, 'next': None, 'visited': [], 'dwell_left': None,
    'version': 0, 'owner': '', 'name': '', 'message': '', 'ready': False,
}  # fmt: skip


def event_qos() -> Any:
    from rclpy.qos import DurabilityPolicy, QoSProfile

    return QoSProfile(depth=EVENTS_KEPT, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def decode(text: str) -> dict[str, Any] | None:
    try:
        out = json.loads(text)
    except ValueError:
        return None
    return out if isinstance(out, dict) else None


class NavLink:
    def __init__(self, node: Any, clock: Callable[[], float] = time.monotonic) -> None:
        from std_msgs.msg import String

        from .qos import LATCHED

        self._msg = String
        self._clock = clock
        self._changed = threading.Condition()
        self._state: dict[str, Any] = {}
        self._at = -1e9
        self._mission: dict[str, Any] = {}
        self._pub = node.create_publisher(String, COMMAND_TOPIC, 10)
        node.create_subscription(String, STATE_TOPIC, lambda m: self.on_state(m.data), LATCHED)
        node.create_subscription(
            String, MISSION_TOPIC, lambda m: self.on_mission(m.data), LATCHED
        )

    def on_state(self, text: str) -> None:
        found = decode(text)
        if found is not None:
            with self._changed:
                self._state, self._at = found, self._clock()
                self._changed.notify_all()

    def on_mission(self, text: str) -> None:
        found = decode(text)
        if found is not None and found.get('mission') is not None:
            with self._changed:
                self._mission = found

    @property
    def alive(self) -> bool:
        return self._clock() - self._at <= STALE_SEC

    def view(self) -> dict[str, Any]:
        """Return the state (idle without the navigator) and the last mission it ran."""
        with self._changed:
            state = dict(self._state) if self.alive else {}
            mission = {k: self._mission.get(k) for k in ('mission', 'version') if self._mission}
        state.pop('answers', None)
        return {**IDLE, **state, 'mission': None, **mission}

    def underway(self) -> bool:
        return self.view()['state'] in (*ACTIVE, 'paused')

    def command(
        self, verb: str, wait: Callable[[float], Any] | None = None, **fields: Any
    ) -> tuple[bool, str]:
        """
        Send a command and return (accepted, message).

        `wait(sec)` lets a program without its own spin thread spin meanwhile.
        """
        if not self.alive:
            return False, NO_MAP
        key = uuid.uuid4().hex
        text = json.dumps({'id': key, 'verb': verb, **fields})
        sent = self._clock()
        end = sent + ANSWER_SEC
        self._pub.publish(self._msg(data=text))
        while self._clock() < end:
            if self._clock() - sent >= RESEND_SEC:
                # A new program's first command can go out before the navigator listens to it.
                sent = self._clock()
                self._pub.publish(self._msg(data=text))
            with self._changed:
                answer = (self._state.get('answers') or {}).get(key)
                if answer is None and wait is None:
                    self._changed.wait(0.1)
            if answer is not None:
                return bool(answer[0]), str(answer[1])
            if wait is not None:
                wait(0.05)
        return False, NO_ANSWER
