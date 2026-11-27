"""scripts/pty_job.py: one command in a pseudo-terminal for a page."""

import time

from conftest import load_script
import pytest


@pytest.fixture
def pj():
    return load_script('pty_job')


def wait(job, until, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = job.state()
        if until(state):
            return state
        time.sleep(0.05)
    raise AssertionError(job.state())


def test_prompt_answer_and_secret(pj):
    job = pj.PtyJob()
    script = (
        'echo start; read -p "Continue? [y/N] " a; echo got $a; '
        'read -s -p "[sudo] password for racecar: " p; echo; echo len ${#p}; exit 3'
    )
    assert job.start(['bash', '-c', script], 'demo')
    state = wait(job, lambda s: s['prompt'] is not None)
    assert state['prompt'] == {'text': 'Continue? [y/N] ', 'secret': False}
    assert not job.start(['true'], 'other')
    job.send('y')
    state = wait(job, lambda s: s['prompt'] is not None and s['prompt']['secret'])
    job.send('hunter2')
    state = wait(job, lambda s: not s['running'])
    assert state['code'] == 3 and 'got y' in state['lines'] and 'len 7' in state['lines']
    assert not any('hunter2' in line for line in state['lines'])
    assert state['label'] == 'demo'


def test_since_and_stop(pj):
    job = pj.PtyJob()
    job.start(['bash', '-c', 'echo one; echo two; sleep 30'], 'long')
    state = wait(job, lambda s: len(s['lines']) >= 2)
    assert job.state(since=1)['lines'] == ['two']
    job.stop()
    state = wait(job, lambda s: not s['running'])
    assert state['code'] != 0
    assert not job.send('late')


def test_colors_stripped_and_lines_capped(pj, monkeypatch):
    monkeypatch.setattr(pj, 'MAX_LINES', 5)
    job = pj.PtyJob()
    job.start(
        [
            'bash',
            '-c',
            'printf "\\033[32mgreen\\033[0m\\n"; for i in $(seq 1 9); do echo $i; done',
        ],
        'x',
    )
    state = wait(job, lambda s: not s['running'])
    assert state['lines'] == ['5', '6', '7', '8', '9'] and state['next'] == 10
    assert pj.clean('\x1b[1;31mred\x1b[0m') == 'red'
