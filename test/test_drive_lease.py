"""racecar_neo_ros2_driver.drive_lease: one /drive holder at a time."""

import fcntl
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import pytest
from racecar_neo_ros2_driver import drive_lease as dl

ROOT = Path(__file__).resolve().parent.parent
# A holder in its own process: takes the lease, says so, then refreshes it at
# 20 Hz and says what each refresh returned.
TAKE = """
import sys, time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from racecar_neo_ros2_driver.drive_lease import DriveLease
lease = DriveLease(sys.argv[3], sys.argv[3], Path(sys.argv[2]))
"""
HOLD = TAKE + """
print(lease.take(), flush=True)
while True:
    time.sleep(0.05)
    print(lease.refresh(), flush=True)
"""


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def make(tmp_path, clock):
    return lambda owner, name=None: dl.DriveLease(owner, name or owner, tmp_path, clock)


@pytest.fixture
def spawn(tmp_path):
    procs = []

    def start(owner):
        p = subprocess.Popen(
            [sys.executable, '-c', HOLD, str(ROOT), str(tmp_path), owner],
            stdout=subprocess.PIPE,
            text=True,
        )
        procs.append(p)
        assert p.stdout.readline() == 'True\n'
        return p

    yield start
    for p in procs:
        p.kill()
        p.wait()


def until(cond, sec=4.0):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


class TestRules:
    def test_first_come_holds(self, make):
        a, b = make('a', 'Web Teleop'), make('b')
        assert a.take() and not b.take() and a.held and not b.held
        assert b.refused() == 'Web Teleop is driving.' and a.holder() is None

    def test_free_after_a_release(self, make):
        a, b = make('a'), make('b')
        a.take()
        a.release()
        assert b.take() and not a.take()

    def test_release_of_anothers_lease_does_nothing(self, make):
        a, b = make('a'), make('b')
        a.take()
        b.release()
        assert a.refresh()

    def test_a_stale_heartbeat_is_free(self, make, clock):
        a, b = make('a'), make('b')
        a.take()
        clock.now += dl.STALE_SEC
        assert not b.take()
        clock.now += 0.01
        assert b.take()

    def test_refresh_keeps_it(self, make, clock):
        a, b = make('a'), make('b')
        a.take()
        for _ in range(40):
            clock.now += 0.1
            assert a.refresh() and not b.take()

    def test_refresh_never_takes(self, make):
        assert not make('a').refresh()

    def test_a_holder_gone_too_long_has_lost_it_even_when_free(self, make, clock):
        a = make('a')
        a.take()
        clock.now += dl.STALE_SEC + 0.01
        assert not a.refresh() and not a.refresh() and not a.held
        assert a.take()

    def test_a_holder_replaced_stops_at_the_next_check(self, make, clock):
        a, b = make('a'), make('b')
        a.take()
        clock.now += dl.STALE_SEC + 0.01
        b.take()
        assert not a.refresh() and not a.hold()

    def test_replaced_within_the_refresh_interval(self, make, tmp_path):
        a = make('a')
        a.take()
        (tmp_path / dl.RECORD).unlink()
        assert not a.refresh()

    def test_one_owner_in_two_processes(self, make):
        bridge, navigator = make('nav', 'Navigation'), make('nav', 'Navigation')
        assert navigator.take() and bridge.take() and navigator.refresh()

    def test_following_another_process_of_the_owner(self, make, tmp_path, monkeypatch):
        navigator, bridge = make('nav'), make('nav')
        assert not bridge.follows()
        navigator.take()
        with monkeypatch.context() as m:
            m.setattr(dl.os, 'getpid', lambda: 999999)
            assert bridge.joined() and bridge.follows()
            bridge.release()
        assert navigator.refresh() and not navigator.joined()
        navigator.share = True
        navigator.hold()
        with monkeypatch.context() as m:
            m.setattr(dl.os, 'getpid', lambda: 999999)
            assert bridge.joined() and not bridge.follows()
        navigator.release()
        assert not bridge.follows()

    def test_the_bridge_follows_a_program_that_lets_the_navigator_drive(self, make):
        bridge, program = make('nav'), make('program', 'Program')
        program.take()
        assert not bridge.follows()
        program.nav = True
        program.refresh()
        assert bridge.follows()
        program.share = True
        program.refresh()
        assert not bridge.follows()

    def test_a_library_takes_the_car_from_the_watch_and_keeps_it(self, make, monkeypatch):
        watch = dl.ProgramWatch(make('program', 'Program'))
        watch.tick(present=True)  # 8080 sees the library's node before the library holds
        library = make('program-4242', 'Program')
        with monkeypatch.context() as m:
            m.setattr(dl.os, 'getpid', lambda: 4242)
            assert library.take()
        watch.tick(present=True)
        assert dl._read(library._where).owner == 'program-4242'
        assert not make('program-7', 'Program').take()  # another program still waits
        assert not make('webteleop').take()

    def test_the_watch_leaves_a_library_that_holds_itself(self, make, monkeypatch):
        watch = dl.ProgramWatch(make('program', 'Program'))
        library = make('program', 'Program')
        library.nav = True
        library.take()
        with monkeypatch.context() as m:
            m.setattr(dl.os, 'getpid', lambda: 999999)
            watch.tick(present=True)
            assert not watch.lease.held
        assert dl._read(library._where).nav

    def test_hold_takes_when_free(self, make):
        a, b = make('a'), make('b')
        assert a.hold() and a.hold() and not b.hold()
        a.release()
        assert b.hold()

    def test_who_drives(self, make, tmp_path, clock):
        assert dl.holder(tmp_path, clock.now) is None
        make('a', 'Planner').take()
        assert dl.holder(tmp_path, clock.now).name == 'Planner'
        assert dl.holder(tmp_path, clock.now + 2) is None
        assert dl.refused(None) == 'Something else is driving.'


class TestBadState:
    @pytest.mark.parametrize('text', ['', '{', '[]', '{"owner": "a"}', '{"pid": 1, "beat": "x"}'])
    def test_a_broken_record_is_free(self, make, tmp_path, text):
        (tmp_path / dl.RECORD).write_text(text)
        assert make('a').take()

    def test_a_heartbeat_from_the_future_is_free(self, make, clock):
        make('a').take()
        clock.now -= 10
        assert make('b').take()

    def test_without_the_folder_nothing_is_enforced(self, tmp_path):
        a, b = (dl.DriveLease(n, n, tmp_path / 'none') for n in 'ab')
        assert a.take() and b.take() and b.hold() and b.refresh()
        assert a.held and b.held
        a.release()
        assert a.held and a.holder() is None

    def test_a_stuck_lock_is_not_holding(self, make, tmp_path):
        a = make('a')
        with (tmp_path / dl.LOCK).open('w') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            start = time.monotonic()
            assert not a.take() and time.monotonic() - start < 1.0
        assert a.take()

    def test_folder_from_the_environment(self, monkeypatch, tmp_path):
        monkeypatch.delenv(dl.FOLDER_ENV, raising=False)
        assert dl.folder() == Path('/run/racecar')
        monkeypatch.setenv(dl.FOLDER_ENV, str(tmp_path))
        assert dl.folder() == tmp_path


class TestGate:
    def test_an_app_holds_it_while_it_runs(self, make):
        app, other = dl.DriveGate(make('a')), make('b')
        assert app.allow(False) and not other.take()
        assert app.allow(True) and app.allow(False) and not other.take()
        app.release()
        assert other.take()

    def test_a_panel_holds_it_only_while_used(self, make):
        panel, other = dl.DriveGate(make('a'), keep=False), make('b')
        assert not panel.allow(False) and other.take()
        other.release()
        assert panel.allow(True) and not other.take()
        panel.idle()
        assert other.take()

    def test_refused_while_moving_never_starts_by_itself(self, make):
        gate, other = dl.DriveGate(make('a')), make('b', 'lab_f.py')
        other.take()
        assert not gate.allow(True) and gate.refused == 'lab_f.py is driving.'
        other.release()
        assert not gate.allow(True) and not gate.allow(True) and gate.refused == ''
        assert gate.allow(False) and gate.allow(True)

    def test_cut_off_while_moving_stays_off_until_zero(self, make, clock):
        gate = dl.DriveGate(make('a'))
        assert gate.allow(True)
        clock.now += dl.STALE_SEC + 0.01  # suspended, or the machine stalled
        assert not gate.allow(True) and not gate.allow(True)
        assert gate.allow(False) and gate.allow(True)

    def test_a_quiet_source_may_start_again(self, make):
        gate, other = dl.DriveGate(make('a'), keep=False), make('b')
        other.take()
        assert not gate.allow(True)
        gate.idle()
        other.release()
        assert gate.allow(True)

    def test_idle_leaves_anothers_lease(self, make):
        gate, other = dl.DriveGate(make('a'), keep=False), make('b')
        other.take()
        gate.idle()
        assert other.refresh()


class TestEveryPublisher:
    def test_every_drive_publisher_asks_the_lease(self):
        # A new /drive publisher must be added here, with its gate.
        pattern = re.compile(r"create_publisher\(\s*AckermannDriveStamped,\s*'/drive'")
        found = {
            str(p.relative_to(ROOT))
            for d in ('scripts', 'racecar_neo_ros2_driver', 'launch')
            for p in (ROOT / d).rglob('*.py')
            if pattern.search(p.read_text())
        }
        retired = {p for p in found if re.search('eps_|pursuit_|smartfollow_', p)}
        labs = 'scripts/dashboards'
        assert found - retired == {
            'racecar_neo_ros2_driver/nav_bridge_node.py',
            'scripts/mapview.py',
            'scripts/calibration/calibrate_dynamics.py',
            f'{labs}/teleop_dashboard/teleop.py',
            f'{labs}/linefollow_dashboard/linefollow.py',
            f'{labs}/wallfollow_dashboard/wallfollow.py',
        }
        for path in found - retired:
            text = (ROOT / path).read_text()
            assert 'DriveGate(' in text or 'drive_gate' in text or 'DriveLease(' in text, path
            assert re.search(r'gate\.allow\(|lease\.refresh\(|lease\.follows\(', text), path


class TestProgramWatch:
    @pytest.fixture
    def watch(self, make):
        return dl.ProgramWatch(make('program', 'Program'))

    def test_holds_from_start_to_end_moving_or_not(self, watch, make, clock):
        other = make('webteleop')
        for _ in range(20):
            watch.tick(present=True)
            clock.now += 0.2
            assert not other.take() and other.refused() == 'Program is driving.'
        watch.tick(present=False)
        assert other.take()

    def test_a_holder_before_it_keeps_the_car_until_it_lets_go(self, watch, make):
        first = make('webteleop', 'Web Teleop')
        first.take()
        watch.tick(present=True)
        assert first.refresh() and not watch.lease.held
        first.release()
        watch.tick(present=True)
        assert watch.lease.held and not first.take()


class TestProgramFilter:
    @pytest.fixture
    def filt(self, tmp_path, clock):
        return dl.ProgramFilter(tmp_path, clock)

    def test_a_program_passes_while_the_car_is_free_or_its_own(self, filt, make, clock):
        assert filt.allows('base_link')
        make('program', 'Program').take()
        clock.now += 0.2
        assert filt.allows('base_link')

    def test_a_program_is_dropped_while_another_holds(self, filt, make, clock):
        other = make('linefollow', 'Line Follow')
        other.take()
        clock.now += 0.2
        assert not filt.allows('base_link') and filt.allows('') and filt.allows('map')
        other.release()
        assert not filt.allows('base_link')  # read again after PROGRAM_CHECK_SEC
        clock.now += 0.2
        assert filt.allows('base_link')

    def test_a_program_that_holds_itself_passes(self, filt, make, clock):
        make('program-4242', 'Program').take()
        clock.now += 0.2
        assert filt.allows('base_link')
        assert dl.is_program('program-4242') and dl.is_program('program')
        assert not dl.is_program('programs') and not dl.is_program('planner')

    def test_two_programs_do_not_share_the_car(self, make):
        first, second = make('program-1', 'Program'), make('program-2', 'Program')
        assert first.take() and not second.take()

    def test_without_the_folder_everything_passes(self, clock, tmp_path):
        assert dl.ProgramFilter(tmp_path / 'none', clock).allows('base_link')

    def test_every_driver_source_has_another_frame(self):
        # A /drive publisher stamped base_link would be taken for a program.
        for path in ROOT.rglob('*.py'):
            if 'test' in path.parts or 'build' in path.parts:
                continue
            text = path.read_text(errors='replace')
            stamped = re.search(r"frame_id\s*=\s*['\"]base_link['\"]", text)
            assert not stamped or '/drive' not in text, path


class TestProcesses:
    def test_a_killed_holder_frees_the_car(self, spawn, tmp_path):
        p, b = spawn('program'), dl.DriveLease('b', 'b', tmp_path)
        assert not b.take()
        p.kill()
        start = time.monotonic()
        assert until(b.take) and time.monotonic() - start < dl.STALE_SEC + 0.5

    def test_a_suspended_holder_frees_the_car_and_has_lost_it_when_resumed(self, spawn, tmp_path):
        p, b = spawn('program'), dl.DriveLease('b', 'b', tmp_path)
        p.send_signal(signal.SIGSTOP)
        assert until(b.take)
        b.release()  # free again: the resumed program must still not drive on
        p.send_signal(signal.SIGCONT)
        time.sleep(0.5)
        p.kill()
        said = p.stdout.read().split()
        assert said[-3:] == ['False'] * 3

    def test_a_holder_whose_forked_child_lives_on_frees_the_car(self, tmp_path):
        code = TAKE + 'import os\nlease.take()\nif os.fork() == 0: time.sleep(5)\n'
        subprocess.run([sys.executable, '-c', code, str(ROOT), str(tmp_path), 'a'], check=True)
        assert until(dl.DriveLease('b', 'b', tmp_path).take, 2.0)

    def test_many_at_once_one_wins(self, tmp_path):
        code = TAKE + 'time.sleep(max(0, float(sys.argv[4]) - time.monotonic()))\n'
        code += 'print(lease.take())\n'
        at = str(time.monotonic() + 1.5)
        procs = [
            subprocess.Popen(
                [sys.executable, '-c', code, str(ROOT), str(tmp_path), f'p{i}', at],
                stdout=subprocess.PIPE,
                text=True,
            )
            for i in range(8)
        ]
        assert sorted(p.communicate()[0].strip() for p in procs) == ['False'] * 7 + ['True']


def test_the_lock_is_not_inherited():
    # A program started by a holder must not keep the lock open.
    assert os.O_CLOEXEC and 'O_CLOEXEC' in Path(dl.__file__).read_text()
