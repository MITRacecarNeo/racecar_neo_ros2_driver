"""nav_runner.py: the mission runner and the navigator's commands."""

import math

import pytest
from racecar_neo_ros2_driver import missions, nav_runner
from racecar_neo_ros2_driver.drive_lease import DriveGate, DriveLease, holder


def wp(x, y, heading=None, radius=0.25, speed=None, dwell=0):
    return {'x': x, 'y': y, 'heading_deg': heading, 'radius_m': radius,
            'speed_mps': speed, 'dwell_sec': dwell}  # fmt: skip


def mission(*points, loops=1):
    return missions.parse({'loops': loops, 'waypoints': list(points)})


def sends(actions):
    return [a for a in actions if a[0] == 'send']


class TestRun:
    def test_start_sends_the_first_waypoint_with_its_speed(self):
        run = nav_runner.Run()
        actions = run.start(mission(wp(2, 0, speed=0.6), wp(2, 2)), (0.0, 0.0, 0.0))
        assert actions[0] == ('limit', 0.6)
        kind, _, x, y, yaw = actions[1]
        assert (kind, x, y) == ('send', 2.0, 0.0) and yaw == pytest.approx(0.0)
        assert run.view(0)['state'] == 'running'

    def test_any_heading_faces_the_way_it_is_approached(self):
        run = nav_runner.Run()
        run.start(mission(wp(0, 0), wp(0, 3)), (0.0, -1.0, 0.0))
        actions = run.tick(1.0, (0.1, 0.0, 0.0), True)
        assert sends(actions)[0][4] == pytest.approx(math.pi / 2)

    def test_reached_within_radius_preempts_with_the_next(self):
        run = nav_runner.Run()
        run.start(mission(wp(2, 0, radius=1.0), wp(4, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (0.5, 0.0, 0.0), True) == []
        actions = run.tick(1.1, (1.2, 0.0, 0.0), True)
        assert ('cancel',) not in actions and sends(actions)[0][2:4] == (4.0, 0.0)
        assert run.view(0)['index'] == 1

    def test_heading_must_match(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, heading=90), wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == []
        assert sends(run.tick(1.1, (1.0, 0.0, math.radians(80)), True))

    def test_last_waypoint_arrives_and_lifts_the_limit(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, speed=0.5)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'arrived'

    def test_dwell_stops_then_goes_on(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, dwell=5), wp(2, 0)), (0.0, 0.0, 0.0))
        assert run.tick(10.0, (1.0, 0.0, 0.0), True) == [('cancel',)]
        assert run.view(12.0)['dwell_left'] == 3.0
        assert run.tick(14.0, (1.0, 0.0, 0.0), True) == []
        assert sends(run.tick(15.0, (1.0, 0.0, 0.0), True))[0][2] == 2.0

    def test_loops_return_to_the_first(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(2, 0), loops=2), (0.0, 0.0, 0.0))
        run.tick(1, (1.0, 0.0, 0.0), True)
        actions = run.tick(2, (2.0, 0.0, 0.0), True)
        assert sends(actions)[0][2] == 1.0 and run.view(0)['loop'] == 2
        run.tick(3, (1.0, 0.0, 0.0), True)
        run.tick(4, (2.0, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'arrived'

    def test_until_stop_keeps_looping(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(2, 0), loops=0), (0.0, 0.0, 0.0))
        for i in range(10):
            run.tick(i, (1.0 + i % 2, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'running' and run.view(0)['loop'] == 6

    def test_nav2_success_near_the_waypoint_counts(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, heading=0), wp(2, 0)), (0.0, 0.0, 0.0))
        # Outside the radius and heading, inside the margins.
        number, near = run.number, (0.65, 0.0, 0.6)
        assert run.tick(0.9, near, True) == []
        assert run.result(number - 1, 'succeeded', 1.0, near) == []
        assert sends(run.result(number, 'succeeded', 1.0, near))[0][2] == 2.0
        assert run.view(0)['index'] == 1

    @pytest.mark.parametrize(
        'pose',
        [(0.55, 0.0, 0.0), (1.0, 0.0, math.pi), None],
        ids=['away', 'facing away', 'no pose'],
    )
    def test_nav2_success_away_from_the_waypoint_sends_it_again(self, pose):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, heading=0), wp(2, 0)), (0.0, 0.0, 0.0))
        number = run.number
        again = sends(run.result(number, 'succeeded', 1.0, pose))
        assert again[0][1] == number + 1 and again[0][2] == 1.0
        assert run.view(0)['state'] == 'running' and run.view(0)['index'] == 0

    def test_hand_off_success_does_not_skip_the_last_waypoint(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        here = (0.9, 0.0, 0.0)
        run.tick(1.0, here, True)
        assert sends(run.result(run.number, 'succeeded', 1.05, here))[0][2] == 3.0
        assert run.view(0)['state'] == 'running'
        run.tick(3.0, (2.9, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'arrived'

    def test_hand_off_success_does_not_skip_in_a_loop(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(3, 0), loops=2), (0.0, 0.0, 0.0))
        for x in (1.0, 3.0, 1.0, 3.0):
            here = (x, 0.0, 0.0)
            run.tick(1.0, here, True)
            if run.view(0)['state'] == 'running':
                before = run.view(0)['index'], run.view(0)['loop']
                run.result(run.number, 'succeeded', 1.1, here)
                assert (run.view(0)['index'], run.view(0)['loop']) == before
        assert run.view(0)['state'] == 'arrived' and run.view(0)['loop'] == 2

    def test_ten_resends_then_the_mission_pauses(self):
        run = nav_runner.Run()
        run.start(mission(wp(5, 0)), (0.0, 0.0, 0.0))
        for _ in range(10):
            assert sends(run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0)))
        assert run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0)) == []
        view = run.view(0)
        assert (view['state'], view['reason']) == ('paused', 'failed')
        assert sends(run.resume(None)) and run.retries == 0

    def test_resends_count_per_waypoint(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(5, 0)), (0.0, 0.0, 0.0))
        for _ in range(9):
            run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0))
        run.tick(1.0, (1.0, 0.0, 0.0), True)
        assert run.view(0)['index'] == 1 and run.retries == 0

    def test_failed_waypoint_pauses_then_resume_skip_stop(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(2, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.result(run.number, 'failed', 1.0, None) == []
        view = run.view(0)
        assert (view['state'], view['reason'], view['index']) == ('paused', 'failed', 0)
        assert sends(run.resume(None))[0][2] == 1.0
        run.result(run.number, 'failed', 1.0, None)
        assert sends(run.skip(None))[0][2] == 2.0
        assert run.stop() == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'stopped' and run.stop() == []

    def test_canceled_from_outside_ends_the_mission(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        assert ('cancel',) in run.result(run.number, 'canceled', 1.0, None)
        assert run.view(0)['state'] == 'stopped'

    @pytest.mark.parametrize('status', ['failed', 'canceled'])
    def test_a_goal_ended_with_the_one_canceled_before_it_is_sent_again(self, status):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.tick(10.0, (0.0, 0.0, 0.0), True)
        run.pause()
        run.resume((0.0, 0.0, 0.0))
        again = run.result(run.number, status, 10.3, (0.0, 0.0, 0.0))
        assert sends(again)[0][2:4] == (1, 0) and run.view(0)['state'] == 'running'
        run.result(run.number, 'failed', 13.0, (0.0, 0.0, 0.0))
        assert run.view(0)['state'] == 'paused'

    def test_user_pause(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        assert run.pause() == [('cancel',)]
        assert run.view(0)['reason'] == 'user' and run.pause() == []
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == []

    def test_relocalizing_pauses_and_resends(self):
        run = nav_runner.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        number = run.number
        assert run.tick(1.0, (1.0, 0.0, 0.0), False) == [('cancel',)]
        assert run.view(0)['state'] == 'relocalizing'
        assert run.result(number, 'canceled', 1.0, None) == []
        assert run.tick(1.5, None, False) == []
        assert sends(run.tick(2.0, (1.0, 0.0, 0.0), True))[0][2] == 3.0

    def test_localization_ending_stops_the_mission(self):
        run = nav_runner.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, None, None) == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'stopped' and run.tick(2.0, None, None) == []

    def test_a_stall_pauses_until_the_car_can_drive(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        here = (0.2, 0.0, 0.0)
        assert run.tick(1.0, here, True, stalled=True) == [('cancel',)]
        view = run.view(0)
        assert (view['state'], view['reason'], view['stalled']) == ('paused', 'stalled', True)
        assert run.resume(here) == [] and run.skip(here) == []
        assert run.tick(2.0, here, True, stalled=True) == []
        assert run.tick(3.0, here, True, stalled=False) == []
        assert run.view(0)['state'] == 'paused' and not run.view(0)['stalled']
        assert sends(run.resume(here))[0][2] == 1.0

    def test_stop_works_while_stalled(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.tick(1.0, (0.2, 0.0, 0.0), True, stalled=True)
        assert run.stop() and run.view(0)['state'] == 'stopped'

    def test_a_stall_while_driving_by_hand_changes_nothing(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.pause()
        assert run.tick(1.0, (0.2, 0.0, 0.0), True, stalled=True) == []
        assert run.view(0)['reason'] == 'user'

    def test_marks_next_and_reached(self):
        def marks():
            view = run.view(0)
            return view['next'], view['visited']

        run = nav_runner.Run()
        assert marks() == (None, [])
        run.start(mission(wp(1, 0), wp(3, 0), wp(5, 0), loops=2), (0.0, 0.0, 0.0))
        assert marks() == (0, [])
        run.tick(1, (1.0, 0.0, 0.0), True)
        assert marks() == (1, [0])
        run.skip(None)
        assert marks() == (2, [0])  # a skipped waypoint is not reached
        run.tick(2, (5.0, 0.0, 0.0), True)
        assert marks() == (0, [0, 2])  # the second loop keeps the marks
        run.tick(3, (1.0, 0.0, 0.0), True)
        run.tick(4, (3.0, 0.0, 0.0), True)
        run.tick(5, (5.0, 0.0, 0.0), True)
        assert marks() == (None, [0, 1, 2]) and run.view(0)['state'] == 'arrived'
        run.start(mission(wp(1, 0)), None)
        assert marks() == (0, [])

    def test_next_while_waiting(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0, dwell=5), wp(3, 0, dwell=5)), (0.0, 0.0, 0.0))
        run.tick(0.0, (1.0, 0.0, 0.0), True)
        assert (run.view(0)['next'], run.view(0)['visited']) == (1, [0])
        run.tick(5.0, (1.0, 0.0, 0.0), True)
        run.tick(6.0, (3.0, 0.0, 0.0), True)
        assert run.view(6)['state'] == 'dwelling' and run.view(6)['next'] is None

    def test_edit_keeps_the_mark_with_the_place(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(3, 0), wp(5, 0)), (0.0, 0.0, 0.0))
        run.tick(1, (1.0, 0.0, 0.0), True)
        run.edit(mission(wp(9, 9), wp(1, 0, radius=1), wp(5, 0)), None)
        assert run.view(0)['visited'] == [1]
        run.edit(mission(wp(9, 9), wp(1, 2), wp(5, 0)), None)
        assert run.view(0)['visited'] == []

    def test_no_events_while_not_recording(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.tick(1.0, (1.0, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'arrived' and run.take_events() == []

    def test_events_of_a_mission(self):
        run = nav_runner.Run()
        run.record_events(True)
        run.start(mission(wp(1, 0), wp(3, 0), wp(5, 0)), (0.0, 0.0, 0.0))
        run.result(run.number, 'succeeded', 0.5, (0.2, 0.0, 0.0))
        run.tick(1.0, (1.0, 0.0, 0.0), True)
        run.tick(2.0, (1.5, 0.0, 0.0), False)
        run.tick(3.0, (1.5, 0.0, 0.0), True)
        run.tick(4.0, (1.5, 0.0, 0.0), True, stalled=True)
        run.tick(5.0, (1.5, 0.0, 0.0), True)
        run.resume(None)
        run.pause()
        run.skip(None)
        run.edit(mission(wp(1, 0), wp(3, 0), wp(6, 0)), None)
        run.stop()
        got = run.take_events()
        assert [e['event'] for e in got] == [
            'start', 'sent', 'sent', 'reached', 'sent', 'relocalizing', 'localized', 'sent',
            'paused', 'resumed', 'sent', 'paused', 'skipped', 'sent', 'edited', 'sent', 'stopped',
        ]  # fmt: skip
        assert [e.get('resend') for e in got if e['event'] == 'sent'] == [
            False, True, False, False, False, False, False
        ]  # fmt: skip
        assert [e['reason'] for e in got if e['event'] == 'paused'] == ['stalled', 'user']
        assert [e['waypoint'] for e in got][:5] == [1, 1, 1, 1, 2]
        assert run.take_events() == []
        run.record_events(False)
        run.start(mission(wp(1, 0)), None)
        assert run.take_events() == []

    def test_events_from_the_middle_of_a_mission(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        run.record_events(True, underway=True)
        first = run.take_events()[0]
        assert first['event'] == 'recording' and len(first['mission']['waypoints']) == 2

    def test_refused_goal_pauses(self):
        run = nav_runner.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        run.refused(run.number)
        assert run.view(0)['reason'] == 'failed'

    def test_edit_resends_only_a_changed_target(self):
        run = nav_runner.Run()
        run.start(mission(wp(1, 0), wp(2, 0)), (0.0, 0.0, 0.0))
        version = run.view(0)['version']
        assert run.edit(mission(wp(1, 0), wp(5, 0)), None) == []
        assert sends(run.edit(mission(wp(1, 1), wp(5, 0)), None))[0][2:4] == (1.0, 1.0)
        assert run.view(0)['version'] == version + 2
        run.tick(1, (1.0, 1.0, 0.0), True)
        assert run.view(0)['index'] == 1
        # The waypoint being driven to is removed: the list ends, the last one is sent.
        assert sends(run.edit(mission(wp(1, 1)), None))[0][2:4] == (1.0, 1.0)
        assert run.view(0)['index'] == 0

    def test_edit_before_a_mission_only_stores_it(self):
        run = nav_runner.Run()
        assert run.edit(mission(wp(1, 0)), None) == [] and run.view(0)['count'] == 1

    def test_legs_for_the_preview(self):
        out = nav_runner.legs(mission(wp(1, 0), wp(1, 1, heading=180), loops=2), (0.0, 0.0, 0.0))
        assert [leg[0] is None for leg in out] == [True, False, False]
        assert out[1][1] == (1.0, 1.0, pytest.approx(math.pi))
        assert out[2][1][:2] == (1.0, 0.0) and len(out) == 3
        assert len(nav_runner.legs(mission(wp(1, 0)), None)) == 1


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
    return lambda owner, name=None: DriveLease(owner, name or owner.title(), tmp_path, clock)


@pytest.fixture
def commander(make):
    return nav_runner.Commander(make('nav', 'Navigation'))


HERE = (0.0, 0.0, 0.0)
TWO = mission(wp(5, 0), wp(9, 0))


def start(c, name='Planner', m=TWO, localized=True, stalled=False, ready=True, **more):
    raw = {'verb': 'start', 'owner': name.lower(), 'name': name, 'mission': m, **more}
    return c.command(raw, HERE, localized, stalled, ready)


def say(c, verb, **more):
    return c.command({'verb': verb, **more}, HERE, True, False, True)


class TestCommander:
    def test_start_drives_and_holds_the_lease_under_its_name(self, commander, make):
        ok, message, actions = start(commander)
        assert ok and message == '' and sends(actions)[0][2:4] == (5, 0)
        assert make('teleop').refused() == 'Planner is driving.'
        view = commander.view(0.0)
        assert view['state'] == 'running' and view['owner'] == 'planner' and 'mission' not in view

    @pytest.mark.parametrize(
        'state, text',
        [
            ({'ready': False}, 'Load a map first.'),
            ({'localized': None}, 'Load a map first.'),
            ({'localized': False}, 'Wait until the car is localized.'),
            ({'stalled': True}, 'Car stalled.'),
        ],
    )
    def test_start_refused(self, commander, make, state, text):
        assert start(commander, **state) == (False, text, [])
        assert commander.run.state == 'idle' and make('b').take()

    def test_second_start_is_refused_by_name(self, commander):
        start(commander)
        assert start(commander, name='Terminal')[:2] == (False, 'Planner is driving.')
        say(commander, 'pause')
        assert start(commander, name='Terminal')[:2] == (False, 'Planner is driving.')
        assert commander.name == 'Planner'
        say(commander, 'stop')
        assert start(commander, name='Terminal')[0]

    def test_start_refused_while_something_else_drives(self, commander, make):
        make('webteleop', 'Web Teleop').take()
        assert start(commander) == (False, 'Web Teleop is driving.', [])
        assert say(commander, 'check')[:2] == (False, 'Web Teleop is driving.')

    def test_check_starts_nothing(self, commander, make):
        assert say(commander, 'check') == (True, '', [])
        assert commander.run.state == 'idle' and make('b').take()

    def test_a_bad_mission_is_an_error(self, commander):
        with pytest.raises(ValueError):
            start(commander, m={'waypoints': []})

    def test_unknown_verb(self, commander):
        assert say(commander, 'rm') == (False, 'Unknown command.', [])

    def test_held_through_pause_wait_and_lost_localization(self, commander, make, clock):
        other = make('b')
        start(commander, m=mission(wp(0.1, 0, dwell=5), wp(9, 0)))
        commander.tick(clock.now, HERE, True, False)
        assert commander.run.state == 'dwelling'

        def passes(seconds, localized=True):
            for _ in range(int(seconds * 5)):
                clock.now += 0.2
                commander.tick(clock.now, HERE, localized, False)
                assert not other.take()

        passes(6)
        assert commander.run.state == 'running'
        passes(3, localized=False)
        assert commander.run.state == 'relocalizing'
        passes(1)
        say(commander, 'pause')
        passes(3)
        assert commander.run.state == 'paused'

    @pytest.mark.parametrize('end', ['stop', 'arrived', 'map'])
    def test_given_back_at_the_end(self, commander, make, clock, end):
        start(commander, m=mission(wp(0.1, 0)) if end == 'arrived' else TWO)
        assert not make('b').take()
        if end == 'stop':
            say(commander, 'stop')
        commander.tick(clock.now, HERE, None if end == 'map' else True, False)
        commander.tick(clock.now, HERE, True, False)
        assert not commander.run.underway() and make('b').take()

    def test_losing_the_lease_stops_the_mission(self, commander, make, clock):
        start(commander)
        clock.now += 2.0
        make('b', 'Web Teleop').take()
        assert ('cancel',) in commander.tick(clock.now, HERE, True, False)
        assert commander.run.state == 'stopped'
        assert commander.view(0.0)['message'] == 'Web Teleop is driving.'
        assert start(commander)[:2] == (False, 'Web Teleop is driving.')

    def test_a_slow_moment_does_not_lose_a_free_lease(self, commander, clock):
        start(commander)
        clock.now += 2.0
        commander.tick(clock.now, HERE, True, False)
        assert commander.run.state == 'running' and commander.lease.held

    def test_shared_only_while_the_car_waits_for_the_operator(
        self, commander, tmp_path, clock
    ):
        def shared():
            commander.tick(clock.now, HERE, localized, False)
            return holder(tmp_path, clock.now).share

        localized = True
        start(commander)
        assert not shared()
        localized = False
        assert shared()
        localized = True
        assert not shared()
        say(commander, 'pause')
        assert shared()
        say(commander, 'resume')
        assert not shared()

    def test_panel_drives_while_shared_and_not_over_the_mission(self, commander, make, clock):
        panel = DriveGate(make('mapview', 'Mapper'), keep=False, shared=True)
        second = DriveGate(make('planner', 'Planner'), keep=False, shared=True)
        app = DriveGate(make('webteleop', 'Web Teleop'))
        start(commander)
        commander.tick(clock.now, HERE, True, False)
        assert not panel.allow(True) and panel.refused == 'Planner is driving.'
        say(commander, 'pause')
        commander.tick(clock.now, HERE, True, False)
        assert not panel.allow(True)  # still held from before the pause
        panel.idle()
        assert panel.refused == '' and panel.allow(True)
        assert not second.allow(True) and second.refused == 'Mapper is driving.'
        assert not app.allow(True)
        say(commander, 'resume')
        commander.tick(clock.now, HERE, True, False)
        assert not panel.allow(True)

    @pytest.mark.parametrize('why', ['failed', 'stalled'])
    def test_a_goal_that_cannot_go_on_ends(self, commander, make, clock, why):
        def happens():
            if why == 'failed':
                commander.run.refused(commander.run.number)
            return commander.tick(clock.now, HERE, True, why == 'stalled')

        start(commander, resumable=True)
        happens()
        assert commander.run.state == 'paused' and not make('b').take()
        commander.tick(clock.now, HERE, True, False)
        say(commander, 'stop')
        commander.tick(clock.now, HERE, True, False)
        start(commander, name='racecar nav')
        assert ('cancel',) in happens()
        assert commander.run.state == why and make('b').take()

    def test_events_are_kept_under_the_key_asked_for(self, commander):
        start(commander, log='k1')
        assert commander.log == 'k1'
        assert [e['event'] for e in commander.run.take_events()] == ['start', 'sent']
        say(commander, 'log', log='')
        say(commander, 'pause')
        assert commander.run.take_events() == []
        say(commander, 'log', log='k2')
        assert [e['event'] for e in commander.run.take_events()] == ['recording']
        say(commander, 'stop')
        start(commander)
        assert commander.log == '' and not commander.run.log_on

    def test_edit(self, commander):
        start(commander)
        ok, _, actions = say(commander, 'edit', mission=mission(wp(7, 0)))
        assert ok and sends(actions)[0][2:4] == (7, 0)
