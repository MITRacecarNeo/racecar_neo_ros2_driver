"""scripts/planner_app.py: the Planner server, its mission runner, and its page."""

import gzip
import http.client
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading

from conftest import load_script
import pytest
from racecar_neo_ros2_driver import missions

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def pa():
    for name in ('sysinfo', 'map_control', 'webui', 'labkit', 'mapview'):
        load_script(name)
    return load_script('planner_app')


def wp(x, y, heading=None, radius=0.25, speed=None, dwell=0):
    return {'x': x, 'y': y, 'heading_deg': heading, 'radius_m': radius,
            'speed_mps': speed, 'dwell_sec': dwell}  # fmt: skip


def mission(*points, loops=1):
    return missions.parse({'loops': loops, 'waypoints': list(points)})


def sends(actions):
    return [a for a in actions if a[0] == 'send']


class TestRun:
    def test_start_sends_the_first_waypoint_with_its_speed(self, pa):
        run = pa.Run()
        actions = run.start(mission(wp(2, 0, speed=0.6), wp(2, 2)), (0.0, 0.0, 0.0))
        assert actions[0] == ('limit', 0.6)
        kind, _, x, y, yaw = actions[1]
        assert (kind, x, y) == ('send', 2.0, 0.0) and yaw == pytest.approx(0.0)
        assert run.view(0)['state'] == 'running'

    def test_any_heading_faces_the_way_it_is_approached(self, pa):
        run = pa.Run()
        run.start(mission(wp(0, 0), wp(0, 3)), (0.0, -1.0, 0.0))
        actions = run.tick(1.0, (0.1, 0.0, 0.0), True)
        assert sends(actions)[0][4] == pytest.approx(math.pi / 2)

    def test_reached_within_radius_preempts_with_the_next(self, pa):
        run = pa.Run()
        run.start(mission(wp(2, 0, radius=1.0), wp(4, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (0.5, 0.0, 0.0), True) == []
        actions = run.tick(1.1, (1.2, 0.0, 0.0), True)
        assert ('cancel',) not in actions and sends(actions)[0][2:4] == (4.0, 0.0)
        assert run.view(0)['index'] == 1

    def test_heading_must_match(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0, heading=90), wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == []
        assert sends(run.tick(1.1, (1.0, 0.0, math.radians(80)), True))

    def test_last_waypoint_arrives_and_lifts_the_limit(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0, speed=0.5)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'arrived'

    def test_dwell_stops_then_goes_on(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0, dwell=5), wp(2, 0)), (0.0, 0.0, 0.0))
        assert run.tick(10.0, (1.0, 0.0, 0.0), True) == [('cancel',)]
        assert run.view(12.0)['dwell_left'] == 3.0
        assert run.tick(14.0, (1.0, 0.0, 0.0), True) == []
        assert sends(run.tick(15.0, (1.0, 0.0, 0.0), True))[0][2] == 2.0

    def test_loops_return_to_the_first(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(2, 0), loops=2), (0.0, 0.0, 0.0))
        run.tick(1, (1.0, 0.0, 0.0), True)
        actions = run.tick(2, (2.0, 0.0, 0.0), True)
        assert sends(actions)[0][2] == 1.0 and run.view(0)['loop'] == 2
        run.tick(3, (1.0, 0.0, 0.0), True)
        run.tick(4, (2.0, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'arrived'

    def test_until_stop_keeps_looping(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(2, 0), loops=0), (0.0, 0.0, 0.0))
        for i in range(10):
            run.tick(i, (1.0 + i % 2, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'running' and run.view(0)['loop'] == 6

    def test_nav2_success_near_the_waypoint_counts(self, pa):
        run = pa.Run()
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
    def test_nav2_success_away_from_the_waypoint_sends_it_again(self, pa, pose):
        run = pa.Run()
        run.start(mission(wp(1, 0, heading=0), wp(2, 0)), (0.0, 0.0, 0.0))
        number = run.number
        again = sends(run.result(number, 'succeeded', 1.0, pose))
        assert again[0][1] == number + 1 and again[0][2] == 1.0
        assert run.view(0)['state'] == 'running' and run.view(0)['index'] == 0

    def test_hand_off_success_does_not_skip_the_last_waypoint(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        here = (0.9, 0.0, 0.0)
        run.tick(1.0, here, True)
        assert sends(run.result(run.number, 'succeeded', 1.05, here))[0][2] == 3.0
        assert run.view(0)['state'] == 'running'
        run.tick(3.0, (2.9, 0.0, 0.0), True)
        assert run.view(0)['state'] == 'arrived'

    def test_hand_off_success_does_not_skip_in_a_loop(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(3, 0), loops=2), (0.0, 0.0, 0.0))
        for x in (1.0, 3.0, 1.0, 3.0):
            here = (x, 0.0, 0.0)
            run.tick(1.0, here, True)
            if run.view(0)['state'] == 'running':
                before = run.view(0)['index'], run.view(0)['loop']
                run.result(run.number, 'succeeded', 1.1, here)
                assert (run.view(0)['index'], run.view(0)['loop']) == before
        assert run.view(0)['state'] == 'arrived' and run.view(0)['loop'] == 2

    def test_ten_resends_then_the_mission_pauses(self, pa):
        run = pa.Run()
        run.start(mission(wp(5, 0)), (0.0, 0.0, 0.0))
        for _ in range(10):
            assert sends(run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0)))
        assert run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0)) == []
        view = run.view(0)
        assert (view['state'], view['reason']) == ('paused', 'failed')
        assert sends(run.resume(None)) and run.retries == 0

    def test_resends_count_per_waypoint(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(5, 0)), (0.0, 0.0, 0.0))
        for _ in range(9):
            run.result(run.number, 'succeeded', 1.0, (0.0, 0.0, 0.0))
        run.tick(1.0, (1.0, 0.0, 0.0), True)
        assert run.view(0)['index'] == 1 and run.retries == 0

    def test_failed_waypoint_pauses_then_resume_skip_stop(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(2, 0), wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.result(run.number, 'failed', 1.0, None) == []
        view = run.view(0)
        assert (view['state'], view['reason'], view['index']) == ('paused', 'failed', 0)
        assert sends(run.resume(None))[0][2] == 1.0
        run.result(run.number, 'failed', 1.0, None)
        assert sends(run.skip(None))[0][2] == 2.0
        assert run.stop() == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'stopped' and run.stop() == []

    def test_canceled_reply_is_not_a_failure(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        assert run.result(run.number, 'canceled', 1.0, None) == []
        assert run.view(0)['state'] == 'running'

    def test_user_pause(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        assert run.pause() == [('cancel',)]
        assert run.view(0)['reason'] == 'user' and run.pause() == []
        assert run.tick(1.0, (1.0, 0.0, 0.0), True) == []

    def test_relocalizing_pauses_and_resends(self, pa):
        run = pa.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        number = run.number
        assert run.tick(1.0, (1.0, 0.0, 0.0), False) == [('cancel',)]
        assert run.view(0)['state'] == 'relocalizing'
        assert run.result(number, 'canceled', 1.0, None) == []
        assert run.tick(1.5, None, False) == []
        assert sends(run.tick(2.0, (1.0, 0.0, 0.0), True))[0][2] == 3.0

    def test_localization_ending_stops_the_mission(self, pa):
        run = pa.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        assert run.tick(1.0, None, None) == [('cancel',), ('limit', None)]
        assert run.view(0)['state'] == 'stopped' and run.tick(2.0, None, None) == []

    def test_a_stall_pauses_until_the_car_can_drive(self, pa):
        run = pa.Run()
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

    def test_stop_works_while_stalled(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.tick(1.0, (0.2, 0.0, 0.0), True, stalled=True)
        assert run.stop() and run.view(0)['state'] == 'stopped'

    def test_a_stall_while_driving_by_hand_changes_nothing(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0)), (0.0, 0.0, 0.0))
        run.pause()
        assert run.tick(1.0, (0.2, 0.0, 0.0), True, stalled=True) == []
        assert run.view(0)['reason'] == 'user'

    def test_marks_next_and_reached(self, pa):
        def marks():
            view = run.view(0)
            return view['next'], view['visited']

        run = pa.Run()
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

    def test_next_while_waiting(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0, dwell=5), wp(3, 0, dwell=5)), (0.0, 0.0, 0.0))
        run.tick(0.0, (1.0, 0.0, 0.0), True)
        assert (run.view(0)['next'], run.view(0)['visited']) == (1, [0])
        run.tick(5.0, (1.0, 0.0, 0.0), True)
        run.tick(6.0, (3.0, 0.0, 0.0), True)
        assert run.view(6)['state'] == 'dwelling' and run.view(6)['next'] is None

    def test_edit_keeps_the_mark_with_the_place(self, pa):
        run = pa.Run()
        run.start(mission(wp(1, 0), wp(3, 0), wp(5, 0)), (0.0, 0.0, 0.0))
        run.tick(1, (1.0, 0.0, 0.0), True)
        run.edit(mission(wp(9, 9), wp(1, 0, radius=1), wp(5, 0)), None)
        assert run.view(0)['visited'] == [1]
        run.edit(mission(wp(9, 9), wp(1, 2), wp(5, 0)), None)
        assert run.view(0)['visited'] == []

    def test_refused_goal_pauses(self, pa):
        run = pa.Run()
        run.start(mission(wp(3, 0)), (0.0, 0.0, 0.0))
        run.refused(run.number)
        assert run.view(0)['reason'] == 'failed'

    def test_edit_resends_only_a_changed_target(self, pa):
        run = pa.Run()
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

    def test_edit_before_a_mission_only_stores_it(self, pa):
        run = pa.Run()
        assert run.edit(mission(wp(1, 0)), None) == [] and run.view(0)['count'] == 1

    def test_legs_for_the_preview(self, pa):
        out = pa.legs(mission(wp(1, 0), wp(1, 1, heading=180), loops=2), (0.0, 0.0, 0.0))
        assert [leg[0] is None for leg in out] == [True, False, False]
        assert out[1][1] == (1.0, 1.0, pytest.approx(math.pi))
        assert out[2][1][:2] == (1.0, 0.0) and len(out) == 3
        assert len(pa.legs(mission(wp(1, 0)), None)) == 1


class FakeNavigator:
    def __init__(self):
        self.touched = 0
        self.started = []
        self.controls = []
        self.edits = []
        self.costmap = []
        self.ready = True
        self.map_name = 'hall'

    def touch(self):
        self.touched += 1

    def snapshot(self, trail_from, trail_id):
        return {'status': {'mode': 'localization'}, 'mission': {'state': 'idle'}}

    def start(self, m):
        if not self.ready:
            return False, 'Load a map first.'
        self.started.append(m)
        return True, ''

    def control(self, verb):
        self.controls.append(verb)

    def edit(self, m):
        self.edits.append(m)

    def preview(self, m):
        legs = [[[0.0, 0.0], [w['x'], w['y']]] if w['x'] < 5 else None for w in m['waypoints']]
        return True, legs

    def set_costmap(self, on):
        self.costmap.append(on)

    def costmap_payload(self):
        if not self.costmap or not self.costmap[-1]:
            return None
        info = {'width': 2, 'height': 1, 'resolution': 0.05, 'origin': [0, 0], 'version': 3}
        return info, gzip.compress(bytes([0, 99]))


class Job:
    def __init__(self):
        self.argv = []

    def start(self, argv):
        self.argv.append(argv)
        return True

    def state(self):
        return {'running': False}


@pytest.fixture
def server(pa, tmp_path, monkeypatch):
    monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
    pa.Handler.viewer = FakeNavigator()
    pa.Handler.job = Job()
    pa.Handler.checks = Job()
    srv = ThreadingHTTPServer(('127.0.0.1', 0), pa.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    srv.server_close()


def request(srv, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
    data = None if body is None else json.dumps(body)
    conn.request(method, path, body=data, headers=headers or {})
    res = conn.getresponse()
    raw = res.read()
    conn.close()
    is_json = (res.getheader('Content-Type') or '').startswith('application/json')
    return res, json.loads(raw) if raw and is_json else raw


RAW = {'loops': 2, 'waypoints': [wp(1, 2, heading=90), wp(3, 4)]}


class TestHttp:
    def test_page(self, server):
        res, data = request(server, 'GET', '/')
        assert res.status == 200 and b'<title>RACECAR Neo Planner</title>' in data

    def test_run(self, pa, server):
        res, data = request(server, 'POST', '/api/run', {'mission': RAW})
        assert res.status == 200 and data['ok']
        assert pa.Handler.viewer.started == [missions.parse(RAW)]
        bad = {'loops': 1, 'waypoints': [wp(1, 2, speed=2.0)]}
        res, data = request(server, 'POST', '/api/run', {'mission': bad})
        assert res.status == 400 and data['message'] == 'Reload the page.'
        pa.Handler.viewer.ready = False
        res, data = request(server, 'POST', '/api/run', {'mission': RAW})
        assert res.status == 409 and data['message'] == 'Load a map first.'

    def test_controls_and_edit(self, pa, server):
        for verb in ('pause', 'resume', 'skip', 'stop'):
            assert request(server, 'POST', '/api/control', {'verb': verb})[0].status == 200
        assert request(server, 'POST', '/api/control', {'verb': 'rm'})[0].status == 400
        assert request(server, 'POST', '/api/stop', {})[0].status == 200
        assert pa.Handler.viewer.controls == ['pause', 'resume', 'skip', 'stop', 'stop']
        assert request(server, 'POST', '/api/edit', {'mission': RAW})[0].status == 200
        assert len(pa.Handler.viewer.edits) == 1

    def test_preview(self, server):
        two = {'loops': 1, 'waypoints': [wp(1, 2), wp(9, 0)]}
        res, data = request(server, 'POST', '/api/preview', {'mission': two})
        assert res.status == 200 and data['legs'][0] == [[0.0, 0.0], [1.0, 2.0]]
        assert data['legs'][1] is None

    def test_save_list_open_and_run_saved(self, pa, server, tmp_path):
        save = {'name': 'loop', 'mission': RAW}
        assert request(server, 'POST', '/api/missions/save', save)[0].status == 200
        res, data = request(server, 'POST', '/api/missions/save', save)
        assert res.status == 409 and data['exists']
        res, _ = request(server, 'POST', '/api/missions/save', {**save, 'replace': True})
        assert res.status == 200
        assert len(list((tmp_path / 'hall' / 'missions' / 'previous').iterdir())) == 1
        res, data = request(server, 'GET', '/api/missions')
        assert data['map'] == 'hall' and [m['name'] for m in data['missions']] == ['loop']
        res, data = request(server, 'GET', '/api/missions/one?name=loop')
        assert data['mission'] == missions.parse(RAW)
        assert request(server, 'GET', '/api/missions/one?name=none')[0].status == 404
        assert request(server, 'POST', '/api/run_saved', {'name': 'loop'})[0].status == 200
        assert request(server, 'POST', '/api/run_saved', {'name': 'none'})[0].status == 404
        res, data = request(server, 'POST', '/api/missions/save', {'name': 'a b', 'mission': RAW})
        assert data['message'] == 'Mission names use letters, digits, - and _.'

    def test_no_map_no_missions(self, pa, server):
        pa.Handler.viewer.map_name = ''
        assert request(server, 'GET', '/api/missions')[1]['missions'] == []
        res, data = request(server, 'POST', '/api/missions/save', {'name': 'x', 'mission': RAW})
        assert res.status == 409 and data['message'] == 'Load a map first.'

    def test_costmap_on_request(self, pa, server):
        assert request(server, 'GET', '/api/costmap')[0].status == 204
        request(server, 'POST', '/api/costmap', {'on': True})
        res, data = request(server, 'GET', '/api/costmap')
        assert json.loads(res.getheader('X-Map-Info'))['version'] == 3
        assert gzip.decompress(data) == bytes([0, 99])

    def test_only_planner_map_actions(self, pa, server):
        res, data = request(server, 'POST', '/api/action', {'action': 'load', 'name': 'hall'})
        assert res.status == 200 and data['argv'][:2] == ['load', 'hall']
        for action in ('start', 'recover', 'resume'):
            res, _ = request(server, 'POST', '/api/action', {'action': action, 'name': 'hall'})
            assert res.status == 400, action

    def test_cross_site_post_refused(self, pa, server):
        res, _ = request(server, 'POST', '/api/run', {'mission': RAW},
                         headers={'Origin': 'http://elsewhere.test'})  # fmt: skip
        assert res.status == 403 and pa.Handler.viewer.started == []

    def test_mapper_routes_still_work(self, server):
        assert request(server, 'GET', '/api/state')[0].status == 200


class TestPage:
    html = (SCRIPTS / 'planner.html').read_text()

    def test_shares_the_map_view_and_override(self):
        assert 'ui/mapview_core.js' in self.html and 'ui/override.js' in self.html
        assert "confirm('Turn on safety override?')" in self.html
        assert 'mv.pick = ' in self.html

    def test_costmap_is_off_until_asked(self):
        assert 'setCostmap(false);' in self.html
        button = 'id="costmap" title="Costmap" aria-label="Costmap" aria-pressed="false"'
        assert button in self.html

    def test_mission_controls_and_text(self):
        for text in ("button('Pause'", "button('Resume'", "button('Skip'", "button('Waypoints'",
                     "button('Set goal'", "button('Load map'", '<h2>Waypoints</h2>',
                     '<h2>Save mission</h2>', 'Until Stop',
                     '`Could not reach waypoint ${m.index + 1}`',
                     '`Car stalled at waypoint ${m.index + 1}`',
                     '--wp-todo: #f63c47; --wp-next: #ffd93d; --wp-done: #6fd08c;',
                     '`No path to waypoint ${failed + 1}.`',
                     'Localization lost. Regain position by driving the car or using Set pose.'):
            assert text in self.html, text
        assert self.html.index('id="override"') < self.html.index('id="more"')

    def test_colors_replace_the_next_text(self):
        assert "'Next'" not in self.html and 'map-goal' not in self.html
        assert 'wpnum ${wpState(i)}' in self.html and 'paintWps();' in self.html

    def test_ascii_only(self):
        assert all(ord(c) < 128 for c in self.html)

    def test_loading_while_mapping_asks_to_save(self):
        assert '<h2>Unsaved map</h2>' in self.html
        assert 'Save and load' in self.html and 'Load without saving' in self.html

    def test_mapper_uses_the_shared_view(self):
        mapper = (SCRIPTS / 'mapview.html').read_text()
        assert 'ui/mapview_core.js' in mapper and 'RcnMapView.create' in mapper
        core = (SCRIPTS / 'web' / 'mapview_core.js').read_text()
        assert 'mv.pick' in core and 'once.once = true' in core
