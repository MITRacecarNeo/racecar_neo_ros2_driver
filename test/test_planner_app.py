"""scripts/planner_app.py: the Planner server, its goal state, and its page."""

import gzip
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading

from conftest import load_script
import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def pa():
    for name in ('sysinfo', 'map_control', 'webui', 'labkit', 'mapview'):
        load_script(name)
    return load_script('planner_app')


class FakeNavigator:
    def __init__(self):
        self.touched = 0
        self.goals = []
        self.stops = 0
        self.costmap = []
        self.ready = True

    def touch(self):
        self.touched += 1

    def snapshot(self, trail_from, trail_id):
        return {'status': {'mode': 'localization'}, 'mission': {'state': 'idle'}}

    def go(self, x, y, yaw):
        if not self.ready:
            return False, 'Load a map first.'
        self.goals.append((x, y, yaw))
        return True, ''

    def stop(self):
        self.stops += 1

    def preview(self, x, y, yaw):
        if x > 5:
            return False, 'No path to this goal.'
        return True, [[0.0, 0.0], [x, y]]

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
def server(pa):
    pa.Handler.viewer = FakeNavigator()
    pa.Handler.job = Job()
    pa.Handler.checks = Job()
    srv = ThreadingHTTPServer(('127.0.0.1', 0), pa.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    srv.server_close()


def request(srv, method, path, body=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
    conn.request(method, path, body=None if body is None else json.dumps(body))
    res = conn.getresponse()
    data = res.read()
    conn.close()
    return res, data


class TestHelpers:
    def test_thin_keeps_both_ends(self, pa):
        points = [[i, 0] for i in range(1000)]
        out = pa.thin(points, 50)
        assert len(out) == 50 and out[0] == [0, 0] and out[-1] == [999, 0]
        assert pa.thin(points[:3], 50) == points[:3]

    def test_mission_ignores_results_of_an_old_goal(self, pa):
        m = pa.Mission()
        first = m.begin([1.0, 2.0, 0.0])
        second = m.begin([3.0, 4.0, 0.5])
        m.update(first, state='arrived')
        assert m.view()['state'] == 'sending'
        m.update(second, state='driving', left=1.234, recoveries=1)
        view = m.view()
        assert view == {'state': 'driving', 'goal': [3.0, 4.0, 0.5], 'left': 1.23, 'recoveries': 1}

    def test_mission_pauses_while_localization_is_lost(self, pa):
        m = pa.Mission()
        number = m.begin([1.0, 0.0, 0.0])
        m.update(number, state='driving')
        assert m.step(True) is None
        assert m.step(False) == 'pause' and m.view()['state'] == 'relocalizing'
        m.update(number, state='stopped')  # the canceled Nav2 goal reports late
        assert m.view()['state'] == 'relocalizing'
        assert m.step(False) == 'nudge' and m.step(False) == 'nudge'
        assert m.step(True) == 'resume'
        assert m.view()['goal'] == [1.0, 0.0, 0.0]

    def test_stop_ends_a_paused_goal(self, pa):
        m = pa.Mission()
        m.begin([1.0, 0.0, 0.0])
        assert m.step(False) == 'pause'
        m.stop()
        assert m.view()['state'] == 'stopped' and m.step(False) is None
        m.stop()
        assert m.view()['state'] == 'stopped'

    def test_goal_stops_when_localization_ends(self, pa):
        m = pa.Mission()
        m.begin([1.0, 0.0, 0.0])
        assert m.step(None) == 'pause' and m.view()['state'] == 'stopped'
        assert m.step(None) is None

    def test_finished_goals_ignore_localization(self, pa):
        m = pa.Mission()
        number = m.begin([1.0, 0.0, 0.0])
        m.update(number, state='arrived')
        assert m.step(False) is None and m.view()['state'] == 'arrived'

    def test_registry(self, pa):
        assert pa.PORT == 8090
        assert pa.PLANNER_ACTIONS == ('load', 'save', 'stop')


class TestHttp:
    def test_page(self, server):
        res, data = request(server, 'GET', '/')
        assert res.status == 200 and b'<title>RACECAR Neo Planner</title>' in data

    def test_goal(self, pa, server):
        res, data = request(server, 'POST', '/api/goal', {'x': 1, 'y': -2, 'yaw': 0.5})
        assert res.status == 200 and json.loads(data)['ok']
        assert pa.Handler.viewer.goals == [(1.0, -2.0, 0.5)]
        bad_goals = (
            {'x': 1, 'y': 2},
            {'x': 'nan', 'y': 0, 'yaw': 0},
            {'x': 'a', 'y': 0, 'yaw': 0},
        )
        for bad in bad_goals:
            res, data = request(server, 'POST', '/api/goal', bad)
            assert res.status == 400 and json.loads(data)['message'] == 'Reload the page.'
        assert len(pa.Handler.viewer.goals) == 1

    def test_goal_without_nav2(self, pa, server):
        pa.Handler.viewer.ready = False
        res, data = request(server, 'POST', '/api/goal', {'x': 1, 'y': 0, 'yaw': 0})
        assert res.status == 409 and json.loads(data)['message'] == 'Load a map first.'

    def test_preview(self, server):
        res, data = request(server, 'POST', '/api/preview', {'x': 1, 'y': 2, 'yaw': 0})
        assert res.status == 200 and json.loads(data)['path'] == [[0.0, 0.0], [1.0, 2.0]]
        res, data = request(server, 'POST', '/api/preview', {'x': 9, 'y': 0, 'yaw': 0})
        assert res.status == 409 and json.loads(data)['message'] == 'No path to this goal.'
        res, _ = request(server, 'POST', '/api/preview', {'x': 'inf', 'y': 0, 'yaw': 0})
        assert res.status == 400

    def test_stop(self, pa, server):
        assert request(server, 'POST', '/api/stop', {})[0].status == 200
        assert pa.Handler.viewer.stops == 1

    def test_costmap_on_request(self, pa, server):
        assert request(server, 'GET', '/api/costmap')[0].status == 204
        request(server, 'POST', '/api/costmap', {'on': True})
        res, data = request(server, 'GET', '/api/costmap')
        assert json.loads(res.getheader('X-Map-Info'))['version'] == 3
        assert gzip.decompress(data) == bytes([0, 99])
        request(server, 'POST', '/api/costmap', {'on': False})
        assert pa.Handler.viewer.costmap == [True, False]

    def test_only_planner_map_actions(self, pa, server):
        res, data = request(server, 'POST', '/api/action', {'action': 'load', 'name': 'hall'})
        assert res.status == 200 and json.loads(data)['argv'][:2] == ['load', 'hall']
        for action in ('start', 'recover', 'resume'):
            res, _ = request(server, 'POST', '/api/action', {'action': action, 'name': 'hall'})
            assert res.status == 400, action
        assert [a[0] for a in pa.Handler.job.argv] == ['load']

    def test_cross_site_post_refused(self, pa, server):
        conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=5)
        conn.request('POST', '/api/goal', body=json.dumps({'x': 0, 'y': 0, 'yaw': 0}),
                     headers={'Origin': 'http://elsewhere.test'})  # fmt: skip
        assert conn.getresponse().status == 403
        conn.close()
        assert pa.Handler.viewer.goals == []

    def test_mapper_routes_still_work(self, server):
        assert request(server, 'GET', '/api/state')[0].status == 200


class TestPage:
    html = (SCRIPTS / 'planner.html').read_text()

    def test_shares_the_map_view_and_override(self):
        assert 'ui/mapview_core.js' in self.html and 'ui/override.js' in self.html
        assert "confirm('Turn on safety override?')" in self.html

    def test_costmap_is_off_until_asked(self):
        assert 'setCostmap(false);' in self.html
        button = 'id="costmap" title="Costmap" aria-label="Costmap" aria-pressed="false"'
        assert button in self.html

    def test_relocalizing_banner_and_preview(self):
        banner = 'Localization lost. Regain position by driving the car or using Set pose.'
        assert banner in self.html and "pill('warn', 'Relocalizing')" in self.html
        assert "post('api/preview'" in self.html and "'No path to this goal.'" in self.html

    def test_labels(self):
        assert "button('Set goal'" in self.html and "button('Load map'" in self.html
        assert self.html.index('id="override"') < self.html.index('id="more"')

    def test_loading_while_mapping_asks_to_save(self):
        assert '<h2>Unsaved map</h2>' in self.html
        assert 'Save and load' in self.html and 'Load without saving' in self.html

    def test_mapper_uses_the_shared_view(self):
        mapper = (SCRIPTS / 'mapview.html').read_text()
        assert 'ui/mapview_core.js' in mapper and 'RcnMapView.create' in mapper
        assert 'function toScreen' not in mapper
