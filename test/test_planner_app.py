"""scripts/planner_app.py: the Planner server and its page."""

import gzip
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading

from conftest import load_script
import pytest
from racecar_neo_ros2_driver import missions

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def pa():
    names = ('sysinfo', 'features', 'map_control', 'webui', 'labkit', 'mapview', 'racecar_log')
    for name in (*names, 'mission_recorder'):
        load_script(name)
    return load_script('planner_app')


def wp(x, y, heading=None, radius=0.25, speed=None, dwell=0):
    return {'x': x, 'y': y, 'heading_deg': heading, 'radius_m': radius,
            'speed_mps': speed, 'dwell_sec': dwell}  # fmt: skip


def mission(*points, loops=1):
    return missions.parse({'loops': loops, 'waypoints': list(points)})


class FakeNavigator:
    def __init__(self):
        self.touched = 0
        self.started = []
        self.labels = []
        self.controls = []
        self.edits = []
        self.costmap = []
        self.ready = True
        self.rec_summary = {'id': 2, 'text': 'bag'}
        self.map_name = 'hall'

    def touch(self):
        self.touched += 1

    def snapshot(self, trail_from, trail_id):
        return {'status': {'mode': 'localization'}, 'mission': {'state': 'idle'}}

    def start(self, m, label=''):
        if not self.ready:
            return False, 'Load a map first.'
        self.labels.append(label)
        self.started.append(m)
        return True, ''

    def control(self, verb):
        self.controls.append(verb)
        return True, ''

    def edit(self, m):
        self.edits.append(m)
        return (True, '') if self.ready else (False, 'Load a map first.')

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
        request(server, 'POST', '/api/run', {'mission': RAW, 'name': 'loop5'})
        request(server, 'POST', '/api/run', {'mission': RAW, 'name': '../x'})
        assert pa.Handler.viewer.labels == ['', 'loop5', '']
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
        pa.Handler.viewer.ready = False
        res, data = request(server, 'POST', '/api/edit', {'mission': RAW})
        assert res.status == 409 and data['message'] == 'Load a map first.'

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
        assert all(set(g) == {'id', 'name'} for g in data['groups'])
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

    def test_recording_summary(self, server):
        assert request(server, 'GET', '/api/recording')[1] == {'id': 2, 'text': 'bag'}

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

    def test_camera_panel_and_override_slot(self):
        assert 'ui/camera_panel.js' in self.html and "key: 'rcn-planner-camera'" in self.html
        # The banner sits outside the map, where its button can be pressed.
        assert self.html.index('id="override-slot"') < self.html.index('<main>')
        assert "getElementById('override-slot')" in (SCRIPTS / 'web' / 'override.js').read_text()

    def test_camera_panel_stops_when_hidden_and_grays_within_a_second(self):
        js = (SCRIPTS / 'web' / 'camera_panel.js').read_text()
        assert 'shown && enabled && !document.hidden' in js and "store.get(key, '0')" in js
        stale_ms = int(js.split('const STALE_MS = ')[1].split(';')[0])
        assert stale_ms / 1000 + load_script('overview').STALE_SEC <= 1.0

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

    def test_recording_is_off_until_its_toggle_is_on(self):
        assert 'noRecord = () => ({ on: false' in self.html
        assert '<input type="checkbox" id="recon"> Record' in self.html
        assert 'Recording</span>' in self.html and "pill('warn', 'Not recording')" in self.html

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
