"""scripts/profile_app.py: the Car Profile app on port 8088."""

import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading

from conftest import load_script
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
# Node names of the sim's RACECAR Neo V2 model export.
MODEL_NODES = {
    'battery', 'bottom_structure', 'esc', 'motor', 'steering_servo', 'wheel_fl', 'wheel_fr',
    'wheel_rl', 'wheel_rr', 'wheels', 'deck_bottom', 'imu', 'middle_structure', 'raspberry_pi',
    'realsense_camera', 'teensy_pcb', 'deck_middle', 'dot_matrix', 'led_strip', 'lidar',
    'top_structure', 'deck_top', 'RacecarNeoV2', 'power_sensor',
}  # fmt: skip


@pytest.fixture
def pa():
    return load_script('profile_app')


class FakeLive:
    def __init__(self):
        self.watched = []
        self.types = {'/scan': 'sensor_msgs/msg/LaserScan'}

    def watch(self, topics):
        self.watched.append(list(topics))
        return dict.fromkeys(topics, 7.0)

    def watch_firmware(self):
        return {'fw.version': '0.9.2', 'fw.git': '0e94b5d'}


@pytest.fixture
def server(pa, car_profile_dir, monkeypatch):
    car_profile_dir.mkdir(parents=True, exist_ok=True)
    (car_profile_dir / 'car.yaml').write_text(
        yaml.safe_dump({'name': 'bill', 'nickname': 'Bill', 'id': 0, 'record': {'battery': 'B'}})
    )
    monkeypatch.setattr(
        pa, 'fetch_about', lambda port_=None: {'driver': '0.9.3', 'firmware': '0.9.2'}
    )
    monkeypatch.setattr(
        pa, 'access_point', lambda run=None: {'ssid': 'racecar-neo-0', 'ip': '10.42.0.1'}
    )
    pa.Handler.live = FakeLive()
    srv = ThreadingHTTPServer(('127.0.0.1', 0), pa.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    srv.server_close()


def request(srv, path, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
    conn.request('GET', path, headers=headers or {})
    res = conn.getresponse()
    data = res.read()
    conn.close()
    return res, data


class TestParts:
    def test_every_clickable_node_is_in_the_model(self, pa):
        nodes = set(pa.PARTS) | set(pa.ALIASES) | set(pa.STRUCTURES)
        assert nodes <= MODEL_NODES

    def test_every_model_part_has_a_view(self, pa):
        decks = {'deck_bottom', 'deck_middle', 'deck_top', 'RacecarNeoV2'}
        assert MODEL_NODES - decks <= set(pa.PARTS) | set(pa.ALIASES) | set(pa.STRUCTURES)

    def test_model_file_matches_when_present(self, pa):
        model = Path('/mnt/nvme/data/models/car') / pa.MODEL_NAME
        if not model.is_file():
            pytest.skip('no model on this machine')
        import struct

        with model.open('rb') as f:
            f.read(12)
            length, _ = struct.unpack('<II', f.read(8))
            names = {n.get('name') for n in json.loads(f.read(length))['nodes']}
        assert names == MODEL_NODES

    def test_calibration_keys_exist(self, pa):
        import sys

        sys.path.insert(0, str(ROOT / 'scripts' / 'calibration'))
        import calibrate

        keys = {k for p in pa.PARTS.values() for k in p.calibration}
        assert keys <= set(calibrate.STEP_KEYS)

    def test_part_view_filters_frames_and_record(self, pa):
        car = {'frames': {'laser_x_mm': 1.0, 'imu_x_mm': 2.0}, 'calibration': [], 'record': {}}
        view = pa.part_view('lidar', car, {'/scan': 7.2})
        assert view['frames'] == {'laser_x_mm': 1.0}
        assert view['topics'] == [{'topic': '/scan', 'hz': 7.2}]
        assert pa.part_view('wheel_fl', car, {})['id'] == 'wheels'
        assert pa.part_view('top_structure', car, {})['label'] == 'Top deck'
        with pytest.raises(KeyError):
            pa.part_view('nothing', car, {})

    def test_parse_firmware(self, pa):
        assert pa.parse_firmware('info=1\n\n  fw.version=0.9.2\n') == {
            'info': '1',
            'fw.version': '0.9.2',
        }

    def test_access_point_reads_ssid_and_address(self, pa):
        out = '802-11-wireless.ssid:racecar-neo-3\nIP4.ADDRESS[1]:10.42.0.1/24\n'
        assert pa.access_point(lambda argv: out) == {'ssid': 'racecar-neo-3', 'ip': '10.42.0.1'}


class TestRates:
    def test_window(self, pa):
        now = [0.0]
        rates = pa.Rates(lambda: now[0])
        for i in range(20):
            now[0] = i * 0.1
            rates.add('/scan')
        assert rates.hz('/scan') == pytest.approx(10.0, abs=0.6)
        now[0] = 10.0
        assert rates.hz('/scan') is None


class TestHandbookRoutes:
    def test_links_point_at_routes(self, pa):
        assert pa.handbook_link('installation.md#nvme') == '/handbook/installation#nvme'
        assert pa.handbook_link('../troubleshooting.md#x') == '/handbook/troubleshooting#x'
        assert pa.handbook_link('https://github.com/x') == 'https://github.com/x'

    def test_view_has_pages_and_html(self, pa):
        view = pa.handbook_view('installation')
        assert view['title'] == 'Installation'
        assert 'id="nvme"' in view['html']
        assert pa.handbook_view('troubleshooting')['title'] == 'Troubleshooting'
        assert pa.handbook_view('missing') is None

    def test_menu_leaves_out_hidden_pages(self, pa):
        names = [p['name'] for p in pa.handbook_menu()]
        assert names[0] == 'index' and 'installation' in names
        assert 'troubleshooting' not in names


def test_power_view(pa):
    battery = {
        'volts': 8.0,
        'amps': 2.0,
        'stale': False,
        'estimate': {'soc': 90.0, 'minutes_left': 60},
    }
    assert pa.power_view(battery) == {
        'Voltage (V)': 8.0, 'Current (A)': 2.0, 'Power (W)': 16.0, 'Charge (%)': 90.0,
        'Time left (min)': 60,
    }  # fmt: skip
    assert pa.power_view({'volts': 8.0, 'stale': True}) == {}
    assert pa.power_view(None) == {}


class TestHttp:
    def test_home_page(self, server):
        res, data = request(server, '/')
        assert res.status == 200 and b'importmap' in data and b'profile_nav.js' in data

    def test_car(self, server):
        car = json.loads(request(server, '/api/car')[1])
        assert car['name'] == 'bill' and car['nickname'] == 'Bill'
        assert car['ap']['ssid'] == 'racecar-neo-0'
        assert {r['key'] for r in car['calibration']} >= {'imu', 'lidar'}

    def test_part_watches_its_topics(self, pa, server):
        part = json.loads(request(server, '/api/part?id=lidar')[1])
        assert part['topics'] == [
            {'topic': '/scan', 'hz': 7.0, 'type': 'sensor_msgs/msg/LaserScan'}
        ]
        assert pa.Handler.live.watched[-1] == ['/scan']
        board = json.loads(request(server, '/api/part?id=teensy_pcb')[1])
        assert board['firmware']['fw.git'] == '0e94b5d'
        assert request(server, '/api/part?id=nope')[0].status == 404

    def test_power_sensor_part(self, pa, server, monkeypatch):
        battery = {'battery': {'volts': 7.5, 'amps': 1.0, 'stale': False}}
        monkeypatch.setattr(pa, 'fetch_home', lambda path, port_=None: battery)
        part = json.loads(request(server, '/api/part?id=power_sensor')[1])
        assert part['deck'] == 'middle' and part['power']['Power (W)'] == 7.5
        assert json.loads(request(server, '/api/part?id=battery')[1])['deck'] == 'middle'

    def test_handbook_menu_route(self, server):
        pages = json.loads(request(server, '/api/handbook')[1])['pages']
        assert 'troubleshooting' not in [p['name'] for p in pages]

    def test_handbook_pages(self, server):
        res, data = request(server, '/handbook/installation')
        assert res.status == 200 and b'/api/handbook/' in data
        page = json.loads(request(server, '/api/handbook/health')[1])
        assert page['title'] == 'Health checks'
        assert request(server, '/handbook/nope')[0].status == 404
        assert request(server, '/handbook')[0].status == 302

    def test_model_with_etag(self, pa, server, monkeypatch, tmp_path):
        assert request(server, '/model')[0].status in (200, 404)
        glb = tmp_path / pa.MODEL_NAME
        glb.write_bytes(b'glTF' + b'\0' * 16)
        monkeypatch.setattr(pa, 'model_path', lambda: glb)
        res, data = request(server, '/model')
        assert res.status == 200 and data.startswith(b'glTF')
        tag = res.getheader('ETag')
        res, data = request(server, '/model', {'If-None-Match': tag})
        assert res.status == 304 and data == b''

    def test_vendored_viewer_served(self, server):
        files = (
            'three.module.min.js',
            'loaders/GLTFLoader.js',
            'controls/OrbitControls.js',
            'utils/BufferGeometryUtils.js',
            'environments/RoomEnvironment.js',
        )
        for path in files:
            res, _ = request(server, f'/ui/vendor/three/{path}')
            assert res.status == 200, path


def test_registered_as_a_core_app():
    registry = yaml.safe_load((ROOT / 'config' / 'apps.yaml').read_text())
    entry = next(a for a in registry['apps'] if a['id'] == 'profile')
    assert entry['desc'] == 'View details about the car'
    assert (
        entry['port'] == 8088 and entry['group'] == 'core' and entry['unit'] == 'racecar-profile'
    )
    assert 'racecar-profile.service' in (ROOT / 'scripts' / 'setup_services.sh').read_text()
    assert (
        '"racecar-profile.service"'
        in (ROOT / 'scripts' / 'polkit' / '49-racecar-restart.rules').read_text()
    )
    unit = (ROOT / 'scripts' / 'racecar-profile.service').read_text()
    assert 'scripts/profile_app.py' in unit and 'WantedBy=multi-user.target' in unit
