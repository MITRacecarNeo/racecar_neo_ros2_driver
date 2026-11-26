"""Shared dashboard assets: scripts/webui.py and scripts/web/."""

from pathlib import Path
import re
from xml.etree import ElementTree

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def webui():
    return load_script('webui')


def test_serves_the_stylesheet(webui):
    body, kind = webui.static_file('/ui/rcn.css')
    assert kind.startswith('text/css') and b'--rcn-orange' in body


def test_serves_fonts_logos_and_icons(webui):
    assert webui.static_file('/ui/fonts/barlow-400.woff2')[1] == 'font/woff2'
    assert webui.static_file('/ui/brand/mark-white.png')[1] == 'image/png'
    assert webui.static_file('/ui/icons.svg?v=2')[1] == 'image/svg+xml'


@pytest.mark.parametrize(
    'path',
    [
        '/index.html',
        '/ui/',
        '/ui/missing.css',
        '/ui/../webui.py',
        '/ui/%2e%2e/webui.py',
        '/ui/fonts/../../dashboard.py',
    ],
)
def test_refuses_anything_else(webui, path):
    assert webui.static_file(path) is None


def test_refuses_unknown_types(webui, tmp_path):
    (tmp_path / 'notes.md').write_text('x')
    assert webui.static_file('/ui/notes.md', root=tmp_path) is None


def test_every_font_face_file_ships(webui):
    css = (webui.WEB_DIR / 'rcn.css').read_text()
    for name in re.findall(r'url\("(fonts/[^"]+)"\)', css):
        assert (webui.WEB_DIR / name).is_file(), name
    for license_file in ('OFL-barlow.txt', 'OFL-barlowsemicondensed.txt', 'OFL-jetbrainsmono.txt'):
        assert (webui.WEB_DIR / 'fonts' / license_file).is_file()


def test_stylesheet_has_no_blue_and_uses_brand_values(webui):
    css = (webui.WEB_DIR / 'rcn.css').read_text().lower()
    assert '#3355aa' not in css  # blue is reserved; never in RACECAR UI
    for value in ('#a01936', '#f49a4b', '#ffb84a', '#2f2d38', '#423f52', '#19686c'):
        assert value in css


def test_icons_parse_and_have_ids(webui):
    root = ElementTree.parse(webui.WEB_DIR / 'icons.svg').getroot()
    ids = {el.get('id') for el in root if el.tag.endswith('symbol')}
    assert {'map', 'notebook', 'gamepad', 'line', 'wall', 'pulse', 'check', 'alert'} <= ids


SHARED = (
    'rcn.css',
    'charts.js',
    'lost.js',
    'override.js',
    'battery.js',
    'stall.js',
    'driving.js',
    'ui.js',
    'api.js',
    'tune.js',
    'logs.js',
)


def test_pages_link_shared_files_with_the_asset_version(webui):
    # A browser keeps /ui/ files for CACHE_CONTROL; the version query, filled
    # in when the page is sent, makes it fetch a changed rcn.css at once.
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    pages = [
        scripts / 'dashboard.html',
        scripts / 'dashboard_launch.html',
        scripts / 'mapview.html',
        scripts / 'dashboards' / 'teleop_dashboard' / 'teleop.html',
        scripts / 'dashboards' / 'linefollow_dashboard' / 'linefollow.html',
        scripts / 'dashboards' / 'wallfollow_dashboard' / 'wallfollow.html',
        scripts / 'dashboards' / 'camlabel_dashboard' / 'camlabel.html',
        scripts / 'terminal.html',
        scripts / 'terminal_login.html',
    ]
    for page in pages:
        text = page.read_text()
        for name in SHARED:
            for link in re.findall(rf'"ui/{re.escape(name)}[^"]*"', text):
                assert link == f'"ui/{name}?v={{{{asset}}}}"', (page.name, link)


def test_lab_pages_show_a_closed_screen():
    # A lab app stopped from the apps page leaves its tab open; lost.js covers it.
    dashboards = Path(__file__).resolve().parent.parent / 'scripts' / 'dashboards'
    for page in (
        'teleop_dashboard/teleop.html',
        'linefollow_dashboard/linefollow.html',
        'wallfollow_dashboard/wallfollow.html',
        'camlabel_dashboard/camlabel.html',
    ):
        text = (dashboards / page).read_text()
        assert 'ui/lost.js?v=' in text and 'data-app="' in text, page


def test_driving_lab_pages_keep_the_override_alive():
    dashboards = Path(__file__).resolve().parent.parent / 'scripts' / 'dashboards'
    for page in (
        'teleop_dashboard/teleop.html',
        'linefollow_dashboard/linefollow.html',
        'wallfollow_dashboard/wallfollow.html',
    ):
        assert 'ui/override.js?v=' in (dashboards / page).read_text(), page


PAGES = (
    'dashboard.html',
    'dashboard_launch.html',
    'mapview.html',
    'planner.html',
    'dashboards/teleop_dashboard/teleop.html',
    'dashboards/linefollow_dashboard/linefollow.html',
    'dashboards/wallfollow_dashboard/wallfollow.html',
    'dashboards/camlabel_dashboard/camlabel.html',
    'terminal.html',
)


def test_app_pages_show_the_low_battery_banner():
    # 8080 included: it feeds the shared banner from its own status poll.
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    for page in PAGES:
        assert 'ui/battery.js?v=' in (scripts / page).read_text(), page


def test_every_page_shows_the_stall_banner():
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    for page in (*PAGES, 'terminal_login.html'):
        assert 'ui/stall.js?v=' in (scripts / page).read_text(), page


def test_driving_pages_show_who_drives():
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    driving = {
        'dashboard.html': '',
        'mapview.html': 'Mapper',
        'planner.html': 'Planner',
        'dashboards/teleop_dashboard/teleop.html': 'Web Teleop',
        'dashboards/linefollow_dashboard/linefollow.html': 'Line Follow',
        'dashboards/wallfollow_dashboard/wallfollow.html': 'Wall Follow',
    }
    for page in PAGES:
        text = (scripts / page).read_text()
        assert ('ui/driving.js?v=' in text) == (page in driving), page
        if driving.get(page):
            assert f'<html lang="en" data-app="{driving[page]}">' in text, page
    for page in ('mapview.html', 'planner.html'):
        text = (scripts / page).read_text()
        assert 'canDrive() && !(window.RcnDriving && RcnDriving.blocked())' in text


def test_override_banner_only_on_8080_and_driving_pages():
    # Every page that loads override.js keeps the override alive.
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    driving = {
        'dashboard.html',
        'mapview.html',
        'planner.html',
        'dashboards/teleop_dashboard/teleop.html',
        'dashboards/linefollow_dashboard/linefollow.html',
        'dashboards/wallfollow_dashboard/wallfollow.html',
    }
    for page in (*PAGES, 'terminal_login.html'):
        assert ('ui/override.js?v=' in (scripts / page).read_text()) == (page in driving), page


class TestFill:
    def test_page_gets_the_version_and_home_port(self, webui, tmp_path):
        page = tmp_path / 'p.html'
        page.write_text('<link href="/ui/rcn.css?v={{asset}}"> :{{home}}/')
        text = webui.read_page(page)
        assert f'?v={webui.ASSET_VERSION}"' in text and ':8080/' in text

    def test_unreadable_page_is_an_error_page(self, webui, tmp_path):
        assert 'unreadable' in webui.read_page(tmp_path / 'absent.html')

    def test_every_page_gets_the_footer_and_home_shortcut(self, webui, tmp_path):
        page = tmp_path / 'p.html'
        page.write_text('<body><main></main></body>')
        text = webui.read_page(page)
        for name in ('footer.js', 'home.js'):
            assert text.index(f'ui/{name}?v={webui.ASSET_VERSION}') < text.index('</body>')
        bare = tmp_path / 'bare.html'
        bare.write_text('<main></main>')
        assert webui.read_page(bare).endswith(
            webui.PAGE_TAGS.replace('{{asset}}', webui.ASSET_VERSION)
        )

    def test_driver_version_is_the_package_version(self, webui):
        import re

        xml = webui.PACKAGE_XML.read_text()
        assert webui.driver_version() == re.search(r'<version>([^<]+)<', xml).group(1)

    def test_home_shortcut_closes_the_tab_or_loads_the_home_page(self, webui):
        body, _ = webui.static_file('/ui/home.js')
        text = body.decode()
        assert '{{' not in text and "RcnRoute.app('home')" in text
        assert "hasAttribute('data-home')" in text
        dashboard = (webui.WEB_DIR.parent / 'dashboard.html').read_text()
        assert '<body data-home>' in dashboard
        assert 'window.close()' in text and 'location.href = home' in text
        assert "querySelector('.rcn-header .rcn-mark')" in text

    def test_footer_script_gets_the_versions_and_home_port(self, webui):
        body, _ = webui.static_file('/ui/footer.js')
        text = body.decode()
        assert '{{' not in text and "RcnRoute.app('home', 'api/about')" in text
        assert f'Driver {webui.driver_version()}' in text
        assert 'racecar OS' in text and 'Copyright' not in text

    @pytest.mark.parametrize(
        'name', ['battery.js', 'override.js', 'lost.js', 'stall.js', 'driving.js']
    )
    def test_shared_scripts_reach_the_home_page(self, webui, name):
        raw = (webui.WEB_DIR / name).read_text()
        assert "RcnRoute.app('home'" in raw and ':8080/' not in raw
        body, _ = webui.static_file(f'/ui/{name}')
        assert b'{{' not in body


class TestOrigins:
    @pytest.mark.parametrize(
        'origin, host, allow_missing, ok',
        [
            ('http://10.42.0.1:8086', '10.42.0.1:8086', False, True),
            ('http://evil.example', '10.42.0.1:8086', False, False),
            ('http://10.42.0.1:8080', '10.42.0.1:8086', False, False),
            (None, '10.42.0.1:8086', False, False),
            (None, '10.42.0.1:8086', True, True),
            ('http://10.42.0.1:8086', None, True, False),
        ],
    )
    def test_same_origin(self, webui, origin, host, allow_missing, ok):
        assert webui.same_origin(origin, host, allow_missing) is ok

    @pytest.mark.parametrize(
        'origin, ok',
        [
            ('http://car.local:8081', True),
            ('http://car.local:8089', False),
            ('https://car.local:8081', False),
            ('http://other:8081', False),
            (None, False),
        ],
    )
    def test_sibling_origin(self, webui, origin, ok):
        assert (webui.sibling_origin(origin, 'car.local:8080', [8081, 8082]) is not None) is ok


class TestSpinUntilStopped:
    def test_error_does_not_end_the_spinner(self, webui):
        calls = []

        def spin_once():
            calls.append(1)
            if len(calls) == 2:
                raise RuntimeError('cannot use Destroyable because destruction was requested')

        errors = webui.spin_until_stopped(spin_once, lambda: len(calls) < 5)
        assert errors == 1 and len(calls) == 5

    def test_stops_when_ros_shuts_down(self, webui):
        assert webui.spin_until_stopped(lambda: None, lambda: False) == 0

    def test_error_from_the_shutdown_is_not_counted(self, webui):
        state = {'ok': True}

        def spin_once():
            state['ok'] = False
            raise RuntimeError('context is not valid')

        assert webui.spin_until_stopped(spin_once, lambda: state['ok']) == 0

    def test_keyboard_interrupt_ends_it(self, webui):
        def spin_once():
            raise KeyboardInterrupt

        assert webui.spin_until_stopped(spin_once, lambda: True) == 0


@pytest.fixture
def served(webui):
    """Run an AppHandler on a free port; yield its port."""
    import threading
    import time

    release = threading.Event()

    class Handler(webui.AppHandler):
        def do_GET(self):
            if self.serve_asset():
                return
            if self.path == '/slow':
                release.wait(5)
            self.send(200, b'ok', 'text/plain')

        def do_POST(self):
            try:
                self.send_json({'got': self.read_json()})
            except ValueError:
                self.send(400, b'bad', 'text/plain')

    from http.server import ThreadingHTTPServer

    srv = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.05)
    yield srv.server_address[1], release
    release.set()
    srv.shutdown()
    srv.server_close()


class TestAppHandler:
    def test_ignored_body_leaves_the_connection_clean(self, served):
        # Keep-alive: a body the handler never reads must not become the
        # start of the next request on the same connection.
        import http.client

        port, _ = served
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request(
            'GET', '/', body=b'{"stray": 1}', headers={'Content-Type': 'application/json'}
        )
        first = conn.getresponse()
        assert first.status == 200 and first.read() == b'ok'
        conn.request('GET', '/')
        second = conn.getresponse()
        assert second.status == 200 and second.read() == b'ok'
        conn.close()

    def test_read_json(self, served):
        import http.client
        import json

        port, _ = served
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request('POST', '/', body=b'{"a": 1}')
        assert json.loads(conn.getresponse().read()) == {'got': {'a': 1}}
        conn.request('POST', '/', body=b'')
        assert json.loads(conn.getresponse().read()) == {'got': {}}
        conn.request('POST', '/', body=b'[1, 2]')
        res = conn.getresponse()
        assert res.status == 400
        res.read()
        conn.close()

    def test_refuses_a_large_body(self, served, webui):
        import http.client

        port, _ = served
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request('POST', '/', body=b'x' * (webui.MAX_BODY + 1))
        assert conn.getresponse().status == 413
        conn.close()

    def test_assets_are_cached_for_a_while(self, served, webui):
        import http.client

        port, _ = served
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request('GET', '/ui/rcn.css')
        res = conn.getresponse()
        assert res.status == 200 and res.getheader('Cache-Control') == webui.CACHE_CONTROL
        res.read()
        conn.close()


class TestServe:
    def test_answers_while_another_request_waits(self, webui):
        # A client that holds its request open must not hold up the others.
        import http.client
        import signal
        import threading
        import time

        release = threading.Event()
        bound = {}

        class Handler(webui.AppHandler):
            def do_GET(self):
                if self.path == '/slow':
                    release.wait(5)
                self.send(200, b'ok', 'text/plain')

        stops, cleanups = [], []
        runner = threading.Thread(
            target=webui.serve,
            args=(0, Handler),
            kwargs={
                'bind': '127.0.0.1',
                'on_stop': lambda: stops.append(1),
                'cleanup': lambda: cleanups.append(1),
                'ready': lambda srv: bound.update(port=srv.server_address[1]),
            },
        )
        old = signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)
        # signal.signal only works on the main thread; serve() runs here on
        # another, so give it a stand-in that records the handler.
        installed = {}
        real_signal = signal.signal
        signal.signal = lambda sig, fn: installed.__setitem__(sig, fn)
        try:
            runner.start()
            for _ in range(100):
                if 'port' in bound and signal.SIGTERM in installed:
                    break
                time.sleep(0.02)
            slow = http.client.HTTPConnection('127.0.0.1', bound['port'], timeout=5)
            slow.request('GET', '/slow')
            fast = http.client.HTTPConnection('127.0.0.1', bound['port'], timeout=2)
            fast.request('GET', '/')
            assert fast.getresponse().read() == b'ok'
            release.set()
            assert slow.getresponse().read() == b'ok'
            installed[signal.SIGTERM](signal.SIGTERM, None)
            runner.join(timeout=5)
            assert not runner.is_alive()
            assert stops == [1] and cleanups == [1]
        finally:
            signal.signal = real_signal
            real_signal(signal.SIGTERM, old[0])
            real_signal(signal.SIGINT, old[1])


LABS = {
    'teleop': 'teleop_dashboard/teleop',
    'linefollow': 'linefollow_dashboard/linefollow',
    'wallfollow': 'wallfollow_dashboard/wallfollow',
}


def _lab_page(name):
    return (
        Path(__file__).resolve().parent.parent / 'scripts' / 'dashboards' / f'{LABS[name]}.html'
    ).read_text()


class TestPageKit:
    @pytest.mark.parametrize('name', list(LABS))
    def test_labs_use_the_shared_kit(self, name):
        text = _lab_page(name)
        for script in ('ui.js', 'api.js', 'tune.js', 'logs.js'):
            assert f'ui/{script}?v=' in text, script
        for gone in (
            'function post(',
            'const $ = (id)',
            'let busy = false',
            'function loadLog(',
            '.card {',
            "'#f49a4b'",
            'Barlow, Arial',
        ):
            assert gone not in text, gone
        assert 'RcnApi.poll(' in text and 'RcnLogs.mount(' in text and 'RcnTune.sliders(' in text

    def test_wallfollow_tunes_with_sliders(self):
        text = _lab_page('wallfollow')
        assert 'type="number"' not in text and 'Apply' not in text

    @pytest.mark.parametrize('name', list(LABS))
    def test_every_slider_is_a_setting_the_app_takes(self, name):
        from conftest import load_dashboard

        module = load_dashboard(name)
        keys = set(re.findall(r"\{ key: '(\w+)'", _lab_page(name)))
        keys |= set(re.findall(r"(?:lo|hi): '(\w+)'", _lab_page(name)))
        assert keys and keys <= set(module.TUNE_KEYS), keys - set(module.TUNE_KEYS)


class TestIdleGate:
    def test_opens_once_and_closes_when_idle(self, webui):
        t, calls = [0.0], []
        gate = webui.IdleGate(
            10.0, lambda: calls.append('open'), lambda: calls.append('close'), lambda: t[0]
        )
        assert gate.touch() is True and gate.touch() is False
        t[0] = 9.0
        assert gate.check() and calls == ['open']
        gate.touch()
        t[0] = 18.0
        assert gate.check()
        t[0] = 19.5
        assert not gate.check() and calls == ['open', 'close']
        assert gate.touch() is True and calls[-1] == 'open'

    def test_without_callbacks_it_only_tracks_viewing(self, webui):
        t = [100.0]
        gate = webui.IdleGate(15.0, clock=lambda: t[0])
        assert not gate.check()
        gate.touch()
        t[0] += 16.0
        assert not gate.check()


def test_every_app_refuses_cross_site_posts():
    """Pages that drive the car or change its files refuse another site's POST."""
    from pathlib import Path

    import yaml

    root = Path(__file__).resolve().parent.parent
    scripts = root / 'scripts'
    registry = yaml.safe_load((root / 'config' / 'apps.yaml').read_text())
    labs = [
        scripts / 'dashboards' / a['dir'] / a['script'] for a in registry['apps'] if 'dir' in a
    ]
    assert labs
    for path in [scripts / 'mapview.py', *labs]:
        text = path.read_text()
        if 'def do_POST' in text:
            assert 'self.refuse_cross_site()' in text, path.name


def test_refuse_cross_site(webui):
    import http.client
    from http.server import ThreadingHTTPServer
    import threading

    class Handler(webui.AppHandler):
        def do_POST(self):  # noqa: N802
            if self.refuse_cross_site():
                return
            self.send(200, b'ok', 'text/plain')

    srv = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]

    def post(headers):
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request('POST', '/x', body=b'{}', headers=headers)
        res = conn.getresponse()
        res.read()
        conn.close()
        return res.status

    try:
        assert post({'Origin': 'http://evil.example'}) == 403
        assert post({'Origin': f'http://127.0.0.1:{port}'}) == 200
        assert post({}) == 200
    finally:
        srv.shutdown()
        srv.server_close()


def test_icon_links_carry_the_asset_version():
    """A new icon shows right after an update: no page or script links icons.svg unversioned."""
    from pathlib import Path

    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    files = [
        *scripts.glob('*.html'),
        *scripts.glob('dashboards/*/*.html'),
        *scripts.glob('web/*.js'),
    ]
    for path in files:
        if path.parent.name in ('eps_dashboard', 'pursuit_dashboard', 'smartfollow_dashboard'):
            continue
        assert 'ui/icons.svg#' not in path.read_text(), path.name


TOP_PAGES = sorted((Path(__file__).resolve().parent.parent / 'scripts').glob('*.html'))


@pytest.mark.parametrize('page', TOP_PAGES, ids=lambda p: p.name)
def test_import_map_addresses_resolve(page):
    # An import map ignores an address that is not ./, ../, / or a full URL.
    text = page.read_text()
    for block in re.findall(r'<script type="importmap">(.*?)</script>', text, re.S):
        for target in re.findall(r':\s*"([^"]+)"', block):
            assert target.startswith(('./', '../', '/', 'http')), (page.name, target)


def _handler(webui, peer='127.0.0.1', **headers):
    h = object.__new__(webui.AppHandler)
    h.client_address = (peer, 50000)
    h.headers = headers
    h.sent = []
    h.send = lambda code, body, kind, headers=None: h.sent.append((code, kind, headers or {}))
    return h


class TestNamePaths:
    def test_base_goes_right_after_head(self, webui):
        out = webui.with_base('<html><head lang="en"><title>x</title></head></html>', '/mapper/')
        head = '<html><head lang="en"><base href="/mapper/"><script src="ui/route.js?v='
        assert out.startswith(head)
        assert out.index('<base') < out.index('<title>')

    def test_base_without_head(self, webui):
        assert webui.with_base('<p>x</p>', '/').startswith('<base href="/">')

    @pytest.mark.parametrize(
        'value, expected',
        [
            (None, '/'),
            ('/mapper/', '/mapper/'),
            ('/mapper', '/'),
            ('/a/b/', '/'),
            ('/"><script>/', '/'),
            ('//evil.example/', '/'),
            ('/mapper/\n', '/'),
        ],
    )
    def test_prefix_is_validated(self, webui, value, expected):
        headers = {} if value is None else {'X-Forwarded-Prefix': value}
        assert _handler(webui, **headers).prefix() == expected

    @pytest.mark.parametrize(
        'peer, forwarded, expected',
        [
            ('127.0.0.1', '10.42.0.5', '10.42.0.5'),
            ('::1', '10.42.0.5', '10.42.0.5'),
            ('10.42.0.5', '127.0.0.1', '10.42.0.5'),
            ('127.0.0.1', 'not an address', '127.0.0.1'),
            ('127.0.0.1', '', '127.0.0.1'),
        ],
    )
    def test_client_ip_trusts_only_the_local_proxy(self, webui, peer, forwarded, expected):
        assert _handler(webui, peer, **{'X-Real-IP': forwarded}).client_ip() == expected

    def test_redirect_stays_under_the_prefix(self, webui):
        h = _handler(webui, **{'X-Forwarded-Prefix': '/profile/'})
        h.redirect('/handbook/index', 303, {'Set-Cookie': 'a=b'})
        code, _, headers = h.sent[0]
        assert code == 303
        assert headers == {'Location': '/profile/handbook/index', 'Set-Cookie': 'a=b'}
        h = _handler(webui)
        h.redirect('login')
        assert h.sent[0][2]['Location'] == '/login'

    def test_route_script_gets_the_routes(self, webui):
        body, kind = webui.static_file('/ui/route.js')
        assert kind.startswith(('text/javascript', 'application/javascript'))
        assert b'{{routes}}' not in body and b'"mapview": ["mapper"' in body


@pytest.mark.parametrize('host', ['10.42.0.1', '192.168.52.200', 'racecar-neo-0.local'])
def test_port_80_pages_are_same_origin(webui, host):
    # nginx passes Host without a port; the browser's Origin on port 80 has none either.
    assert webui.same_origin(f'http://{host}', host)
    assert not webui.same_origin('http://other.local', host)
