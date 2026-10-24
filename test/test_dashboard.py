"""Unit tests for scripts/dashboard.py."""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from types import SimpleNamespace

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def dashboard():
    return load_script('dashboard')


class TestMonitoredAndRateTopics:
    EXPECTED_NODES = {
        'led',
        'pit',
        'throttle',
        'mux',
        'gamepad',
        'imu_fusion',
        'lidar',
        'realsense',
        'edgetpu',
        'dotmatrix',
    }

    def test_monitored_covers_all_subsystems(self, dashboard):
        # Includes nodes the watchdog does not supervise (edgetpu, dotmatrix).
        assert set(dashboard.MONITORED) == self.EXPECTED_NODES

    @pytest.mark.parametrize('name', sorted(EXPECTED_NODES))
    def test_monitored_entry_has_label_and_topic(self, dashboard, name):
        cfg = dashboard.MONITORED[name]
        assert 'label' in cfg and cfg['label']
        assert 'topic' in cfg and cfg['topic'].startswith('/')

    def test_rate_topics_subset_of_known_publishers(self, dashboard):
        # The RealSense card watches /camera/color; depth and its IMU are
        # rate-monitored too.
        known = {cfg['topic'] for cfg in dashboard.MONITORED.values()}
        known |= {'/camera/depth', '/imu/realsense'}
        for t in dashboard.DISPLAY_RATE_TOPICS:
            assert t in known, f'{t} in the rates table but no known node publishes it'


class TestGetStatus:
    def test_returns_dict_with_required_keys(self, dashboard):
        snapshot = dashboard.get_status()
        for key in (
            'timestamp',
            'nodes',
            'node_list',
            'topic_list',
            'rates',
            'watchdog_log',
            'log_dir',
            'autonomy',
        ):
            assert key in snapshot

    def test_status_is_json_serializable(self, dashboard):
        # /api/status serves this through json.dumps.
        json.dumps(dashboard.get_status())


class _Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def monotonic(self) -> float:
        return self.t


@pytest.fixture
def sampler(dashboard, monkeypatch):
    """Build a _RateSampler with its bookkeeping but no rclpy node behind it."""
    clock = _Clock()
    monkeypatch.setattr(dashboard, 'time', clock)
    s = dashboard._RateSampler.__new__(dashboard._RateSampler)
    s._window = 3.0
    s._stamps = {'/scan': deque(), '/motor': deque()}
    s._lock = threading.Lock()
    s._diagnostic_rates = {}
    s._diagnostic_last_update = {}
    return s, clock


def _diagnostics(name, rate):
    value = SimpleNamespace(key='Actual frequency (Hz)', value=rate)
    return SimpleNamespace(status=[SimpleNamespace(name=name, values=[value])])


class TestRateSampler:
    def test_rate_is_arrivals_over_the_window(self, sampler):
        s, clock = sampler
        for i in range(30):
            clock.t = 1000.0 + i * 0.1
            s._record('/scan')
        clock.t = 1002.95
        assert s.measure_hz('/scan') == pytest.approx(10.0)

    def test_arrivals_older_than_the_window_are_dropped(self, sampler):
        s, clock = sampler
        for _ in range(30):
            s._record('/scan')
        clock.t += 3.5
        assert s.measure_hz('/scan') is None

    def test_a_single_arrival_is_not_a_rate(self, sampler):
        s, _ = sampler
        s._record('/motor')
        assert s.measure_hz('/motor') is None

    def test_unknown_topic_is_none(self, sampler):
        s, _ = sampler
        assert s.measure_hz('/nope') is None

    def test_realsense_rate_comes_from_diagnostics(self, sampler):
        s, _ = sampler
        s._record_diagnostics(_diagnostics('camera: color', '59.4'))
        assert s.measure_hz('/camera/color') == pytest.approx(59.4)

    def test_stale_diagnostics_read_as_no_data(self, sampler):
        s, clock = sampler
        s._record_diagnostics(_diagnostics('camera: depth', '29.8'))
        clock.t += 3.5
        assert s.measure_hz('/camera/depth') is None

    def test_realsense_without_diagnostics_is_none(self, sampler):
        s, _ = sampler
        assert s.measure_hz('/imu/realsense') is None


class TestMeasureHz:
    def test_no_sampler_reads_as_no_data(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_sampler', None)
        assert dashboard._measure_hz('/scan') is None

    def test_delegates_to_the_sampler(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_sampler', SimpleNamespace(measure_hz=lambda t: 7.2))
        assert dashboard._measure_hz('/scan') == 7.2


class TestSystemHealth:
    def test_collect_system_health_keys(self, dashboard):
        # Without vcgencmd or rpi_volt both entries still exist, as unavailable.
        health = dashboard._collect_system_health()
        assert set(health) == {'rtc', 'under_voltage'}
        for entry in health.values():
            assert {'label', 'status', 'detail'} <= set(entry)
            assert entry['status'] in ('healthy', 'stale', 'dead')

    @pytest.mark.parametrize(
        'alarm,status,detail',
        [
            (None, 'dead', 'UNAVAILABLE'),
            (True, 'dead', 'TRIPPED (5V dipped this boot)'),
            (False, 'healthy', 'OK'),
        ],
    )
    def test_under_voltage_states(self, dashboard, monkeypatch, alarm, status, detail):
        monkeypatch.setattr(dashboard, '_read_under_voltage_alarm', lambda: alarm)
        monkeypatch.setattr(dashboard, '_read_battery_voltage', lambda: 2.9)
        entry = dashboard._collect_system_health()['under_voltage']
        assert (entry['status'], entry['detail']) == (status, detail)


class TestDashboardHTML:
    def test_html_template_present(self, dashboard):
        assert '<!DOCTYPE html>' in dashboard.DASHBOARD_HTML

    def test_html_references_api_endpoint(self, dashboard):
        html = dashboard.DASHBOARD_HTML
        assert "'/api/status'" in html and "'/api/status?refresh=1'" in html

    def test_title_says_racecar(self, dashboard):
        html = dashboard.DASHBOARD_HTML
        assert 'RACECAR Neo' in html
        assert 'UAV Neo' not in html

    def test_system_health_section_present(self, dashboard):
        # The target div and the JS field name must agree.
        html = dashboard.DASHBOARD_HTML
        assert 'id="system-health"' in html
        assert 'data.system_health' in html


class TestConfig:
    def test_port_matches_the_service(self, dashboard):
        assert dashboard.PORT == 8080


class TestWatchdogView:
    def test_no_watchdog_state_keeps_the_topic_status(self, dashboard):
        assert dashboard.watchdog_view('dead', None) == ('dead', '')

    def test_healthy_keeps_the_topic_status(self, dashboard):
        assert dashboard.watchdog_view('healthy', {'state': 'healthy'}) == ('healthy', '')

    def test_gave_up_is_dead_with_advice(self, dashboard):
        status, detail = dashboard.watchdog_view('dead', {'state': 'gave_up'})
        assert status == 'dead' and 'gave up' in detail

    def test_no_device_is_amber(self, dashboard):
        assert dashboard.watchdog_view('dead', {'state': 'no_device'})[0] == 'stale'

    def test_restarting_shows_the_count(self, dashboard):
        status, detail = dashboard.watchdog_view('dead', {'state': 'restarting', 'restarts': 3})
        assert status == 'stale' and '3' in detail

    def test_disabled_is_gray(self, dashboard):
        assert dashboard.watchdog_view('dead', {'state': 'disabled'})[0] == 'unsupervised'


class TestAutonomyView:
    def test_stopped_or_never_started_is_gray(self, dashboard):
        for values in (None, {'mode': 'off', 'map': 'room'}):
            view = dashboard.autonomy_view(values, 1.0)
            assert view['status'] == 'unsupervised' and view['headline'] == 'Stopped'
            assert view['port'] == 8084

    def test_localized_is_healthy_with_fit_and_spread(self, dashboard):
        values = {
            'mode': 'localization', 'map': 'room', 'localizer': 'amcl', 'localized': 'true',
            'scan_fit': '0.81', 'pose_sd_m': '0.12', 'pose_sd_deg': '5.3',
        }  # fmt: skip
        view = dashboard.autonomy_view(values, 0.5)
        assert view['status'] == 'healthy'
        assert view['headline'] == 'Localization on room (amcl)'
        assert view['lines'] == ['Localized', 'Scan fit 81%', 'Spread 0.12 m, 5 deg']

    def test_not_localized_is_yellow(self, dashboard):
        view = dashboard.autonomy_view({'mode': 'localization', 'localized': 'false'}, 0.5)
        assert view['status'] == 'stale' and view['lines'][0] == 'Not localized'

    def test_mapping_shows_the_save(self, dashboard):
        values = {'mode': 'mapping', 'map': 'hall', 'localized': 'true'}
        view = dashboard.autonomy_view(values, 0.5)
        assert view['headline'] == 'Mapping on hall' and view['lines'][-1] == 'Saved not yet'

    def test_old_status_is_red(self, dashboard):
        view = dashboard.autonomy_view({'mode': 'mapping', 'map': 'hall'}, 12.0)
        assert view['status'] == 'dead' and '12 s' in view['lines'][0]

    def test_html_has_the_autonomy_section(self, dashboard):
        html = (Path(dashboard.__file__).parent / 'dashboard.html').read_text()
        assert 'id="autonomy"' in html and 'data.autonomy' in html


class FakeRun:
    """subprocess.run stand-in: answers is-active from `states`, records the rest."""

    def __init__(self, states, fail=None, stderr=''):
        self.states, self.fail, self.stderr, self.calls = dict(states), fail, stderr, []

    def __call__(self, argv, **_):
        self.calls.append(argv)
        if argv[1] == 'is-active':
            out = '\n'.join(self.states.get(u, 'inactive') for u in argv[2:])
            return SimpleNamespace(stdout=out, returncode=0, stderr='')
        verb, unit = argv[-2], argv[-1]
        if self.fail == verb:
            return SimpleNamespace(stdout='', returncode=1, stderr=self.stderr)
        self.states[unit] = 'active' if verb == 'start' else 'inactive'
        return SimpleNamespace(stdout='', returncode=0, stderr='')


class TestHome:
    def test_apps_cover_the_six_dashboards(self, dashboard):
        assert [a['port'] for a in dashboard.APPS] == [None, 8888, 8084, 8081, 8082, 8083, 8085]
        core = [a['id'] for a in dashboard.APPS if a['group'] == 'core']
        assert core == ['terminator', 'jupyter', 'mapview']
        exclusive = [a['id'] for a in dashboard.APPS if a['exclusive']]
        assert exclusive == ['webteleop', 'linefollow', 'wallfollow']
        startable = [a['id'] for a in dashboard.APPS if a['startable']]
        assert startable == ['webteleop', 'linefollow', 'wallfollow', 'camlabel']

    def test_every_app_icon_exists(self, dashboard):
        scripts = Path(__file__).resolve().parent.parent / 'scripts'
        icons = (scripts / 'web' / 'icons.svg').read_text()
        for app in dashboard.APPS:
            assert f'id="{app["icon"]}"' in icons, app['icon']

    def test_terminator_is_a_placeholder(self, dashboard):
        run = FakeRun({})
        assert dashboard.open_app('terminator', run) == (False, 'Terminator is not available.')
        assert run.calls == []
        states = dashboard.unit_states(dashboard._app_units(), FakeRun({}))
        apps = {a['id']: a for a in dashboard.app_list(states)}
        assert apps['terminator']['state'] == 'unavailable'

    def test_open_exclusive_stops_the_other_first(self, dashboard):
        run = FakeRun({'racecar-webteleop': 'active'})
        ok, _ = dashboard.open_app('linefollow', run)
        assert ok
        verbs = [(c[-2], c[-1]) for c in run.calls if c[1] == '--no-ask-password']
        assert verbs == [('stop', 'racecar-webteleop'), ('start', 'racecar-linefollow')]

    def test_open_running_exclusive_does_nothing(self, dashboard):
        run = FakeRun({'racecar-wallfollow': 'active'})
        assert dashboard.open_app('wallfollow', run)[0]
        assert not [c for c in run.calls if c[1] == '--no-ask-password']

    def test_open_camlabel_leaves_the_driving_app_running(self, dashboard):
        run = FakeRun({'racecar-webteleop': 'active'})
        ok, message = dashboard.open_app('camlabel', run)
        assert ok and message == 'Camera label started.'
        assert run.states['racecar-webteleop'] == 'active'
        assert run.states['racecar-camlabel'] == 'active'

    def test_open_stopped_core_app_names_the_command(self, dashboard):
        run = FakeRun({'racecar-mapview': 'inactive'})
        ok, message = dashboard.open_app('mapview', run)
        assert not ok and 'racecar service start mapview' in message
        assert not [c for c in run.calls if c[1] == '--no-ask-password']

    def test_clear_apps_stops_utilities_only(self, dashboard):
        run = FakeRun({'racecar-mapview': 'active', 'racecar-jupyter': 'active',
                       'racecar-webteleop': 'active', 'racecar-camlabel': 'active',
                       'racecar-dashboard': 'active', 'racecar-teleop': 'active'})
        assert dashboard.clear_apps(run) == (True, 'Utilities stopped.')
        assert run.states['racecar-webteleop'] == 'inactive'
        assert run.states['racecar-camlabel'] == 'inactive'
        for unit in ('racecar-mapview', 'racecar-jupyter', 'racecar-dashboard', 'racecar-teleop'):
            assert run.states[unit] == 'active'

    def test_clear_apps_names_the_unit_it_may_not_stop(self, dashboard):
        stderr = 'Failed to stop: Interactive authentication required.'
        run = FakeRun({'racecar-linefollow': 'active'}, fail='stop', stderr=stderr)
        ok, message = dashboard.clear_apps(run)
        assert not ok and 'racecar-linefollow' in message and 'setup_user_env.sh' in message

    def test_clear_apps_with_none_running(self, dashboard):
        assert dashboard.clear_apps(FakeRun({})) == (True, 'No utilities running.')

    def test_clear_log_hides_earlier_lines_and_keeps_the_file(self, dashboard, tmp_path,
                                                              monkeypatch):
        log = tmp_path / 'watchdog.log'
        log.write_text('old 1\nold 2\n')
        monkeypatch.setattr(dashboard, 'WATCHDOG_LOG', log)
        monkeypatch.setattr(dashboard, 'WATCHDOG_CLEARED', tmp_path / 'watchdog.cleared')
        assert dashboard._read_watchdog_tail() == ['old 1', 'old 2']
        assert dashboard.clear_watchdog_log() == (True, 'Log cleared.')
        assert dashboard._read_watchdog_tail() == []
        with log.open('a') as f:
            f.write('new 1\n')
        assert dashboard._read_watchdog_tail() == ['new 1']
        assert log.read_text().startswith('old 1')
        log.write_text('fresh\n')  # a new session's log is shorter than the mark
        assert dashboard._read_watchdog_tail() == ['fresh']

    def test_open_refuses_unknown(self, dashboard):
        assert dashboard.open_app('../x', FakeRun({})) == (False, 'Unknown app: ../x')

    def test_app_ready_needs_an_answer(self, dashboard):
        server = ThreadingHTTPServer(('127.0.0.1', 0), BaseHTTPRequestHandler)
        port = server.server_address[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            assert dashboard.app_ready(port)
        finally:
            server.shutdown()
            server.server_close()
        assert not dashboard.app_ready(port, timeout=0.2)

    def test_launch_page_opens_then_waits(self, dashboard):
        html = dashboard._read_page(dashboard._LAUNCH_PATH)
        assert "fetch('/api/open'" in html and '/api/ready?app=' in html
        assert 'location.replace' in html and 'id="cancel"' in html
        assert 'window.close()' in html
        assert 'Try again' not in html

    def test_unit_states_in_order(self, dashboard):
        run = FakeRun({'racecar-mapview': 'active', 'racecar-jupyter': 'failed'})
        states = dashboard.unit_states(['racecar-mapview', 'racecar-jupyter', 'racecar-x'], run)
        assert states == {
            'racecar-mapview': 'active', 'racecar-jupyter': 'failed', 'racecar-x': 'inactive'
        }
        apps = dashboard.app_list(states)
        by_id = {a['id']: a['state'] for a in apps}
        assert by_id['mapview'] == 'active' and by_id['webteleop'] == 'unknown'
        assert by_id['terminator'] == 'unavailable'

    def test_activate_stops_the_running_lab_first(self, dashboard):
        run = FakeRun({'racecar-webteleop': 'active', 'racecar-mapview': 'active'})
        ok, message = dashboard.activate('wallfollow', run)
        assert ok and message == 'Wall follow active.'
        verbs = [(c[-2], c[-1]) for c in run.calls if c[1] == '--no-ask-password']
        assert verbs == [('stop', 'racecar-webteleop'), ('start', 'racecar-wallfollow')]
        assert run.states['racecar-mapview'] == 'active'  # core services untouched

    def test_activate_none_stops_every_lab(self, dashboard):
        run = FakeRun({'racecar-linefollow': 'active'})
        assert dashboard.activate('none', run) == (True, 'Lab dashboards stopped.')
        assert run.states['racecar-linefollow'] == 'inactive'

    def test_activate_names_the_missing_permission(self, dashboard):
        stderr = 'Failed to start: Interactive authentication required.'
        run = FakeRun({}, fail='start', stderr=stderr)
        ok, message = dashboard.activate('linefollow', run)
        assert not ok and message == (
            'No permission to start racecar-linefollow. Run scripts/setup_user_env.sh.'
        )

    def test_activate_refuses_unknown_and_core_units(self, dashboard):
        run = FakeRun({})
        for target in ('mapview', 'jupyter', 'camlabel', '../x', ''):
            ok, _ = dashboard.activate(target, run)
            assert not ok
        assert all(c[1] == 'is-active' for c in run.calls) or run.calls == []

    def test_polkit_rule_scope(self):
        scripts = Path(__file__).resolve().parent.parent / 'scripts'
        text = (scripts / 'polkit' / '49-racecar-dashboards.rules').read_text()
        for unit in ('racecar-webteleop', 'racecar-linefollow', 'racecar-wallfollow',
                     'racecar-camlabel'):
            assert f'"{unit}.service"' in text
        assert '["start", "stop"]' in text and 'subject.user === "racecar"' in text
        for core in ('racecar-dashboard', 'racecar-teleop', 'racecar-autonomy',
                     'racecar-mapview', 'racecar-jupyter'):
            assert core not in text
        assert 'enable' not in text.split('polkit.addRule')[1]
        assert '49-racecar-dashboards.rules' in (scripts / 'setup_user_env.sh').read_text()

    def test_page_uses_the_shared_assets(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert '/ui/rcn.css' in html and '/ui/brand/' in html and '/launch?app=' in html
        assert html.count('/ui/brand/mark.png') == 1, 'one logo on the page, in color'
        assert '/ui/brand/lockup' not in html


class TestHistory:
    def test_window_drops_old_samples(self, dashboard):
        h = dashboard.History(window_sec=100)
        h.add(0.0, 8.0, 1.0)
        h.add(150.0, 7.9, 2.0)
        assert h.buckets(150.0, bucket_sec=10) == [[155.0, 7.9, 2.0]]

    def test_buckets_average_each_interval(self, dashboard):
        h = dashboard.History(window_sec=1000)
        for t, v, a in ((0, 8.0, 1.0), (5, 8.2, 3.0), (12, 7.0, 0.5)):
            h.add(float(t), v, a)
        assert h.buckets(20.0, bucket_sec=10) == [[5.0, 8.1, 2.0], [15.0, 7.0, 0.5]]

    def test_latest_has_age(self, dashboard):
        h = dashboard.History()
        assert h.latest(10.0) is None
        h.add(4.0, 8.1, 1.5)
        assert h.latest(10.0) == ((8.1, 1.5), 6.0)

    def test_battery_view(self, dashboard, monkeypatch):
        h = dashboard.History()
        h.add(100.0, 7.9, 2.0)
        monkeypatch.setattr(dashboard, '_battery', h)
        assert dashboard.battery_view(102.0) == {
            'volts': 7.9, 'amps': 2.0, 'age': 2.0, 'stale': False
        }
        assert dashboard.battery_view(110.0)['stale']

    def test_system_sample_is_temperature_and_memory_in_use(self, dashboard, monkeypatch):
        h = dashboard.History()
        monkeypatch.setattr(dashboard, '_system', h)
        monkeypatch.setattr(dashboard, '_read_soc_temp', lambda: 61.26)
        monkeypatch.setattr(dashboard, '_read_memory', lambda: {'available': 2000, 'total': 8000})
        dashboard.sample_system(5.0)
        assert h.latest(5.0) == ((61.3, 75.0), 0.0)

    def test_window_is_two_hours(self, dashboard):
        assert dashboard.BATTERY_WINDOW_SEC == 7200

    def test_page_charts_both_series(self, dashboard):
        html = dashboard.DASHBOARD_HTML
        assert "fetch('/api/history')" in html and "fetch('/api/overview')" in html
        for chart in ('ov-volts', 'ov-amps', 'ov-temp', 'ov-mem', 'ov-imu', 'ov-speed'):
            assert f'id="{chart}"' in html
        assert 'Pack' not in html and 'pack' not in html


class FakeGraphNode:
    def __init__(self):
        self.made, self.destroyed = [], []

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


class TestOnDemand:
    def test_rate_subscriptions_detach_and_counts_reset(self, sampler):
        s, _clock = sampler
        node = FakeGraphNode()
        s._subs = {'/scan': 'sub-scan', '/motor': 'sub-motor'}
        s.destroy_subscription = node.destroy_subscription
        s.attached_at = 1.0
        s._stamps['/scan'].extend([1.0, 1.1])
        s.detach_subscriptions()
        assert sorted(node.destroyed) == ['sub-motor', 'sub-scan']
        assert s._subs == {} and s.attached_at is None and not s._stamps['/scan']

    def test_warming_until_the_window_fills(self, sampler):
        s, clock = sampler
        s.attached_at = clock.t
        assert s.warming()
        clock.t += s._window + 0.1
        assert not s.warming()

    def test_view_tracking(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_last_view', 0.0)
        dashboard._wake.clear()
        assert dashboard.note_view() is True and dashboard._wake.is_set()
        assert dashboard.viewing()
        assert dashboard.note_view() is False
