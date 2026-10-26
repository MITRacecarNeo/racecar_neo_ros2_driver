"""Unit tests for scripts/dashboard.py."""

from collections import deque
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
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
        assert set(health) == {'data', 'rtc', 'under_voltage'}
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
        assert [a['port'] for a in dashboard.APPS] == [8086, 8888, 8084, 8081, 8082, 8083, 8085]
        core = [a['id'] for a in dashboard.APPS if a['group'] == 'core']
        assert core == ['terminal', 'jupyter', 'mapview']
        exclusive = [a['id'] for a in dashboard.APPS if a['exclusive']]
        assert exclusive == ['webteleop', 'linefollow', 'wallfollow']
        startable = [a['id'] for a in dashboard.APPS if a['startable']]
        assert startable == ['terminal', 'webteleop', 'linefollow', 'wallfollow', 'camlabel']

    def test_every_app_icon_exists(self, dashboard):
        scripts = Path(__file__).resolve().parent.parent / 'scripts'
        icons = (scripts / 'web' / 'icons.svg').read_text()
        for app in dashboard.APPS:
            assert f'id="{app["icon"]}"' in icons, app['icon']

    def test_terminal_starts_when_opened(self, dashboard):
        run = FakeRun({})
        assert dashboard.open_app('terminal', run) == (True, 'Terminal started.')
        assert run.states['racecar-terminal'] == 'active'
        assert dashboard.open_app('terminal', run) == (True, 'Terminal is running.')

    def test_clear_apps_leaves_the_terminal_running(self, dashboard):
        run = FakeRun({'racecar-terminal': 'active', 'racecar-camlabel': 'active'})
        dashboard.clear_apps(run)
        assert run.states['racecar-terminal'] == 'active'
        assert run.states['racecar-camlabel'] == 'inactive'

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
        assert ok and message == 'Camera Label started.'
        assert run.states['racecar-webteleop'] == 'active'
        assert run.states['racecar-camlabel'] == 'active'

    def test_open_stopped_core_app_names_the_command(self, dashboard):
        run = FakeRun({'racecar-mapview': 'inactive'})
        ok, message = dashboard.open_app('mapview', run)
        assert not ok and 'racecar service start mapview' in message
        assert not [c for c in run.calls if c[1] == '--no-ask-password']

    def test_clear_apps_stops_utilities_only(self, dashboard):
        run = FakeRun(
            {
                'racecar-mapview': 'active',
                'racecar-jupyter': 'active',
                'racecar-webteleop': 'active',
                'racecar-camlabel': 'active',
                'racecar-dashboard': 'active',
                'racecar-teleop': 'active',
            }
        )
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

    def test_clear_log_hides_earlier_lines_and_keeps_the_file(
        self, dashboard, tmp_path, monkeypatch
    ):
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
            'racecar-mapview': 'active',
            'racecar-jupyter': 'failed',
            'racecar-x': 'inactive',
        }
        apps = dashboard.app_list(states)
        by_id = {a['id']: a['state'] for a in apps}
        assert by_id['mapview'] == 'active' and by_id['webteleop'] == 'unknown'
        assert by_id['terminal'] == 'unknown'

    def test_activate_stops_the_running_lab_first(self, dashboard):
        run = FakeRun({'racecar-webteleop': 'active', 'racecar-mapview': 'active'})
        ok, message = dashboard.activate('wallfollow', run)
        assert ok and message == 'Wall Follow active.'
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
        for unit in (
            'racecar-webteleop',
            'racecar-linefollow',
            'racecar-wallfollow',
            'racecar-camlabel',
            'racecar-terminal',
        ):
            assert f'"{unit}.service"' in text
        assert '["start", "stop"]' in text and 'subject.user === "racecar"' in text
        for core in (
            'racecar-dashboard',
            'racecar-teleop',
            'racecar-autonomy',
            'racecar-mapview',
            'racecar-jupyter',
        ):
            assert core not in text
        assert 'enable' not in text.split('polkit.addRule')[1]
        assert '49-racecar-dashboards.rules' in (scripts / 'setup_user_env.sh').read_text()

    def test_teleop_rule_allows_restart_only(self):
        scripts = Path(__file__).resolve().parent.parent / 'scripts'
        rule = (scripts / 'polkit' / '49-racecar-teleop.rules').read_text()
        body = rule.split('polkit.addRule')[1]
        assert '"racecar-teleop.service"' in body and 'action.lookup("verb") === "restart"' in body
        assert 'subject.user === "racecar"' in body
        for other in ('"start"', '"stop"', 'enable', 'racecar-autonomy'):
            assert other not in body
        assert '49-racecar-teleop.rules' in (scripts / 'setup_user_env.sh').read_text()

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
            'volts': 7.9,
            'amps': 2.0,
            'age': 2.0,
            'stale': False,
            'estimate': None,
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


class TestOverrideSession:
    def test_off_publishes_zero(self, dashboard):
        assert dashboard.OverrideSession().session(10.0) == 0

    def test_start_publishes_a_nonzero_number(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        assert o.session(10.5) != 0

    def test_each_start_is_a_new_number(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        first = o.number
        o.start(11.0)
        assert o.number != first

    def test_lapses_without_keepalive(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        o.mux_active(True, 10.2)
        o.keep(11.5)
        assert o.session(13.4) != 0
        assert o.session(13.6) == 0
        assert 'stopped responding' in o.ended

    def test_keepalive_does_not_restart(self, dashboard):
        o = dashboard.OverrideSession()
        o.keep(10.0)
        assert o.session(10.0) == 0

    def test_mux_cancel_ends_it_for_good(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        o.mux_active(False, 10.1)
        assert o.session(10.1) != 0
        o.mux_active(True, 10.3)
        o.mux_active(False, 10.8)
        o.keep(10.9)
        assert o.session(10.9) == 0
        assert o.ended == 'Ended by the controller.'

    def test_unconfirmed_ends_when_teleop_is_down(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        o.keep(12.0)
        o.keep(13.5)
        assert o.session(13.5) == 0
        assert o.ended == 'Teleop is not running.'

    def test_view(self, dashboard):
        o = dashboard.OverrideSession()
        o.start(10.0)
        assert o.view(10.0) == {'on': True, 'confirmed': False, 'ended': ''}
        o.mux_active(True, 10.1)
        assert o.view(10.1)['confirmed']
        o.stop()
        assert o.view(10.2) == {'on': False, 'confirmed': False, 'ended': ''}


class TestOverrideEndpoint:
    @pytest.fixture
    def post(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_override', dashboard.OverrideSession())
        server = ThreadingHTTPServer(('127.0.0.1', 0), dashboard.DashboardHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()

        def send(action, headers=None):
            conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=5)
            conn.request(
                'POST', '/api/override', body=json.dumps({'action': action}), headers=headers or {}
            )
            resp = conn.getresponse()
            send.cors = resp.getheader('Access-Control-Allow-Origin')
            return resp.status, json.loads(resp.read())

        yield send
        server.shutdown()
        server.server_close()

    def test_start_keep_stop(self, post):
        assert post('start')[1]['on'] is True
        assert post('keep')[1]['on'] is True
        assert post('stop')[1]['on'] is False
        assert post('keep')[1]['on'] is False

    def test_lab_page_can_read_the_reply(self, post):
        post('keep', {'Origin': 'http://127.0.0.1:8081'})
        assert post.cors == 'http://127.0.0.1:8081'

    def test_other_origins_get_no_cors_header(self, post):
        post('keep', {'Origin': 'http://127.0.0.1:8085'})
        assert post.cors is None

    def test_unknown_action_is_rejected(self, post):
        assert post('go')[0] == 400

    def test_page_has_the_toggle_and_warning(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'id="override"' in html and 'id="override-banner"' in html
        assert 'The car may start moving without warning. Do not leave it unattended.' in html


class TestOverrideOrigin:
    @pytest.mark.parametrize(
        'origin, host, allowed',
        [
            ('http://car.local:8081', 'car.local:8080', True),
            ('http://car.local:8083', 'car.local:8080', True),
            ('http://car.local:8085', 'car.local:8080', False),
            ('http://other:8081', 'car.local:8080', False),
            ('https://car.local:8081', 'car.local:8080', False),
            (None, 'car.local:8080', False),
        ],
    )
    def test_only_driving_lab_pages_on_this_host(self, dashboard, origin, host, allowed):
        assert (dashboard.override_origin(origin, host) is not None) is allowed


TELEOP, AUTONOMY = 'racecar-teleop.service', 'racecar-autonomy.service'


class TestFeatures:
    def test_tiles(self, dashboard):
        assert [f['id'] for f in dashboard.FEATURES] == ['camera', 'detections', 'autonomy']
        needs = [a['id'] for a in dashboard.APPS if a.get('needs') == 'camera']
        assert needs == ['linefollow', 'camlabel']

    def test_states(self, dashboard):
        wanted = {'camera': True, 'detections': False}
        items = dashboard.feature_list(
            {TELEOP: 'active', AUTONOMY: 'inactive'}, wanted, wanted, False, {}
        )
        assert [i['state'] for i in items] == ['active', 'inactive', 'inactive']

    def test_pending_restart_shows_starting(self, dashboard):
        items = dashboard.feature_list(
            {TELEOP: 'active'},
            {'camera': True, 'detections': True},
            {'camera': True, 'detections': False},
            False,
            {},
        )
        assert items[1]['state'] == 'activating'
        items = dashboard.feature_list(
            {TELEOP: 'active'}, dashboard.features.DEFAULTS, dashboard.features.DEFAULTS, True, {}
        )
        assert items[0]['state'] == items[1]['state'] == 'activating'

    def test_teleop_down_reads_off(self, dashboard):
        items = dashboard.feature_list(
            {TELEOP: 'inactive'},
            dashboard.features.DEFAULTS,
            dashboard.features.DEFAULTS,
            False,
            {},
        )
        assert items[0]['state'] == 'inactive'

    def test_last_run(self, dashboard, tmp_path):
        path = tmp_path / 'autonomy.conf'
        assert dashboard.last_run(path) == {}
        path.write_text('AUTONOMY_MODE=localization\nAUTONOMY_MAP=hall\n')
        assert dashboard.last_run(path) == {'mode': 'localization', 'map': 'hall'}
        path.write_text('AUTONOMY_MODE=off\nAUTONOMY_MAP=hall\n')
        assert dashboard.last_run(path) == {}


class _ToolRun(FakeRun):
    """FakeRun that also answers `racecar map` calls."""

    def __init__(self, states, codes=None):
        super().__init__(states)
        self.codes = codes or {}
        self.tool = []

    def __call__(self, argv, **kw):
        if argv[:2] == list(self.__dashboard.MAP_TOOL):
            action = argv[2]
            self.tool.append(action)
            if action == 'stop':
                self.states[AUTONOMY] = 'inactive'
            if action == 'resume':
                self.states[AUTONOMY] = 'active'
            code = self.codes.get(action, 0)
            return SimpleNamespace(stdout=f'{action} done\n', stderr='', returncode=code)
        return super().__call__(argv, **kw)


class TestApplyFeatures:
    @pytest.fixture
    def tool(self, dashboard, monkeypatch, tmp_path):
        _ToolRun._ToolRun__dashboard = dashboard
        monkeypatch.setattr(dashboard.features, 'state_file', lambda: tmp_path / 'f.conf')
        monkeypatch.setattr(dashboard.features, 'boot_id', lambda: 'b')
        return _ToolRun

    def test_restarts_teleop_and_writes_the_choice(self, dashboard, tool):
        run = tool({TELEOP: 'active'})
        message = dashboard.apply_features({'camera': False, 'detections': False}, run)
        assert ['systemctl', '--no-ask-password', 'restart', TELEOP] in run.calls
        assert run.tool == [] and message == 'Camera off. Detections off.'
        assert dashboard.features.read() == {'camera': False, 'detections': False}

    def test_autonomy_is_stopped_first_and_resumed(self, dashboard, tool):
        run = tool({TELEOP: 'active', AUTONOMY: 'active'})
        message = dashboard.apply_features(
            {'camera': True, 'detections': True}, run, sleep=lambda s: None
        )
        restart = run.calls.index(['systemctl', '--no-ask-password', 'restart', TELEOP])
        assert run.tool == ['stop', 'resume'] and restart > 0
        assert message.endswith('resume done')

    def test_a_failed_stop_leaves_teleop_alone(self, dashboard, tool):
        run = tool({TELEOP: 'active', AUTONOMY: 'active'}, codes={'stop': 1})
        message = dashboard.apply_features(dashboard.features.DEFAULTS, run)
        assert message.startswith('Could not stop autonomy')
        assert not [c for c in run.calls if 'restart' in c]

    def test_autonomy_actions(self, dashboard, tool):
        run = tool({AUTONOMY: 'active'})
        assert dashboard.autonomy_action('save-stop', run) == 'stop done'
        assert run.tool == ['save', 'stop']
        run = tool({AUTONOMY: 'active'}, codes={'save': 1})
        assert dashboard.autonomy_action('save-stop', run).startswith('Not stopped.')
        assert run.tool == ['save']
        assert dashboard.autonomy_action('resume', tool({})) == 'resume done'
        with pytest.raises(ValueError):
            dashboard.autonomy_action('rm', run)


class TestFeatureEndpoint:
    def test_rejects_unknown_and_busy(self, dashboard, monkeypatch):
        job = dashboard.FeatureJob()
        monkeypatch.setattr(dashboard, '_features_job', job)
        assert dashboard.change_feature('lidar', False) == (False, "unknown feature 'lidar'")
        assert dashboard.change_feature('autonomy', True, 'go')[0] is False
        assert job.start('busy', lambda: 'x' if __import__('time').sleep(0.3) is None else '')
        assert dashboard.change_feature('autonomy', True, 'resume') == (
            False,
            'Another change is running.',
        )

    def test_page_has_features(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'id="features"' in html and 'id="autosheet"' in html
        assert 'needs the camera. Turn it on?' in html


class TestDrivingAppsAndAutonomy:
    @pytest.fixture
    def tool(self, dashboard):
        _ToolRun._ToolRun__dashboard = dashboard
        return _ToolRun

    def test_opening_a_driving_app_saves_and_stops_mapping(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'mapping', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'})
        ok, _ = dashboard.open_app('webteleop', run)
        assert ok and run.tool == ['save', 'stop']
        assert run.states['racecar-webteleop'] == 'active'

    def test_opening_a_driving_app_stops_localization(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'localization', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'})
        assert dashboard.open_app('linefollow', run)[0] and run.tool == ['stop']

    def test_a_failed_save_keeps_mapping_and_the_app_closed(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'mapping', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'}, codes={'save': 1})
        ok, message = dashboard.open_app('wallfollow', run)
        assert not ok and message.startswith('Autonomy is still running.')
        assert run.states.get('racecar-wallfollow', 'inactive') == 'inactive'

    def test_camera_label_leaves_autonomy_running(self, dashboard, tool):
        run = tool({AUTONOMY: 'active'})
        assert dashboard.open_app('camlabel', run)[0] and run.tool == []

    def test_page_asks_before_stopping_autonomy(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'Save it, stop it, and open' in html and 'will close.' in html


GUARD_VALUES = {
    'level': 'countdown',
    'trigger': 'estimate',
    'valid': 'true',
    'volts': '7.12',
    'soc': '3',
    'power_w': '17.4',
    'minutes_left': '2',
    'shutdown_in': '42',
    'cancel_used': 'false',
}


class TestBatteryEstimate:
    def test_view(self, dashboard):
        view = dashboard.estimate_view(GUARD_VALUES, 0.5)
        assert view == {
            'level': 'countdown',
            'trigger': 'estimate',
            'minutes_left': 2.0,
            'power_w': 17.4,
            'soc': 3.0,
            'shutdown_in': 42.0,
            'can_cancel': True,
        }

    def test_stale_or_missing(self, dashboard):
        assert dashboard.estimate_view(GUARD_VALUES, 6.0) is None
        assert dashboard.estimate_view(None, None) is None

    def test_empty_values_are_none(self, dashboard):
        view = dashboard.estimate_view({'level': 'ok', 'minutes_left': ''}, 0.5)
        assert view['minutes_left'] is None and view['level'] == 'ok'

    @pytest.mark.parametrize(
        'changes',
        [{'trigger': 'floor'}, {'cancel_used': 'true'}, {'level': 'warn'}],
    )
    def test_cancel_only_for_an_estimate_countdown_once(self, dashboard, changes):
        assert not dashboard.estimate_view({**GUARD_VALUES, **changes}, 0.5)['can_cancel']


class TestOrigins:
    @pytest.mark.parametrize(
        'origin, host, allowed',
        [
            (None, 'car.local:8080', True),
            ('http://car.local:8080', 'car.local:8080', True),
            ('http://car.local:8081', 'car.local:8080', False),
            ('http://evil.example:8080', 'car.local:8080', False),
        ],
    )
    def test_page_origin(self, dashboard, origin, host, allowed):
        assert dashboard.page_origin(origin, host) is allowed

    @pytest.mark.parametrize(
        'origin, allowed',
        [
            ('http://car.local:8084', True),
            ('http://car.local:8085', True),
            ('http://car.local:8089', False),
            ('http://other:8084', False),
        ],
    )
    def test_lab_origin(self, dashboard, origin, allowed):
        assert (dashboard.lab_origin(origin, 'car.local:8080') is not None) is allowed


class TestTurnOff:
    def test_terminals_first_then_the_sequence(self, dashboard, monkeypatch):
        order = []

        class FakeAnnouncer:
            def __init__(self, run):
                pass

            def deliver(self, msg):
                order.append(('message', msg.text))

        monkeypatch.setattr(dashboard, 'Announcer', FakeAnnouncer)
        monkeypatch.setattr(
            dashboard, 'shutdown_sequence', lambda run, report: order.append(('sequence',)) or []
        )
        dashboard.turn_off()
        assert order == [
            ('message', 'Turning the car off now. Turn off the power switch.'),
            ('sequence',),
        ]


class TestBatteryEndpoints:
    @pytest.fixture
    def server(self, dashboard, monkeypatch):
        calls = []
        monkeypatch.setattr(dashboard, 'turn_off', lambda: calls.append('turn_off'))
        monkeypatch.setattr(
            dashboard,
            '_sampler',
            SimpleNamespace(
                cancel_shutdown=lambda: calls.append('cancel'),
                estimate=lambda: (GUARD_VALUES, 0.5),
            ),
        )
        srv = ThreadingHTTPServer(('127.0.0.1', 0), dashboard.DashboardHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()

        def request(method, path, headers=None):
            conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
            conn.request(
                method, path, body=b'{}' if method == 'POST' else None, headers=headers or {}
            )
            resp = conn.getresponse()
            request.cors = resp.getheader('Access-Control-Allow-Origin')
            return resp.status, json.loads(resp.read())

        request.calls = calls
        yield request
        srv.shutdown()
        srv.server_close()

    def test_shutdown(self, server):
        status, body = server('POST', '/api/shutdown')
        assert status == 200
        assert body['message'] == 'Turning the car off now. Turn off the power switch.'
        time.sleep(0.2)
        assert server.calls == ['turn_off']

    def test_shutdown_refuses_another_site(self, server):
        status, _ = server('POST', '/api/shutdown', {'Origin': 'http://evil.example'})
        assert status == 403
        time.sleep(0.2)
        assert server.calls == []

    def test_cancel(self, server):
        assert server('POST', '/api/battery/cancel')[0] == 200
        assert server.calls == ['cancel']

    def test_battery_for_a_lab_page(self, server, dashboard, monkeypatch):
        h = dashboard.History()
        h.add(time.time(), 7.12, 2.4)
        monkeypatch.setattr(dashboard, '_battery', h)
        status, body = server('GET', '/api/battery', {'Origin': 'http://127.0.0.1:8084'})
        assert status == 200 and server.cors == 'http://127.0.0.1:8084'
        assert body['battery']['estimate']['shutdown_in'] == 42.0


class TestBatteryPage:
    def test_turn_off_tile_next_to_the_override(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert html.index('id="override"') < html.index('id="poweroff"')
        assert html.index('id="poweroff"') < html.index('id="apps-utilities"')
        assert "confirm('Turn the car off?')" in html
        assert '/api/shutdown' in html

    def test_header_pills_banner_and_cancel(self, dashboard):
        html = dashboard._load_dashboard_html()
        for element in ('batt-time', 'batt-power', 'battery-banner', 'battery-cancel'):
            assert f'id="{element}"' in html
        assert '/api/battery/cancel' in html

    def test_time_pill_switches_views(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert "const PILL_VIEWS = ['percent', 'time', 'clock'];" in html
        assert 'until ${at.toLocaleTimeString' in html
        assert "store.set('rcn-battery-view', pillView)" in html
        assert 'Turning the car off now. Turn off the power switch.' in html


NMCLI_DEVICES = """wlan1:wifi:connected:racecar-neo-ap
wlan0:wifi:connected:Duck
lo:loopback:connected (externally):lo
p2p-dev-wlan0:wifi-p2p:disconnected:
eth0:ethernet:unavailable:
"""
IP_ADDR = """1: lo    inet 127.0.0.1/8 scope host lo
3: wlan0    inet 10.0.0.198/24 brd 10.0.0.255 scope global dynamic wlan0
4: wlan1    inet 10.42.0.1/24 brd 10.42.0.255 scope global wlan1
"""


def _fake_network(argv, **kwargs):
    if argv[:2] == ['ip', '-4']:
        out = IP_ADDR
    elif argv[-2:] == ['show', 'racecar-neo-ap']:
        out = 'ap\nracecar-neo-0\n'
    elif argv[-2:] == ['show', 'Duck']:
        out = 'infrastructure\nDuck\n'
    else:
        out = NMCLI_DEVICES
    return SimpleNamespace(returncode=0, stdout=out)


class TestNetwork:
    def test_rows(self, dashboard):
        assert dashboard.network_view(_fake_network) == [
            {
                'device': 'wlan1',
                'kind': 'Access point',
                'state': 'connected',
                'network': 'racecar-neo-0',
                'address': '10.42.0.1/24',
            },
            {
                'device': 'wlan0',
                'kind': 'Wi-Fi',
                'state': 'connected',
                'network': 'Duck',
                'address': '10.0.0.198/24',
            },
            {
                'device': 'eth0',
                'kind': 'Ethernet',
                'state': 'unavailable',
                'network': '',
                'address': '',
            },
        ]

    def test_no_networkmanager(self, dashboard):
        def run(argv, **kwargs):
            raise OSError('no nmcli')

        assert dashboard.network_view(run) == []

    def test_clock(self, dashboard):
        clock = dashboard.clock_view(1000.0)
        assert clock['epoch'] == 1000.0
        assert clock['utc_offset'] == time.localtime(1000.0).tm_gmtoff
        assert clock['zone'] == time.localtime(1000.0).tm_zone

    def test_page_has_the_clock_and_network_pill(self, dashboard):
        html = dashboard._load_dashboard_html()
        for element in ('clock', 'net-pill', 'net-menu', 'net-table'):
            assert f'id="{element}"' in html
        assert "['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat']" in html
        assert "'PM'} ${carClock.zone}`" in html
