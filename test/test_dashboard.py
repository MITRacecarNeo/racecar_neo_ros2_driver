"""Unit tests for scripts/dashboard.py."""

from collections import deque
import json
import threading
from types import SimpleNamespace

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def dashboard():
    return load_script('dashboard')


class TestMonitoredAndRateTopics:
    EXPECTED_NODES = {
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
        assert "fetch('/api/status')" in dashboard.DASHBOARD_HTML

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
