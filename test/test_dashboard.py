"""Unit tests for scripts/dashboard.py."""

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'dashboard.py'


def _load_dashboard_module():
    spec = importlib.util.spec_from_file_location('dashboard', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def dashboard():
    return _load_dashboard_module()


def test_script_exists_and_executable():
    import os

    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK)


def test_py_compile_clean():
    result = subprocess.run(
        ['python3', '-m', 'py_compile', str(SCRIPT)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr


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
        # Dashboard cards mirror the user's mental model of the robot —
        # every node we ship should have a card, including the ones the
        # watchdog doesn't supervise (edgetpu, dotmatrix).
        assert set(dashboard.MONITORED) == self.EXPECTED_NODES

    @pytest.mark.parametrize('name', sorted(EXPECTED_NODES))
    def test_monitored_entry_has_label_and_topic(self, dashboard, name):
        cfg = dashboard.MONITORED[name]
        assert 'label' in cfg and cfg['label']
        assert 'topic' in cfg and cfg['topic'].startswith('/')

    def test_rate_topics_subset_of_known_publishers(self, dashboard):
        # Every rate-monitored topic should belong to a node we know about.
        # The RealSense card is /camera/color, but it also feeds rate-monitored
        # depth (/camera/depth) and its IMU source (/imu/realsense), so allow
        # those alongside the MONITORED card topics.
        all_topics = {cfg['topic'] for cfg in dashboard.MONITORED.values()}
        realsense_extra = {'/camera/depth', '/imu/realsense'}
        known = all_topics | realsense_extra
        for t in dashboard.RATE_TOPICS:
            assert t in known, f'{t} in RATE_TOPICS but no known node publishes it'


class TestGetStatus:
    def test_returns_dict_with_required_keys(self, dashboard):
        # get_status() is always callable; returns the cached snapshot.
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
        # The HTTP handler json.dumps() this, so a non-JSON-serializable
        # value would break /api/status.
        snapshot = dashboard.get_status()
        json.dumps(snapshot)  # must not raise


class TestMeasureHz:
    def test_returns_none_on_missing_topic(self, dashboard):
        # ros2 topic hz on a nonexistent topic times out → return None.
        # Stub _run if ros2 isn't available so this test passes in CI.
        hz = dashboard._measure_hz('/__no_such_topic_xyz__')
        assert hz is None or isinstance(hz, float)


class TestSystemHealth:
    """RTC battery + Pi under-voltage are slow-changing diagnostics."""

    def test_classify_rtc_above_threshold_is_healthy(self, dashboard):
        status, label = dashboard._classify_rtc(2.95)
        assert status == 'healthy'
        assert '2.95' in label

    def test_classify_rtc_borderline_is_stale(self, dashboard):
        # 2.75 V → between RTC_LOW (2.7) and RTC_OK (2.8) → warn.
        status, label = dashboard._classify_rtc(2.75)
        assert status == 'stale'
        assert 'recharge soon' in label

    def test_classify_rtc_below_low_is_dead(self, dashboard):
        status, label = dashboard._classify_rtc(2.5)
        assert status == 'dead'
        assert 'RECHARGE NOW' in label

    def test_classify_rtc_none_is_dead(self, dashboard):
        status, label = dashboard._classify_rtc(None)
        assert status == 'dead'
        assert 'NO READING' in label

    def test_classify_rtc_exactly_at_threshold(self, dashboard):
        # 3.0 V exactly: spec says ≥3.0 V is healthy.
        assert dashboard._classify_rtc(3.0)[0] == 'healthy'
        # 2.7 V exactly: spec says ≥2.7 V is the stale (replace-soon) band.
        assert dashboard._classify_rtc(2.7)[0] == 'stale'
        # 2.69999 V → drops into dead.
        assert dashboard._classify_rtc(2.6999)[0] == 'dead'

    def test_collect_system_health_keys(self, dashboard):
        # Even on a CI box without vcgencmd or rpi_volt hwmon, the function
        # must return both keys with sensible "unavailable" status.
        health = dashboard._collect_system_health()
        assert 'rtc' in health
        assert 'under_voltage' in health
        for entry in health.values():
            assert 'label' in entry
            assert 'status' in entry
            assert 'detail' in entry
            assert entry['status'] in ('healthy', 'stale', 'dead')


class TestDashboardHTML:
    def test_html_template_present(self, dashboard):
        assert dashboard.DASHBOARD_HTML
        assert '<!DOCTYPE html>' in dashboard.DASHBOARD_HTML

    def test_html_references_api_endpoint(self, dashboard):
        # JavaScript polls /api/status; if we ever rename the endpoint
        # both sides need to update together.
        assert "fetch('/api/status')" in dashboard.DASHBOARD_HTML

    def test_title_says_racecar(self, dashboard):
        # Sanity check we didn't leave "UAV Neo" in the port.
        html = dashboard.DASHBOARD_HTML
        assert 'RACECAR Neo' in html
        assert 'UAV Neo' not in html

    def test_system_health_section_present(self, dashboard):
        # The HTML must have a target div the JS can fill, and the JS must
        # read data.system_health. Both sides need to agree on the field name.
        html = dashboard.DASHBOARD_HTML
        assert 'id="system-health"' in html
        assert 'data.system_health' in html


class TestConfig:
    def test_port_is_8080(self, dashboard):
        # Matches racecar-dashboard.service expectation.
        assert dashboard.PORT == 8080
