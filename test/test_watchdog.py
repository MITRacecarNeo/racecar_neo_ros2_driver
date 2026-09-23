"""Unit tests for scripts/watchdog.py (NODES schema + pure helpers)."""

import importlib.util
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'watchdog.py'


def _load_watchdog_module():
    spec = importlib.util.spec_from_file_location('watchdog', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def watchdog():
    return _load_watchdog_module()


def test_script_exists_and_executable():
    import os

    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK)


def test_bash_syntax_clean_python():
    result = subprocess.run(
        ['python3', '-m', 'py_compile', str(SCRIPT)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr


class TestNodesDict:
    """The NODES dict is the watchdog's contract — every entry must be well-formed."""

    EXPECTED_NAMES = {
        'pit',
        'throttle',
        'mux',
        'gamepad',
        'imu_fusion',
        'lidar',
        'realsense',
    }
    REQUIRED_KEYS = {
        'topic',
        'launch',
        'device_check',
        'device_label',
        'kill_pattern',
        'process_check',
    }

    def test_all_expected_nodes_present(self, watchdog):
        assert set(watchdog.NODES) == self.EXPECTED_NAMES

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_node_has_required_keys(self, watchdog, name):
        cfg = watchdog.NODES[name]
        missing = self.REQUIRED_KEYS - set(cfg)
        assert not missing, f'{name} missing keys: {missing}'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_topic_starts_with_slash(self, watchdog, name):
        topic = watchdog.NODES[name]['topic']
        assert topic.startswith('/'), f'{name} topic {topic!r} must start with /'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_launch_file_exists(self, watchdog, name):
        launch_dir = SCRIPT.parent.parent / 'launch'
        launch_file = launch_dir / watchdog.NODES[name]['launch']
        assert launch_file.is_file(), f'{name}: launch file {launch_file} missing'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_device_check_callable(self, watchdog, name):
        cb = watchdog.NODES[name]['device_check']
        assert callable(cb)
        # device_check should be safe to call repeatedly and return bool.
        result = cb()
        assert isinstance(result, bool)

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_process_check_callable(self, watchdog, name):
        cb = watchdog.NODES[name]['process_check']
        assert callable(cb)
        result = cb()
        assert isinstance(result, bool)

    def test_realsense_topic(self, watchdog):
        # RealSense color is remapped onto /camera/color (the only camera).
        assert watchdog.NODES['realsense']['topic'] == '/camera/color'

    def test_lidar_has_freshness_threshold(self, watchdog):
        # sllidar can silently desync from the CP2102 (the SDK swallows
        # transient read errors); process-presence + topic-advertisement both
        # stay green. Freshness check on /scan is what catches it.
        # Threshold must be > 0.1s (one scan period) and < restart cooldown
        # (so a stall actually triggers a restart instead of being absorbed).
        fresh = watchdog.NODES['lidar'].get('freshness_sec')
        assert fresh is not None, 'lidar must define freshness_sec'
        assert 1.0 <= fresh <= watchdog.RESTART_COOLDOWN


class TestConfig:
    def test_poll_interval_reasonable(self, watchdog):
        # Too short = thrashing; too long = slow failure detection.
        assert 1 <= watchdog.POLL_INTERVAL <= 30

    def test_restart_cooldown_at_least_15s(self, watchdog):
        # Prevents respawn loops if a node crashes immediately on start.
        assert watchdog.RESTART_COOLDOWN >= 15

    def test_package_name(self, watchdog):
        assert watchdog.PACKAGE == 'racecar_neo_ros2_driver'


class TestHelpers:
    def test_clean_fastrtps_orphans_safe_on_empty_shm(self, watchdog):
        # /dev/shm in a test environment may have no fastrtps segments —
        # the function must handle that gracefully and return 0.
        n = watchdog._clean_fastrtps_orphans()
        assert isinstance(n, int)
        assert n >= 0

    def test_find_rpi_volt_alarm_returns_path_or_none(self, watchdog):
        result = watchdog._find_rpi_volt_alarm()
        # Either a Path (on Pi 5 with rpi_volt driver) or None (CI / dev box).
        assert result is None or hasattr(result, 'read_text')

    def test_is_running_returns_callable(self, watchdog):
        cb = watchdog._is_running('/nonexistent/path/xyz_unique_string_123')
        assert callable(cb)
        # No such process should exist with that path substring.
        assert cb() is False


class TestFreshnessMonitor:
    """The bookkeeping side of _FreshnessMonitor — independent of rclpy."""

    def test_age_none_before_any_message(self, watchdog):
        fm = watchdog._FreshnessMonitor.__new__(watchdog._FreshnessMonitor)
        fm._node = None
        fm._topics = ['/scan']
        import threading

        fm._lock = threading.Lock()
        fm._last = {}
        fm._subs = {}
        assert fm.age('/scan') is None

    def test_age_recent_after_mark(self, watchdog):
        fm = watchdog._FreshnessMonitor.__new__(watchdog._FreshnessMonitor)
        fm._node = None
        fm._topics = ['/scan']
        import threading

        fm._lock = threading.Lock()
        fm._last = {}
        fm._subs = {}
        fm._mark('/scan')
        age = fm.age('/scan')
        assert age is not None
        assert 0 <= age < 0.5  # called immediately, should be tiny

    def test_reset_clears_last_seen(self, watchdog):
        fm = watchdog._FreshnessMonitor.__new__(watchdog._FreshnessMonitor)
        fm._node = None
        fm._topics = ['/scan']
        import threading

        fm._lock = threading.Lock()
        fm._last = {}
        fm._subs = {}
        fm._mark('/scan')
        assert fm.age('/scan') is not None
        fm.reset('/scan')
        assert fm.age('/scan') is None
