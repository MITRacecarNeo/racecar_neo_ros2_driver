"""Unit tests for scripts/watchdog.py: NODES schema and restart logic."""

from pathlib import Path
import subprocess
from types import SimpleNamespace

from conftest import load_script
import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'


@pytest.fixture(scope='module')
def watchdog():
    return load_script('watchdog')


class TestNodesDict:
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
        missing = self.REQUIRED_KEYS - set(watchdog.NODES[name])
        assert not missing, f'{name} missing keys: {missing}'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_topic_starts_with_slash(self, watchdog, name):
        topic = watchdog.NODES[name]['topic']
        assert topic.startswith('/'), f'{name} topic {topic!r} must start with /'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_launch_file_exists(self, watchdog, name):
        launch_file = LAUNCH_DIR / watchdog.NODES[name]['launch']
        assert launch_file.is_file(), f'{name}: launch file {launch_file} missing'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    @pytest.mark.parametrize('key', ['device_check', 'process_check'])
    def test_checks_return_bool(self, watchdog, name, key):
        assert isinstance(watchdog.NODES[name][key](), bool)

    def test_realsense_topic(self, watchdog):
        assert watchdog.NODES['realsense']['topic'] == '/camera/color'

    def test_lidar_has_freshness_threshold(self, watchdog):
        # A stalled sllidar keeps its process and advertisement; only freshness
        # catches it. The window must span several scans and stay under the
        # cooldown, or a stall is absorbed instead of restarted.
        fresh = watchdog.NODES['lidar'].get('freshness_sec')
        assert fresh is not None, 'lidar must define freshness_sec'
        assert 1.0 <= fresh <= watchdog.RESTART_COOLDOWN


class TestConfig:
    def test_poll_interval_reasonable(self, watchdog):
        assert 1 <= watchdog.POLL_INTERVAL <= 30

    def test_restart_cooldown_at_least_15s(self, watchdog):
        # A node that crashes on start must not respawn in a tight loop.
        assert watchdog.RESTART_COOLDOWN >= 15

    def test_package_name(self, watchdog):
        assert watchdog.PACKAGE == 'racecar_neo_ros2_driver'


class TestStaleAge:
    def test_no_window_is_never_stale(self, watchdog):
        assert watchdog.stale_age(None, True, True, 100.0, 60.0) is None

    def test_old_message_is_stale(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 7.5) == 7.5

    def test_recent_message_is_fresh(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 0.2) is None

    def test_exactly_at_the_window_is_fresh(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 5.0) is None

    def test_no_message_yet_is_not_stale(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, None) is None

    def test_grace_after_a_restart(self, watchdog):
        # The restarted node gets one full window before it is judged.
        assert watchdog.stale_age(5.0, True, True, 4.9, 30.0) is None
        assert watchdog.stale_age(5.0, True, True, 5.0, 30.0) == 30.0

    @pytest.mark.parametrize('topic_alive,proc_alive', [(False, True), (True, False)])
    def test_not_judged_while_down(self, watchdog, topic_alive, proc_alive):
        assert watchdog.stale_age(5.0, topic_alive, proc_alive, 100.0, 30.0) is None


class TestFailureReason:
    @pytest.mark.parametrize(
        'topic_alive,proc_alive,stale,expected',
        [
            (True, True, None, None),
            (False, False, None, 'topic+process down'),
            (False, True, None, 'topic not advertised'),
            (True, False, None, 'process not running'),
            (True, True, 7.25, 'topic stale (7.2s)'),
            (False, True, 7.25, 'topic not advertised'),
        ],
    )
    def test_reasons(self, watchdog, topic_alive, proc_alive, stale, expected):
        assert watchdog.failure_reason(topic_alive, proc_alive, stale) == expected


class TestRestartDecision:
    def test_healthy_node_skips_the_device_check(self, watchdog):
        def device_check():
            raise AssertionError('device_check must not run for a healthy node')

        assert watchdog.restart_decision(None, device_check) == 'healthy'

    def test_failed_node_with_device_restarts(self, watchdog):
        assert watchdog.restart_decision('topic not advertised', lambda: True) == 'restart'

    def test_failed_node_without_device_is_left_alone(self, watchdog):
        assert watchdog.restart_decision('process not running', lambda: False) == 'no-device'


class TestCooldown:
    def test_inside_the_cooldown(self, watchdog):
        assert watchdog.cooldown_remaining(110.0, 100.0, 30.0) == pytest.approx(20.0)

    def test_after_the_cooldown(self, watchdog):
        assert watchdog.cooldown_remaining(130.0, 100.0, 30.0) == 0.0

    def test_never_restarted(self, watchdog):
        assert watchdog.cooldown_remaining(1_700_000_000.0, 0.0) == 0.0

    def test_restart_node_honours_the_cooldown(self, watchdog, monkeypatch):
        def popen(*_a, **_k):
            raise AssertionError('restart launched inside the cooldown')

        monkeypatch.setattr(watchdog, 'subprocess', SimpleNamespace(Popen=popen))
        monkeypatch.setitem(watchdog._last_restart, 'lidar', watchdog.time.time())
        watchdog._restart_node('lidar', watchdog.NODES['lidar'])


class TestHelpers:
    def test_clean_fastrtps_orphans_returns_a_count(self, watchdog):
        assert watchdog._clean_fastrtps_orphans() >= 0

    def test_is_running_false_for_an_absent_process(self, watchdog):
        assert watchdog._is_running('/nonexistent/path/xyz_unique_string_123')() is False

    def test_pgrep_errors_count_as_running_until_the_threshold(self, watchdog, monkeypatch):
        def run(*_a, **_k):
            raise OSError('pgrep missing')

        fake = SimpleNamespace(run=run, TimeoutExpired=subprocess.TimeoutExpired)
        monkeypatch.setattr(watchdog, 'subprocess', fake)
        check = watchdog._is_running('/any')
        results = [check() for _ in range(watchdog.PGREP_FAIL_THRESHOLD)]
        assert results[:-1] == [True] * (watchdog.PGREP_FAIL_THRESHOLD - 1)
        assert results[-1] is False


class _StubNode:
    def __init__(self) -> None:
        self.destroyed = []

    def destroy_subscription(self, sub) -> None:
        self.destroyed.append(sub)


@pytest.fixture
def monitor(watchdog):
    node = _StubNode()
    return watchdog._FreshnessMonitor(node, ['/scan']), node


class TestFreshnessMonitor:
    def test_age_none_before_any_message(self, monitor):
        fm, _ = monitor
        assert fm.age('/scan') is None

    def test_age_recent_after_mark(self, monitor):
        fm, _ = monitor
        fm._mark('/scan')
        assert 0 <= fm.age('/scan') < 0.5

    def test_reset_clears_last_seen_and_drops_the_subscription(self, monitor):
        fm, node = monitor
        fm._subs['/scan'] = 'sub'
        fm._mark('/scan')
        fm.reset('/scan')
        assert fm.age('/scan') is None
        assert node.destroyed == ['sub']
