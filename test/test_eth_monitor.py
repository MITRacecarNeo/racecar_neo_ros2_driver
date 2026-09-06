"""Unit tests for scripts/eth_monitor.py (the eth0 address and link logger)."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'eth_monitor.py'
UNIT = Path(__file__).parent.parent / 'scripts' / 'racecar-eth-monitor.service'


def _load():
    spec = importlib.util.spec_from_file_location('eth_monitor', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules['eth_monitor'] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def mon():
    return _load()


def test_script_exists_and_executable():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK)


def test_py_compile_clean():
    result = subprocess.run(
        ['python3', '-m', 'py_compile', str(SCRIPT)],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stderr


class TestStateRendering:
    def test_renders_every_field(self, mon):
        st = mon.State(v4=['192.168.52.200/24'], v6_default=False,
                       carrier='1', operstate='up', nm_state='connected',
                       v4_default='')
        line = st.render()
        for fragment in ('v4=192.168.52.200/24', 'v4_default=none',
                         'v6_default=no', 'carrier=1', 'operstate=up',
                         'nm=connected'):
            assert fragment in line

    def test_no_address_renders_as_none(self, mon):
        assert 'v4=NONE' in mon.State().render()

    def test_key_ignores_nothing_that_matters(self, mon):
        a = mon.State(v4=['10.0.0.1/24'], carrier='1')
        b = mon.State(v4=['10.0.0.1/24'], carrier='0')
        assert a.key() != b.key(), 'a carrier change must count as a change'


class TestClassify:
    def _st(self, mon, **kw):
        base = {'v4': ['192.168.52.200/24'], 'v6_default': False,
                'carrier': '1', 'operstate': 'up', 'nm_state': 'connected'}
        base.update(kw)
        return mon.State(**base)

    def test_first_sample_is_start(self, mon):
        assert mon.classify(None, self._st(mon)) == 'START'

    def test_lost_address_is_named(self, mon):
        prev = self._st(mon, v4=['192.168.52.200/24', '10.0.0.5/24'])
        cur = self._st(mon, v4=['10.0.0.5/24'])
        tag = mon.classify(prev, cur)
        assert 'ADDR_LOST:192.168.52.200/24' in tag

    def test_gained_address_is_named(self, mon):
        prev = self._st(mon, v4=[])
        cur = self._st(mon, v4=['192.168.52.200/24'])
        assert 'ADDR_GAINED:192.168.52.200/24' in mon.classify(prev, cur)

    def test_carrier_transition_is_named(self, mon):
        # The reported failure recovers only when the cable is reseated, so a
        # carrier drop is the transition most worth spotting in the log.
        prev = self._st(mon, carrier='1')
        cur = self._st(mon, carrier='0')
        assert 'CARRIER:1->0' in mon.classify(prev, cur)

    def test_operstate_transition_is_named(self, mon):
        prev = self._st(mon, operstate='up')
        cur = self._st(mon, operstate='down')
        assert 'OPERSTATE:up->down' in mon.classify(prev, cur)

    def test_nm_transition_is_named(self, mon):
        prev = self._st(mon, nm_state='connected')
        cur = self._st(mon, nm_state='disconnected')
        assert 'NM:connected->disconnected' in mon.classify(prev, cur)

    def test_v6_default_transition_is_named(self, mon):
        prev = self._st(mon, v6_default=False)
        cur = self._st(mon, v6_default=True)
        assert 'V6_DEFAULT:gained' in mon.classify(prev, cur)

    def test_simultaneous_changes_all_reported(self, mon):
        prev = self._st(mon, v4=['192.168.52.200/24'], carrier='1')
        cur = self._st(mon, v4=[], carrier='0')
        tag = mon.classify(prev, cur)
        assert 'ADDR_LOST' in tag and 'CARRIER' in tag


class TestLogging:
    def test_once_writes_a_line(self, tmp_path):
        log = tmp_path / 'eth.log'
        result = subprocess.run(
            ['python3', str(SCRIPT), '--once', '--log', str(log)],
            capture_output=True, text=True, timeout=20,
        )
        assert result.returncode == 0, result.stderr
        text = log.read_text()
        assert '[ONCE]' in text
        assert 'carrier=' in text

    def test_log_is_appended_not_truncated(self, tmp_path, mon):
        log = tmp_path / 'eth.log'
        mon.write(log, 'START', mon.State(v4=['1.2.3.4/24']))
        mon.write(log, 'OK', mon.State(v4=['1.2.3.4/24']))
        assert len(log.read_text().strip().splitlines()) == 2

    def test_creates_the_log_directory(self, tmp_path, mon):
        log = tmp_path / 'nested' / 'dir' / 'eth.log'
        mon.write(log, 'START', mon.State())
        assert log.is_file()

    def test_line_carries_a_timestamp_and_tag(self, tmp_path, mon):
        log = tmp_path / 'eth.log'
        mon.write(log, 'ADDR_LOST:1.2.3.4/24', mon.State())
        line = log.read_text().strip()
        assert line.startswith('20')
        assert '[ADDR_LOST:1.2.3.4/24]' in line


class TestServiceUnit:
    def test_unit_ships(self):
        assert UNIT.is_file()

    def test_unit_needs_no_ros_environment(self):
        # It reads sysfs, ip and nmcli only; sourcing ROS would make it fail
        # to start before the workspace is built.
        text = UNIT.read_text()
        assert 'setup.bash' not in text

    def test_unit_restarts_and_survives_reboot(self):
        text = UNIT.read_text()
        assert 'Restart=always' in text
        assert 'WantedBy=multi-user.target' in text

    def test_unit_is_not_auto_installed(self):
        # It is a temporary diagnostic, not part of the running car, so
        # setup_services.sh must not enable it alongside the four real units.
        installer = (Path(__file__).parent.parent / 'scripts' / 'setup_services.sh').read_text()
        assert 'racecar-eth-monitor' not in installer
