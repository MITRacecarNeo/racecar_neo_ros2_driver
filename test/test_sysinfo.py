"""Unit tests for scripts/sysinfo.py."""

import subprocess
from types import SimpleNamespace

from conftest import load_script
import pytest

sysinfo = load_script('sysinfo')


def _completed(stdout, returncode=0):
    return SimpleNamespace(stdout=stdout, returncode=returncode)


@pytest.fixture
def fake_run(monkeypatch):
    """Replace subprocess.run in sysinfo; set .result or .exc on the returned stub."""
    stub = SimpleNamespace(result=_completed(''), exc=None, calls=[])

    def run(cmd, **_kwargs):
        stub.calls.append(cmd)
        if stub.exc is not None:
            raise stub.exc
        return stub.result

    fake = SimpleNamespace(run=run, TimeoutExpired=subprocess.TimeoutExpired)
    monkeypatch.setattr(sysinfo, 'subprocess', fake)
    return stub


class TestDecodeThrottled:
    def test_live_and_sticky_under_voltage(self):
        flags, active = sysinfo.decode_throttled('throttled=0x50005\n')
        assert flags == 0x50005
        assert active == [
            'under-voltage',
            'currently throttled',
            'under-voltage has occurred',
            'throttling has occurred',
        ]

    def test_sticky_bits_only(self):
        # A brownout earlier this boot that has since cleared.
        flags, active = sysinfo.decode_throttled('throttled=0x50000')
        assert flags == 0x50000
        assert active == ['under-voltage has occurred', 'throttling has occurred']

    def test_clean(self):
        assert sysinfo.decode_throttled('throttled=0x0') == (0, [])

    def test_unparsable(self):
        assert sysinfo.decode_throttled('error=1') == (None, [])

    def test_read_throttled_uses_vcgencmd(self, fake_run):
        fake_run.result = _completed('throttled=0x50000\n')
        assert sysinfo.read_throttled()[0] == 0x50000
        assert fake_run.calls == [['vcgencmd', 'get_throttled']]

    def test_read_throttled_without_vcgencmd(self, fake_run):
        fake_run.exc = FileNotFoundError('vcgencmd')
        assert sysinfo.read_throttled() == (None, [])


class TestClassifyRtc:
    def test_above_ok_is_healthy(self):
        status, label = sysinfo.classify_rtc(2.95)
        assert status == 'healthy'
        assert '2.95' in label

    def test_ok_line_is_healthy(self):
        assert sysinfo.classify_rtc(sysinfo.RTC_OK_VOLTS)[0] == 'healthy'

    def test_between_low_and_ok_is_stale(self):
        status, label = sysinfo.classify_rtc(2.75)
        assert status == 'stale'
        assert 'recharge soon' in label

    def test_low_line_is_stale(self):
        assert sysinfo.classify_rtc(sysinfo.RTC_LOW_VOLTS)[0] == 'stale'

    def test_below_low_is_dead(self):
        status, label = sysinfo.classify_rtc(2.6999)
        assert status == 'dead'
        assert 'RECHARGE NOW' in label

    def test_no_reading_is_dead(self):
        assert sysinfo.classify_rtc(None) == ('dead', 'NO READING')


class TestRtcVoltage:
    def test_parses_pmic_output(self, fake_run):
        fake_run.result = _completed('     BATT_V volt(24)=2.91650000V\n')
        assert sysinfo.read_rtc_voltage() == pytest.approx(2.9165)

    def test_nonzero_exit_is_none(self, fake_run):
        fake_run.result = _completed('', returncode=255)
        assert sysinfo.read_rtc_voltage() is None

    def test_timeout_is_none(self, fake_run):
        fake_run.exc = subprocess.TimeoutExpired('vcgencmd', 3)
        assert sysinfo.read_rtc_voltage() is None


class TestUnderVoltageAlarm:
    def test_missing_hwmon_is_none(self, monkeypatch):
        monkeypatch.setattr(sysinfo, 'under_voltage_alarm_path', lambda: None)
        assert sysinfo.read_under_voltage_alarm() is None

    @pytest.mark.parametrize('text,expected', [('1\n', True), ('0\n', False)])
    def test_reads_the_sticky_flag(self, monkeypatch, tmp_path, text, expected):
        alarm = tmp_path / 'in0_lcrit_alarm'
        alarm.write_text(text)
        monkeypatch.setattr(sysinfo, 'under_voltage_alarm_path', lambda: alarm)
        assert sysinfo.read_under_voltage_alarm() is expected

    def test_path_is_a_path_or_none(self):
        result = sysinfo.under_voltage_alarm_path()
        assert result is None or result.name == 'in0_lcrit_alarm'


class TestRunCmd:
    def test_returns_stdout(self, fake_run):
        fake_run.result = _completed('out\n')
        assert sysinfo.run_cmd(['true']) == 'out\n'

    def test_nonzero_exit_is_empty(self, fake_run):
        fake_run.result = _completed('partial', returncode=1)
        assert sysinfo.run_cmd(['false']) == ''

    def test_missing_binary_is_empty(self, fake_run):
        fake_run.exc = FileNotFoundError('nope')
        assert sysinfo.run_cmd(['nope']) == ''


class TestDiagnosticRates:
    @staticmethod
    def _msg(*statuses):
        return SimpleNamespace(
            status=[
                SimpleNamespace(
                    name=name, values=[SimpleNamespace(key=k, value=v) for k, v in values]
                )
                for name, values in statuses
            ]
        )

    def test_extracts_the_actual_frequency(self):
        msg = self._msg(
            (
                'camera: color',
                [('Target frequency (Hz)', '60.0'), ('Actual frequency (Hz)', '59.4')],
            ),
            ('camera: Temperatures', [('Asic Temperature', '54')]),
        )
        out = {}
        sysinfo.read_diagnostic_rates(msg, out)
        assert out == {'/camera/color': pytest.approx(59.4)}

    def test_ignores_an_unparsable_rate(self):
        out = {}
        sysinfo.read_diagnostic_rates(
            self._msg(('camera: depth', [('Actual frequency (Hz)', 'n/a')])), out
        )
        assert out == {}

    def test_names_cover_the_three_realsense_streams(self):
        assert set(sysinfo.REALSENSE_DIAGNOSTIC_NAMES.values()) == {
            '/camera/color',
            '/camera/depth',
            '/imu/realsense',
        }


class TestMemory:
    MEMINFO = 'MemTotal:        8245972 kB\nMemFree:  100 kB\nMemAvailable:    6291456 kB\n'

    def test_mib_totals(self):
        assert sysinfo.parse_meminfo(self.MEMINFO) == {
            'total': 8052,
            'available': 6144,
            'used': 1908,
        }

    def test_missing_field_is_none(self):
        assert sysinfo.parse_meminfo('MemTotal: 1024 kB\n') is None


class TestCpuTimes:
    STAT = 'cpu  494969 599 69983 34490 4416 0 9467 0 0 0\ncpu0 1 2 3 4 5 6 7 8 0 0\n'

    def test_idle_and_iowait_are_not_busy(self):
        busy, total = sysinfo.parse_cpu_times(self.STAT)
        assert total == 494969 + 599 + 69983 + 34490 + 4416 + 9467
        assert busy == total - 34490 - 4416

    def test_guest_fields_are_not_double_counted(self):
        # guest (field 9) is already inside user.
        assert sysinfo.parse_cpu_times('cpu 10 0 0 90 0 0 0 0 7 0\n') == (10, 100)

    def test_malformed_is_none(self):
        assert sysinfo.parse_cpu_times('cpu x y\n') is None
        assert sysinfo.parse_cpu_times('intr 1 2\n') is None


class TestArmClock:
    def test_firmware_clock_in_mhz(self, fake_run):
        fake_run.result = _completed('frequency(0)=1000008576\n')
        current, _maximum = sysinfo.read_arm_clock()
        assert current == 1000
        assert fake_run.calls[0] == ['vcgencmd', 'measure_clock', 'arm']

    def test_no_vcgencmd(self, fake_run):
        fake_run.exc = FileNotFoundError('vcgencmd')
        assert sysinfo.read_arm_clock()[0] is None


class TestDisk:
    def test_gib_and_percent(self, monkeypatch):
        gib = 1024**3
        usage = SimpleNamespace(disk_usage=lambda _p: (100 * gib, 25 * gib, 75 * gib))
        monkeypatch.setattr(sysinfo, 'shutil', usage)
        assert sysinfo.read_disk('/') == {
            'total': 100,
            'used': 25,
            'free': 75,
            'percent_used': 25,
        }

    def test_unreadable_path_is_none(self):
        assert sysinfo.read_disk('/nonexistent/racecar/path') is None


class TestNtp:
    @pytest.mark.parametrize('stdout,expected', [('yes\n', True), ('no\n', False)])
    def test_synchronized_flag(self, fake_run, stdout, expected):
        fake_run.result = _completed(stdout)
        assert sysinfo.ntp_synchronized() is expected

    def test_timedatectl_failure_is_none(self, fake_run):
        fake_run.result = _completed('', returncode=1)
        assert sysinfo.ntp_synchronized() is None


class TestFormatUptime:
    @pytest.mark.parametrize(
        'seconds,expected',
        [(59, '0m'), (125, '2m'), (3 * 3600 + 5 * 60, '3h 5m'), (90061, '1d 1h 1m')],
    )
    def test_compact_form(self, seconds, expected):
        assert sysinfo.format_uptime(seconds) == expected
