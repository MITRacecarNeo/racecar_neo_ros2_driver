"""Unit tests for scripts/health.py (`racecar health`)."""

import json
from types import SimpleNamespace

from conftest import load_script
import pytest

health = load_script('health')


def _clock(step):
    t = [0.0]

    def clock():
        t[0] += step
        return t[0]

    return clock


class TestPower:
    def test_samples_until_the_time_is_up(self):
        samples = health.sample_ext5v(1.0, read=lambda: 4.9, clock=_clock(0.25))
        assert samples == [(0.25, 4.9), (0.5, 4.9), (0.75, 4.9)]

    def test_stops_when_vcgencmd_fails(self):
        assert health.sample_ext5v(1.0, read=lambda: None, clock=_clock(0.1)) == []

    @pytest.mark.parametrize(
        'volts,status', [([4.9, 4.88], 'OK'), ([4.9, 4.8], 'WARN'), ([4.9, 4.7, 4.9], 'FAIL')]
    )
    def test_ext5v_row(self, volts, status):
        row = health.ext5v_row([(i * 0.01, v) for i, v in enumerate(volts)], 30)
        assert row.status == status

    def test_ext5v_row_reports_the_share_under_the_minimum(self):
        row = health.ext5v_row([(0, 4.9), (0.1, 4.7), (0.2, 4.7), (0.3, 4.9)], 30)
        assert '50.0% under 4.75 V' in row.detail and 'min 4.700 V' in row.detail

    def test_no_samples_skips(self):
        assert health.ext5v_row([], 30).status == 'SKIP'

    @pytest.mark.parametrize(
        'before,after,status',
        [
            (0, 0, 'OK'),
            (0x50000, 0x50000, 'WARN'),
            (0, 0x50000, 'FAIL'),
            (0x50000, 0x50005, 'FAIL'),
            (None, None, 'SKIP'),
        ],
    )
    def test_throttle_row(self, before, after, status):
        assert health.throttle_row(before, after).status == status

    def test_battery_over_the_floor(self):
        row = health.battery_row([(7.4, 2.5), (7.3, 2.6)], 3.55)
        assert row.status == 'OK' and 'floor 7.10 V (2S)' in row.detail

    def test_battery_under_the_floor_fails(self):
        assert health.battery_row([(7.4, 2.5), (7.05, 3.0)], 3.55).status == 'FAIL'

    def test_battery_without_teleop_skips(self):
        assert health.battery_row([], 3.55).status == 'SKIP'


class TestNvme:
    def test_speed_test_writes_reads_and_removes(self, tmp_path):
        write, read = health.speed_test(tmp_path, size_mb=8, chunk_mb=4)
        assert write > 0 and read > 0
        assert not (tmp_path / health.SPEED_FILE).exists()

    def test_speed_row_flags_a_mismatch(self, tmp_path):
        def bad(_folder):
            raise ValueError('read back differs at 4 MB')

        row = health.speed_row(tmp_path, bad)
        assert row.status == 'FAIL' and 'differs' in row.detail

    def test_slow_drive_warns(self, tmp_path):
        assert health.speed_row(tmp_path, lambda f: (40.0, 400.0)).status == 'WARN'
        assert health.speed_row(tmp_path, lambda f: (370.0, 410.0)).status == 'OK'

    def test_speed_skips_off_the_drive(self):
        assert health.speed_row(None, lambda f: (1.0, 1.0)).status == 'SKIP'

    def test_unsafe_shutdowns_since_the_last_report(self):
        last = {'time': '2026-10-26 10:00:00', 'smart': {'unsafe_shutdowns': 4}}
        assert health.unsafe_row({'unsafe_shutdowns': 4}, last).status == 'OK'
        row = health.unsafe_row({'unsafe_shutdowns': 6}, last)
        assert row.status == 'OK' and '2 new since 2026-10-26 10:00:00' in row.detail

    def test_first_report_has_nothing_to_compare(self):
        row = health.unsafe_row({'unsafe_shutdowns': 4}, None)
        assert row.status == 'OK' and 'no earlier report' in row.detail

    def test_kernel_log_clean(self):
        log = 'brcm-pcie 1000110000.pcie: link up, 5.0 GT/s PCIe x1\nAER: enabled with IRQ 42\n'
        assert health.kernel_row(log).status == 'OK'

    @pytest.mark.parametrize(
        'line',
        [
            'nvme nvme0: I/O tag 965 (83c5) QID 1 timeout, completion polled',
            'nvme nvme0: resetting controller',
            'pcieport 0000:00:00.0: AER: Corrected error received: 0000:04:00.0',
            'pcieport 0000:00:00.0: PCIe Bus Error: severity=Corrected',
        ],
    )
    def test_kernel_log_errors_fail(self, line):
        row = health.kernel_row(f'boot\n{line}\n')
        assert row.status == 'FAIL' and line in row.detail

    def test_no_drive_fails_alone(self, monkeypatch, tmp_path):
        monkeypatch.setattr(health.sysinfo, 'NVME_SYS', tmp_path / 'absent')
        checks, extra = health.run_nvme(None)
        assert [c.status for c in checks] == ['FAIL'] and extra == {}

    def test_fallback_to_the_sd_card_fails(self, monkeypatch, tmp_path):
        from racecar_neo_ros2_driver import data_root

        (tmp_path / 'model').write_text('CT500P310SSD8\n')
        monkeypatch.setattr(health.sysinfo, 'NVME_SYS', tmp_path)
        monkeypatch.setattr(health.sysinfo, 'read_nvme_smart', lambda: None)
        monkeypatch.setattr(health.sysinfo, 'read_nvme_temp', lambda: 50.0)
        monkeypatch.setattr(
            data_root, 'current', lambda: (tmp_path, 'Data goes to ~/data on the SD card.')
        )
        run = lambda argv, **kw: SimpleNamespace(returncode=0, stdout='')  # noqa: E731
        checks, _ = health.run_nvme(None, run=run, test=lambda f: (400.0, 400.0))
        rows = {c.name: c for c in checks}
        assert rows['drive'].detail == 'CT500P310SSD8'
        assert rows['data root'].status == 'FAIL'
        assert rows['speed'].status == 'SKIP'
        assert rows['kernel log'].status == 'OK'


class TestReport:
    def _checks(self):
        return [
            health.Check('power', 'ext5v', 'OK', 'min 4.882 V'),
            health.Check('nvme', 'kernel log', 'FAIL', 'nvme nvme0: timeout'),
        ]

    def test_writes_three_files(self, tmp_path):
        extra = {'ext5v_samples': [(0.0, 4.9), (0.01, 4.89)], 'smart': {'unsafe_shutdowns': 3}}
        health.write_report(tmp_path / 'r', '2026-10-26 16:28:50', 'car1', self._checks(), extra)
        report = json.loads((tmp_path / 'r' / 'report.json').read_text())
        assert report['result'] == 'FAIL' and report['car'] == 'car1'
        assert report['smart'] == {'unsafe_shutdowns': 3}
        assert report['checks'][1]['name'] == 'kernel log'
        md = (tmp_path / 'r' / 'report.md').read_text()
        assert '## Power' in md and '## Data drive' in md
        assert '| kernel log | FAIL | nvme nvme0: timeout |' in md
        csv = (tmp_path / 'r' / 'samples.csv').read_text().splitlines()
        assert csv == ['time_s,ext5v_v', '0.0000,4.90000', '0.0100,4.89000']

    def test_last_report_with_smart(self, tmp_path):
        for name, smart in (('20261025_100000', {'unsafe_shutdowns': 1}), ('20261026_100000', {})):
            (tmp_path / name).mkdir()
            (tmp_path / name / 'report.json').write_text(json.dumps({'smart': smart}))
        (tmp_path / 'battery').mkdir()
        assert health.last_report(tmp_path, 'smart') == {'smart': {'unsafe_shutdowns': 1}}

    def test_render_groups_by_step(self):
        lines = health.render_lines(self._checks())
        assert lines[1] == 'POWER' and lines[4] == 'NVME'


class TestLink:
    def _report(self, **kw):
        base = {'rate_hz': 200.0, 'gap_ms': 5.0, 'frames': 1000, 'dropped_frames': 0}
        return {**base, 'crc_failures': 0, 'skipped_bytes': 0, 'reconnects': 0, 'resets': 0, **kw}

    def test_clean_link_passes(self):
        rows = health.link_rows([self._report(), self._report(frames=1200)])
        assert all(c.status == 'OK' for c in rows)
        assert rows[0].detail.startswith('mean 200.0 Hz')

    def test_counts_are_new_during_the_sample(self):
        rows = {
            c.name: c
            for c in health.link_rows(
                [
                    self._report(dropped_frames=5, crc_failures=1),
                    self._report(dropped_frames=7, crc_failures=1),
                ]
            )
        }
        assert (rows['lost frames'].status, rows['lost frames'].detail) == ('WARN', '2')
        assert rows['crc failures'].status == 'OK'

    def test_faults(self):
        rows = {
            c.name: c
            for c in health.link_rows(
                [
                    self._report(),
                    self._report(gap_ms=80.0, crc_failures=1, resets=1, rate_hz=150.0),
                ]
            )
        }
        assert rows['longest gap'].status == 'FAIL'
        assert rows['crc failures'].status == 'FAIL'
        assert rows['teensy resets'].status == 'FAIL'
        assert rows['frame rate'].status == 'WARN'

    def test_no_reports(self):
        rows = health.link_rows([])
        assert [(c.name, c.status) for c in rows] == [('teensy link', 'FAIL')]

    def test_all_runs_link(self):
        assert health.DEFAULT_STEPS == ('power', 'link', 'nvme')
