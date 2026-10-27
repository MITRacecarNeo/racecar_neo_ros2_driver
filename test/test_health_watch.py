"""Unit tests for scripts/health_watch.py and the shake and poke steps in scripts/health.py."""

from conftest import load_script
import pytest
import yaml

watch = load_script('health_watch')
health = load_script('health')


def _steps():
    return [
        watch.Step('still', 'Hold the car still.', 5, start=0.0, end=5.0),
        watch.Step('left and right', 'Shake.', 10, start=10.0, end=20.0),
        watch.Step('up and down', 'Shake.', 10, start=30.0, end=40.0),
    ]


class TestKernelEvents:
    @pytest.mark.parametrize(
        'line,kind',
        [
            ('usb 4-2.3: USB disconnect, device number 3', 'usb'),
            ('usb 4-2.3: reset high-speed USB device number 5 using xhci-hcd', 'usb'),
            ('hwmon hwmon1: Undervoltage detected!', 'under-voltage'),
            ('nvme nvme0: I/O tag 591 (824f) QID 3 timeout, completion polled', 'nvme'),
            ('pcieport 0000:00:00.0: AER: Corrected error received', 'pcie'),
        ],
    )
    def test_faults(self, line, kind):
        assert watch.kernel_event(line + '\n') == (kind, line)

    @pytest.mark.parametrize(
        'line',
        [
            'usb 4-2.3: new high-speed USB device number 5 using xhci-hcd',
            'brcm-pcie 1000110000.pcie: link up, 5.0 GT/s PCIe x1 (!SSC)',
            'pcieport 0000:00:00.0: AER: enabled with IRQ 42',
        ],
    )
    def test_ordinary_lines(self, line):
        assert watch.kernel_event(line) is None


class TestThrottle:
    def test_only_new_bits_and_live_bits(self):
        assert watch.new_throttle_bits(0x50000, 0x50000) == 0
        assert watch.new_throttle_bits(0, 0x50005) == 0x50005
        assert watch.new_throttle_bits(0x50005, 0x50005) == 0x5


class TestSteps:
    def test_run_steps_times_each_step(self):
        t = [100.0]
        prompts, lines = [], []

        def sleep(sec):
            t[0] += sec

        steps = [watch.Step('still', 'Hold the car still.', 5), watch.Step('a', 'Shake.', 10)]
        watch.run_steps(steps, lambda: t[0], prompts.append, lines.append, sleep)
        assert [(s.start, s.end) for s in steps] == [(100.0, 105.0), (105.0, 115.0)]
        assert 'Step 1 of 2: Hold the car still.' in lines and 'Go. 10 seconds.' in lines
        assert prompts == ['Press Enter to start.'] * 2

    def test_events_charged_to_their_step(self):
        events = [
            watch.Event(12.0, 'usb', 'usb 4-2.3: USB disconnect, device number 3'),
            watch.Event(20.5, 'PIT link', '120 ms without a frame'),
            watch.Event(25.0, 'nvme', 'nvme nvme0: timeout'),
        ]
        ext5v = [(1.0, 4.9), (11.0, 4.86), (31.0, 4.88)]
        rows = {c.name: c for c in watch.step_checks('shake', _steps(), events, ext5v)}
        assert rows['still'].status == 'OK' and rows['still'].detail == 'min 4.900 V, no events'
        left = rows['left and right']
        assert left.status == 'FAIL'
        assert left.detail == (
            'min 4.860 V; 1 usb, 1 PIT link; first: usb usb 4-2.3: USB disconnect, device number 3'
        )
        assert rows['up and down'].status == 'OK'
        assert rows['between steps'].status == 'FAIL' and '1 nvme' in rows['between steps'].detail

    def test_scan_drop_against_the_still_rate(self):
        scan = [i * 0.15 for i in range(34)]  # about 6.7 Hz over the still step
        scan += [10.0 + i * 0.15 for i in range(14)]  # steady for 2 s...
        scan += [12.5, 15.0]  # ...then one scan a second or less
        drops = watch.rate_drops(_steps()[:2], scan, [])
        assert drops and drops[0].kind == 'sensor rate' and drops[0].detail.startswith('/scan')
        assert all(10.0 < d.t <= 20.0 for d in drops)

    def test_realsense_drop(self):
        rs = [(1.0, '/camera/color', 60.0), (3.0, '/camera/color', 59.0)]
        rs += [(12.0, '/camera/color', 58.0), (14.0, '/camera/color', 12.0)]
        drops = watch.rate_drops(_steps(), [], rs)
        assert [d.detail for d in drops] == ['/camera/color 12 Hz']

    def test_stream_still_down_is_charged_to_the_step_it_dropped_in(self):
        steps = _steps()
        scan = [i * 0.15 for i in range(34)]  # still step at about 6.7 Hz
        scan += [10.0 + i * 0.15 for i in range(14)]  # then gone from 12 s on
        drops = watch.rate_drops(steps, scan, [])
        assert len(drops) == 1 and 10.0 < drops[0].t <= 20.0

    def test_stream_that_came_back_can_drop_again(self):
        steps = _steps()
        scan = [i * 0.15 for i in range(34)]
        scan += [10.0 + i * 0.15 for i in range(14)]
        scan += [26.0 + i * 0.15 for i in range(28)]  # back from 26 s to 30.2 s, then gone
        drops = watch.rate_drops(steps, scan, [])
        assert [d.t <= 20.0 for d in drops] == [True, False]

    def test_realsense_already_down_is_not_charged_again(self):
        rs = [(1.0, '/camera/color', 60.0), (15.0, '/camera/color', 5.0)]
        rs += [(25.0, '/camera/color', 5.0), (33.0, '/camera/color', 5.0)]
        drops = watch.rate_drops(_steps(), [], rs)
        assert [d.t for d in drops] == [15.0]

    def test_steady_rates_have_no_drops(self):
        scan = [i * 0.15 for i in range(300)]
        rs = [(float(t), '/imu/realsense', 200.0) for t in range(45)]
        assert watch.rate_drops(_steps(), scan, rs) == []


class TestPokeList:
    def test_shipped_list(self):
        names = [c['name'] for c in health.poke_connectors()]
        assert names == [
            'battery',
            'buck output',
            'Pi 5 V input',
            'Teensy UART',
            'lidar USB',
            'RealSense USB-C',
            'NVMe ribbon',
        ]

    def test_car_profile_list_replaces_it(self, car_profile_dir):
        (car_profile_dir / 'params').mkdir(parents=True)
        own = {'poke': [{'name': 'servo', 'where': 'the servo plug on the PIT board'}]}
        (car_profile_dir / 'params' / 'health.yaml').write_text(yaml.safe_dump(own))
        assert health.poke_connectors() == own['poke']

    def test_poke_steps_start_still(self):
        steps = health.poke_steps([{'name': 'fan', 'where': "the fan's plug on the Pi"}])
        assert [s.name for s in steps] == ['still', 'fan']
        assert steps[1].prompt == "Press and wiggle the fan's plug on the Pi."
        assert steps[1].seconds == health.POKE_SEC

    def test_shake_steps_start_still(self):
        assert [s.name for s in health.shake_steps()][:2] == ['still', 'left and right']


class TestMotionReport:
    def test_events_table_in_the_report(self, tmp_path):
        checks = [health.Check('poke', 'lidar USB', 'FAIL', 'min 4.880 V; 1 usb')]
        events = [{'t': 31.2, 'step': 'lidar USB', 'kind': 'usb', 'detail': 'USB disconnect'}]
        health.write_report(tmp_path, 'now', 'car1', checks, {'events': events})
        md = (tmp_path / 'report.md').read_text()
        assert '## Poke test' in md and '## Events' in md
        assert '| 31.20 | lidar USB | usb | USB disconnect |' in md

    def test_teleop_stopped_refuses(self, monkeypatch, tmp_path):
        class Idle:
            link_count = 0

            def __init__(self, _dir):
                pass

            def start(self):
                pass

            def stop(self):
                pass

        monkeypatch.setattr(health.health_watch, 'Watch', Idle)
        monkeypatch.setattr(health, 'TELEOP_WAIT_SEC', 0.2)
        checks, extra = health.run_motion('shake', [], health.shake_steps(), tmp_path)
        assert [(c.name, c.status) for c in checks] == [('teleop', 'FAIL')]
        assert 'racecar service start teleop' in checks[0].detail and extra == {}


class TestLinkEvents:
    def test_new_counts_become_events(self):
        before = {'crc_failures': 1, 'reconnects': 0, 'resets': 0}
        after = {'crc_failures': 3, 'reconnects': 1, 'resets': 0}
        assert watch.link_events(before, after) == ['2 CRC failures', '1 reconnect']

    def test_first_report_has_no_events(self):
        assert watch.link_events({}, {'crc_failures': 9}) == []
