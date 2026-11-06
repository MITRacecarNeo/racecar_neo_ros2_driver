"""Unit tests for racecar_neo_ros2_driver/pit_info.py."""

from racecar_neo_ros2_driver import pit_info
from racecar_neo_ros2_driver.pit_protocol import InfoChunk

PAYLOAD = (
    b'info=1\nfw.version=0.9.1\nfw.label=v0.9.1.10-pcb\nfw.git=574e229\n'
    b'fw.git_dirty=0\nfw.speed_control=closed_loop\nesc.pwm_min_us=1000\n'
    b'pid.kp=20.000\nnew.key=x=y\n'
)


def _chunks(payload, size=40, revision=1):
    starts = range(0, len(payload), size)
    parts = [payload[i:][:size] for i in starts]
    return [InfoChunk(revision, len(payload), i, len(parts), p) for i, p in enumerate(parts)]


def _known(payload=PAYLOAD):
    info = pit_info.FirmwareInfo(0.0)
    for chunk in _chunks(payload):
        info.add(chunk)
    assert info.state == pit_info.KNOWN
    return info


class TestParse:
    def test_values_stay_text(self):
        values = pit_info.parse_info(PAYLOAD)
        assert values['fw.version'] == '0.9.1' and values['pid.kp'] == '20.000'

    def test_value_may_hold_equals_and_bad_lines_are_skipped(self):
        values = pit_info.parse_info(b'a=1\nno separator\n=x\nb=c=d\n')
        assert values == {'a': '1', 'b': 'c=d'}


class TestAssembler:
    def test_joins_out_of_order(self):
        asm = pit_info.InfoAssembler()
        chunks = _chunks(PAYLOAD)
        results = [asm.add(c) for c in reversed(chunks)]
        assert results[-1] == PAYLOAD and all(r is None for r in results[:-1])

    def test_new_revision_restarts(self):
        asm = pit_info.InfoAssembler()
        old = _chunks(PAYLOAD, revision=1)
        new = _chunks(PAYLOAD, revision=2)
        assert asm.add(old[0]) is None
        assert asm.add(new[1]) is None
        # The revision-1 part was dropped, so revision 2 needs every part again.
        results = [asm.add(c) for c in new]
        assert results[-1] == PAYLOAD

    def test_length_mismatch_is_dropped(self):
        asm = pit_info.InfoAssembler()
        assert asm.add(InfoChunk(1, 99, 0, 1, b'a=1\n')) is None


class TestRequest:
    def test_requests_until_the_payload_is_in(self):
        info = pit_info.FirmwareInfo(0.0)
        assert info.requesting(0.5)
        chunks = _chunks(PAYLOAD)
        assert [info.add(c) for c in chunks][-1] is True
        assert not info.requesting(0.6)
        assert info.values['fw.git'] == '574e229'

    def test_timeout_gives_unknown(self):
        info = pit_info.FirmwareInfo(0.0, timeout_sec=2.0)
        assert info.requesting(1.9)
        assert not info.requesting(2.1)
        assert info.state == pit_info.UNKNOWN
        assert info.report()['level'] == pit_info.WARN

    def test_late_chunks_are_ignored(self):
        info = pit_info.FirmwareInfo(0.0)
        info.requesting(5.0)
        assert info.add(_chunks(b'a=1\n')[0]) is False
        assert info.state == pit_info.UNKNOWN

    def test_restart_asks_again(self):
        info = _known()
        info.restart(10.0)
        assert info.requesting(10.5) and info.values == {}


class TestReport:
    def test_pending(self):
        report = pit_info.FirmwareInfo(0.0).report()
        assert report['level'] == pit_info.OK and report['values'] == {'state': 'pending'}

    def test_release_build(self):
        report = _known().report()
        assert report['level'] == pit_info.OK and report['message'] == '0.9.1'
        values = report['values']
        assert values['state'] == 'known' and values['fw.label'] == 'v0.9.1.10-pcb'
        assert values['esc.pwm_min_us'] == '1000'
        assert 'pid.kp' not in values and 'new.key' not in values

    def test_raw_pwm_build_warns(self):
        report = _known(PAYLOAD.replace(b'closed_loop', b'raw_pwm')).report()
        assert report['level'] == pit_info.WARN
        assert report['message'] == '0.9.1: open-loop PWM test build'

    def test_unlisted_speed_control_warns(self):
        report = _known(PAYLOAD.replace(b'closed_loop', b'other')).report()
        assert report['level'] == pit_info.WARN and 'other' in report['message']

    def test_dirty_build_warns(self):
        report = _known(PAYLOAD.replace(b'git_dirty=0', b'git_dirty=1')).report()
        assert report['level'] == pit_info.WARN and 'uncommitted' in report['message']

    def test_summary(self):
        assert _known().summary() == '0.9.1 (v0.9.1.10-pcb, 574e229, closed_loop)'
