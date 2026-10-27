"""Unit tests for racecar_neo_ros2_driver/pit_link.py."""

from racecar_neo_ros2_driver import pit_link
from racecar_neo_ros2_driver import pit_protocol as pit


def _stream(stats, start_us, count, step_us=5000, crc_ok=True):
    for i in range(count):
        stats.frame(start_us + i * step_us, crc_ok)


class TestCounts:
    def test_steady_stream(self):
        stats = pit_link.LinkStats(0.0)
        _stream(stats, 1000, 200)
        report = stats.report(1.0)
        assert report['level'] == pit_link.OK and report['message'] == '200 Hz'
        values = report['values']
        assert values[pit_link.RATE_KEY] == '200.0' and values['gap_ms'] == '5.0'
        assert values['frames'] == '200' and values['dropped_frames'] == '0'

    def test_gap_counts_the_missing_frames(self):
        stats = pit_link.LinkStats(0.0)
        stats.frame(0, True)
        stats.frame(30_000, True)  # five frames missing
        assert stats.counts['dropped_frames'] == 5
        report = stats.report(1.0)
        assert report['level'] == pit_link.WARN and report['values']['gap_ms'] == '30.0'

    def test_crc_failure_counts_and_warns(self):
        stats = pit_link.LinkStats(0.0)
        _stream(stats, 0, 3)
        stats.frame(15_000, False)
        assert stats.counts['crc_failures'] == 1
        assert stats.report(1.0)['level'] == pit_link.WARN

    def test_timestamp_going_back_is_a_reset(self):
        stats = pit_link.LinkStats(0.0)
        _stream(stats, 90_000_000, 3)
        stats.frame(2_000, True)
        assert stats.counts['resets'] == 1 and stats.counts['dropped_frames'] == 0

    def test_wrap_of_the_32_bit_clock_is_a_step(self):
        stats = pit_link.LinkStats(0.0)
        stats.frame((1 << 32) - 2_500, True)
        stats.frame(2_500, True)
        assert stats.counts['resets'] == 0 and stats.counts['dropped_frames'] == 0

    def test_reconnect_does_not_read_as_a_gap(self):
        stats = pit_link.LinkStats(0.0)
        stats.frame(0, True)
        stats.reconnect()
        stats.frame(5_000_000, True)
        assert stats.counts['reconnects'] == 1 and stats.counts['dropped_frames'] == 0

    def test_no_frames_is_an_error(self):
        report = pit_link.LinkStats(0.0).report(1.0)
        assert report['level'] == pit_link.ERROR and report['message'] == 'no frames'

    def test_window_starts_over_after_a_report(self):
        stats = pit_link.LinkStats(0.0)
        stats.frame(0, False)
        assert not stats.due(0.5) and stats.due(1.0)
        stats.report(1.0)
        _stream(stats, 5_000, 200)
        report = stats.report(2.0)
        assert report['level'] == pit_link.OK
        assert report['values']['crc_failures'] == '1'


class TestSkippedBytes:
    def test_garbage_before_a_frame_is_counted(self):
        stats = pit_link.LinkStats(0.0)
        frame = bytes(pit.TX_PACKET_SIZE)  # zeros: no magic
        buf = bytearray(b'\x01\x02' + frame)
        _, consumed = pit.scan_for_packet(buf)
        stats.skipped(consumed)
        assert stats.counts['skipped_bytes'] == consumed > 0
