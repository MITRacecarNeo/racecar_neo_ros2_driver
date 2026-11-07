"""Unit tests for racecar_neo_ros2_driver/pit_stall.py."""

import math

import pytest
from racecar_neo_ros2_driver import pit_stall


class TestDecode:
    def test_contract_fields(self):
        s = pit_stall.decode((2.0, 1.43, 4.0))
        assert (s.state, s.seconds_left, s.trips, s.name) == (2, pytest.approx(1.4), 4, 'cooldown')

    def test_older_firmware_zeros_are_normal(self):
        assert pit_stall.decode((0.0, 0.0, 0.0)) == pit_stall.StallState()

    @pytest.mark.parametrize('bad', [math.nan, math.inf, -1.0, 1e9])
    def test_bad_values_read_as_zero(self, bad):
        s = pit_stall.decode((bad, bad, bad))
        assert (s.state, s.seconds_left, s.trips) == (0, 0.0, 0)


class TestWatch:
    def test_reports_changes_only(self):
        watch = pit_stall.StallWatch()
        assert watch.update((0.0, 0.0, 0.0)) is None
        tripped = watch.update((1.0, 0.0, 1.0))
        assert tripped is not None and tripped.state == 1
        assert watch.update((1.0, 0.0, 1.0)) is None

    def test_countdown_publishes_in_tenths(self):
        watch = pit_stall.StallWatch()
        watch.update((2.0, 2.0, 1.0))
        assert watch.update((2.0, 1.995, 1.0)) is None
        assert watch.update((2.0, 1.9, 1.0)) is not None

    @pytest.mark.parametrize(
        'ekf, level, message',
        [
            ((0.0, 0.0, 0.0), pit_stall.OK, 'Normal'),
            (
                (1.0, 0.0, 1.0),
                pit_stall.WARN,
                'Tripped: return the throttle to neutral; 1 trip since boot',
            ),
            ((2.0, 1.6, 2.0), pit_stall.WARN, 'Cooldown, 2 s; 2 trips since boot'),
            ((3.0, 28.0, 3.0), pit_stall.ERROR, 'Lockout, 28 s; 3 trips since boot'),
        ],
    )
    def test_report(self, ekf, level, message):
        watch = pit_stall.StallWatch()
        watch.update(ekf)
        report = watch.report(True)
        assert (report['level'], report['message']) == (level, message)
        assert report['values']['supported'] == '1'

    def test_firmware_without_stall_protection(self):
        report = pit_stall.StallWatch().report(False)
        assert report['level'] == pit_stall.OK and report['message'] == 'Not in this firmware'
        assert pit_stall.StallWatch().report(None)['values']['supported'] == ''
