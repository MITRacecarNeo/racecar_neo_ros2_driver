"""Unit tests for teensy_clock: unwrap, delay filtering, drift, restarts."""

import numpy as np
import pytest

from racecar_neo_ros2_driver.teensy_clock import TeensyClock

FRAME_US = 6000  # about 165 Hz
BASE_NS = 1_791_160_000 * 10**9  # an arbitrary ROS epoch


def _stream(n, start_us=0, delay_ms=(1.0, 17.0), seed=0, drift_ppm=0.0, epoch_ns=BASE_NS):
    """Yield (timestamp_us as signed i32, receive_ns, true_ns) for n frames."""
    rng = np.random.default_rng(seed)
    for i in range(n):
        teensy_us = start_us + i * FRAME_US
        true_ns = epoch_ns + int(i * FRAME_US * 1000 * (1 + drift_ppm * 1e-6))
        receive_ns = true_ns + int(rng.uniform(*delay_ms) * 1e6)
        signed = ((teensy_us + 2**31) % 2**32) - 2**31
        yield signed, receive_ns, true_ns


def _run(clock, frames):
    return [(clock.stamp(ts, rx), rx, true) for ts, rx, true in frames]


class TestDelayFiltering:
    def test_stamps_track_sample_time_not_arrival(self):
        out = _run(TeensyClock(), _stream(3000))
        late = out[200:]  # after the window holds a near-minimum delay
        stamp_err = np.array([s - t for s, _, t in late]) / 1e6
        receive_err = np.array([rx - t for _, rx, t in late]) / 1e6
        # Stamps sit at the minimum delay (about 1 ms) with sub-ms spread;
        # receive times spread across the full 1 to 17 ms.
        assert stamp_err.std() < 0.05
        assert stamp_err.mean() == pytest.approx(1.0, abs=0.2)
        assert receive_err.std() > 3.0

    def test_stamp_spacing_is_the_frame_period(self):
        out = _run(TeensyClock(), _stream(1000))
        gaps = np.diff([s for s, _, _ in out[200:]]) / 1e6
        assert gaps.mean() == pytest.approx(FRAME_US / 1000, abs=1e-3)
        assert gaps.max() - gaps.min() < 0.2

    def test_never_moves_backwards(self):
        out = _run(TeensyClock(), _stream(3000, seed=3))
        assert all(b >= a for (a, _, _), (b, _, _) in zip(out, out[1:], strict=False))


class TestUnwrap:
    def test_signed_wrap_is_continuous(self):
        clock = TeensyClock()
        start = 2**31 - 200 * FRAME_US  # crosses the i32 boundary mid-stream
        out = _run(clock, _stream(400, start_us=start))
        gaps = np.diff([s for s, _, _ in out[100:]]) / 1e6
        assert clock.restarts == 0
        assert gaps.max() < FRAME_US / 1000 + 0.5

    def test_unsigned_wrap_is_continuous(self):
        clock = TeensyClock()
        out = _run(clock, _stream(400, start_us=2**32 - 200 * FRAME_US))
        assert clock.restarts == 0
        assert np.diff([s for s, _, _ in out]).min() >= 0


class TestRestart:
    def test_teensy_reset_restarts_the_mapping(self):
        clock = TeensyClock()
        first = list(_stream(500, start_us=3_000_000_000))
        last_true = first[-1][2]
        second = list(_stream(500, start_us=0, epoch_ns=last_true + FRAME_US * 1000, seed=1))
        out = _run(clock, first + second)
        assert clock.restarts == 1
        # Stamps stay on the receive timeline; no half-hour jump.
        errs = np.array([s - t for s, _, t in out]) / 1e6
        assert np.abs(errs).max() < 20.0

    def test_receive_clock_step_restarts(self):
        clock = TeensyClock()
        frames = list(_stream(600))
        stepped = [
            (ts, rx + (2 * 10**9 if i >= 300 else 0), t) for i, (ts, rx, t) in enumerate(frames)
        ]
        out = _run(clock, stepped)
        assert clock.restarts == 1
        # After the step, stamps follow the new receive timeline.
        assert out[-1][0] > frames[-1][2] + 10**9

    def test_jitter_alone_never_restarts(self):
        clock = TeensyClock()
        _run(clock, _stream(5000, delay_ms=(1.0, 40.0), seed=7))
        assert clock.restarts == 0


class TestDrift:
    def test_few_ppm_drift_stays_tracked(self):
        out = _run(TeensyClock(), _stream(20000, drift_ppm=20.0))  # 2 min of frames
        errs = np.array([s - t for s, _, t in out[-2000:]]) / 1e6
        assert errs.mean() == pytest.approx(1.0, abs=0.5)
