"""Unit tests for calibrate_lidar.py: wall fit and yaw correction on synthetic scans."""

import math

from conftest import load_script as _load
import numpy as np
import pytest
import yaml

N_BEAMS = 1080
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / N_BEAMS
LASER_X, LASER_Y = 0.2383, 0.0


@pytest.fixture(scope='module')
def cal():
    return _load('calibrate_lidar')


@pytest.fixture(scope='module')
def common():
    return _load('calibrate_common')


def _wall_scan(
    true_yaw_deg: float, wall_x: float, half_width: float = 1.5, noise: float = 0.0, seed: int = 0
):
    """Ranges a lidar at (LASER_X, LASER_Y) sees of a wall at base x = wall_x."""
    rng = np.random.default_rng(seed)
    ranges = np.full(N_BEAMS, np.inf)
    for i in range(N_BEAMS):
        heading = math.radians(true_yaw_deg) + ANGLE_MIN + i * ANGLE_INC
        dx = math.cos(heading)
        if dx <= 1e-6:
            continue
        r = (wall_x - LASER_X) / dx
        if abs(LASER_Y + r * math.sin(heading)) <= half_width:
            ranges[i] = r + (rng.normal(0.0, noise) if noise else 0.0)
    return ranges


def _fit(cal, ranges, assumed_yaw_deg=180.0):
    pts = cal.scan_points(ranges, ANGLE_MIN, ANGLE_INC, 0.15, 12.0)
    base = cal.to_base(pts, LASER_X, LASER_Y, math.radians(assumed_yaw_deg))
    return cal.fit_wall(cal.wall_sector(base, LASER_X, LASER_Y))


class TestWallFit:
    def test_square_wall_fits_at_zero(self, cal):
        fit = _fit(cal, _wall_scan(180.0, 1.2))
        assert fit.angle_deg == pytest.approx(0.0, abs=1e-6)
        assert fit.distance_mm == pytest.approx(1200.0)
        assert fit.rms_mm == pytest.approx(0.0, abs=1e-6)

    @pytest.mark.parametrize('true_yaw', [180.7, 178.8, 183.0])
    def test_recovers_the_true_yaw(self, cal, true_yaw):
        fit = _fit(cal, _wall_scan(true_yaw, 1.0))
        assert cal.corrected_yaw_deg(180.0, fit) == pytest.approx(true_yaw, abs=1e-3)

    def test_noise_averages_out_over_a_collection(self, cal):
        # 5 s at 7.2 Hz is 36 scans; 5 mm range noise per return.
        scans = [list(_wall_scan(180.4, 1.0, noise=0.005, seed=s)) for s in range(36)]
        fit = _fit(cal, cal.median_ranges(scans))
        assert cal.corrected_yaw_deg(180.0, fit) == pytest.approx(180.4, abs=0.05)
        assert fit.rms_mm < 5.0

    def test_sector_keeps_only_points_ahead(self, cal):
        pts = np.array([[1.5, 0.0], [0.0, 0.0], [1.5, 2.0]])
        kept = cal.wall_sector(pts, LASER_X, LASER_Y)
        assert kept.tolist() == [[1.5, 0.0]]

    def test_too_few_points(self, cal):
        assert cal.fit_wall(np.zeros((cal.MIN_WALL_POINTS - 1, 2))) is None

    def test_returns_beyond_range_limits_are_dropped(self, cal):
        ranges = np.array([0.1, 1.0, 13.0, np.inf, np.nan])
        assert len(cal.scan_points(ranges, 0.0, 0.01, 0.15, 12.0)) == 1


class TestMedianRanges:
    def test_median_per_beam_ignoring_inf(self, cal):
        got = cal.median_ranges([[1.0, np.inf], [3.0, 2.0], [2.0, 4.0]])
        assert got.tolist() == [2.0, 3.0]

    def test_odd_length_scans_dropped(self, cal):
        got = cal.median_ranges([[1.0, 1.0], [5.0], [3.0, 3.0]])
        assert got.tolist() == [2.0, 2.0]

    def test_no_scans(self, cal):
        assert cal.median_ranges([]) is None


class TestUpdateLocalValues:
    def test_keeps_other_keys(self, common, tmp_path):
        (tmp_path / 'frames.local.yaml').write_text(
            'camera_pitch_deg: 1.5\nlaser_yaw_deg: 180.0\n'
        )
        (path,) = common.update_local_values(
            'frames', {'laser_yaw_deg': 179.6}, [tmp_path], 'test'
        )
        text = path.read_text()
        assert text.startswith('#')
        assert yaml.safe_load(text) == {'camera_pitch_deg': 1.5, 'laser_yaw_deg': 179.6}

    def test_creates_the_file(self, common, tmp_path):
        (path,) = common.update_local_values('frames', {'laser_yaw_deg': 180.2}, [tmp_path], 't')
        assert yaml.safe_load(path.read_text()) == {'laser_yaw_deg': 180.2}
