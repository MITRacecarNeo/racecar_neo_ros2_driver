"""Unit tests for scripts/calibrate_mag.py (ellipsoid fit and local YAML output)."""

from pathlib import Path

from conftest import load_script
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def mag():
    return load_script('calibrate_mag')


def _ellipsoid(n, offset, axes, radius=50e-6, seed=1):
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return (v * radius * np.asarray(axes) + np.asarray(offset)).tolist()


class TestFailurePath:
    def test_too_few_samples_is_no_fit(self, mag):
        samples = _ellipsoid(mag.MIN_SAMPLES - 1, (0, 0, 0), (1, 1, 1))
        assert mag.fit_ellipsoid(samples) is None

    def test_too_few_samples_writes_nothing(self, mag, car_profile_dir):
        samples = _ellipsoid(mag.MIN_SAMPLES - 1, (1e-5, 0, 0), (1, 1, 1))
        assert mag.fit_and_write(samples) is None
        assert not car_profile_dir.exists()

    def test_degenerate_data_writes_nothing(self, mag, car_profile_dir):
        samples = [[1e-5, 2e-5, 3e-5]] * (mag.MIN_SAMPLES * 2)
        assert mag.fit_and_write(samples) is None
        assert not car_profile_dir.exists()

    def test_planar_data_writes_nothing(self, mag, car_profile_dir):
        # Yaw-only rotation leaves z unconstrained; that is not an ellipsoid.
        samples = [[5e-5 * np.cos(t), 5e-5 * np.sin(t), 0.0] for t in np.linspace(0, 6, 300)]
        assert mag.fit_and_write(samples) is None
        assert not car_profile_dir.exists()


class TestFit:
    OFFSET = (1.0e-5, -5.0e-6, 3.0e-6)
    AXES = (1.2, 1.0, 0.8)

    def test_recovers_hard_iron_offset(self, mag):
        fit = mag.fit_ellipsoid(_ellipsoid(500, self.OFFSET, self.AXES))
        assert fit is not None
        assert fit.hard_iron == pytest.approx(self.OFFSET, abs=1e-9)

    def test_correction_maps_ellipsoid_to_sphere(self, mag):
        samples = _ellipsoid(500, self.OFFSET, self.AXES)
        fit = mag.fit_ellipsoid(samples)
        corrected = (fit.soft_iron @ (np.asarray(samples) - fit.hard_iron).T).T
        norms = np.linalg.norm(corrected, axis=1)
        assert norms.std() / norms.mean() < 1e-6

    def test_offset_larger_than_the_field(self, mag):
        # Hard iron bigger than the field puts zero field outside the ellipsoid.
        offset, axes = (7.0e-5, -2.0e-5, 1.0e-5), (1.1, 1.0, 0.9)
        samples = _ellipsoid(2000, offset, axes)
        fit = mag.fit_ellipsoid(samples)
        assert fit is not None
        assert fit.hard_iron == pytest.approx(offset, abs=1e-9)
        corrected = (fit.soft_iron @ (np.asarray(samples) - fit.hard_iron).T).T
        norms = np.linalg.norm(corrected, axis=1)
        assert norms.std() / norms.mean() < 1e-6


class TestLocalYaml:
    def test_writes_the_profile(self, mag, car_profile_dir):
        result = mag.fit_and_write(_ellipsoid(500, (1e-5, 0, 0), (1, 1, 1)))
        assert result is not None
        _, written = result
        assert written == [car_profile_dir / 'params' / 'lsm9ds1_mag_cal.yaml']
        assert written[0].is_file()

    def test_writes_only_keys_pit_node_declares(self, mag, car_profile_dir):
        mag.fit_and_write(_ellipsoid(500, (1e-5, 0, 0), (1, 1, 1)))
        path = car_profile_dir / 'params' / 'lsm9ds1_mag_cal.yaml'
        doc = yaml.safe_load(path.read_text())
        tracked = yaml.safe_load((ROOT / 'config' / 'lsm9ds1_mag_cal.yaml').read_text())
        assert set(doc) == {'pit_node'}
        assert set(doc['pit_node']['ros__parameters']) == set(
            tracked['pit_node']['ros__parameters']
        )
        soft = doc['pit_node']['ros__parameters']['magnetometer.soft_iron_matrix.data']
        assert len(soft) == 9


def _turn(field_ut, tilt_deg=3.0, degrees=360.0, n=400, hard=(0.0, 0.0, 0.0), squash=1.0):
    """Field vectors (T) seen while the car turns about an axis tilted from z."""
    b = np.asarray(field_ut, float) * 1e-6
    t = np.radians(tilt_deg)
    axis = np.array([np.sin(t), 0.0, np.cos(t)])
    out = []
    for a in np.radians(np.linspace(0.0, degrees, n, endpoint=False)):
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rot = np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k
        v = rot.T @ b
        v[0] *= squash
        out.append((v + np.asarray(hard, float) * 1e-6).tolist())
    return out


def _status(result, name):
    return next(c.status for c in result.criteria() if c.name == name)


class TestCheckCircle:
    def test_good_calibration_passes(self, mag):
        result = mag.check_circle(_turn((20.0, 5.0, 40.0)))
        assert {c.status for c in result.criteria()} == {'PASS'}
        assert result.offset < 0.01 and result.roundness < 0.01
        assert result.field_ut == pytest.approx(np.linalg.norm((20.0, 5.0, 40.0)), rel=1e-6)

    def test_hard_iron_moves_the_center(self, mag):
        result = mag.check_circle(_turn((20.0, 5.0, 40.0), hard=(8.0, 0.0, 0.0)))
        assert result.offset > mag.CHECK_MAX_OFFSET
        assert _status(result, 'center offset') == 'FAIL'

    def test_soft_iron_squashes_the_circle(self, mag):
        result = mag.check_circle(_turn((20.0, 5.0, 40.0), squash=0.8))
        assert _status(result, 'roundness') == 'FAIL'

    def test_partial_turn(self, mag):
        result = mag.check_circle(_turn((20.0, 5.0, 40.0), degrees=200.0))
        assert _status(result, 'turn coverage') == 'FAIL'

    def test_field_strength_only_warns(self, mag):
        # A uniform scale error leaves the heading unchanged.
        result = mag.check_circle(_turn((100.0, 0.0, 60.0)))
        assert _status(result, 'field strength') == 'WARN'
        assert _status(result, 'center offset') == 'PASS'

    def test_raw_fit_is_reported_without_a_verdict(self, mag):
        good = mag.check_circle(_turn((20.0, 5.0, 40.0)))
        raw = mag.check_circle(_turn((20.0, 5.0, 40.0), hard=(30.0, 0.0, 0.0)))
        line = good.criteria(raw)[-1]
        assert (line.name, line.status, line.limit) == ('raw, for contrast', 'INFO', '')
        assert line.measured.endswith('uT')

    def test_too_few_samples(self, mag):
        assert mag.check_circle(_turn((20.0, 5.0, 40.0), n=mag.MIN_SAMPLES - 1)) is None
